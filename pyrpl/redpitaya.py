###############################################################################
#    pyrpl - DSP servo controller for quantum optics with the RedPitaya
#    Copyright (C) 2014-2016  Leonhard Neuhaus  (neuhaus@spectro.jussieu.fr)
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
###############################################################################

from . import redpitaya_client
from . import hardware_modules as rp
from .sshshell import SshShell
from .pyrpl_utils import get_unique_name_list_from_class_list, update_with_typeconversion
from .memory import MemoryTree
from .errors import ExpectedPyrplError
from .widgets.startup_widget import HostnameSelectorWidget

import logging
import os
import random
import hashlib
import socket
import time
from time import sleep
import numpy as np

from paramiko import SSHException
from scp import SCPClient, SCPException
from collections import OrderedDict
from qtpy import QtCore


class FpgaFlashError(RuntimeError):
    """The FPGA bitstream could not be loaded onto the board.

    Raised from update_fpga() during RedPitaya construction, so the GUI's
    connect-with-retry dialog shows the reason instead of the session
    continuing against whatever design happens to be in the PL (which only
    surfaces later as garbage register reads).
    """
    pass


class RedPitayaSignalLauncher(QtCore.QObject):
    """Carries board-level Qt signals. RedPitaya itself is a plain object (not a
    QObject), so cross-thread board events are routed through this QObject, which
    is created on the GUI thread. Slots connected with the default AutoConnection
    therefore run on the GUI thread even when the signal is emitted from a
    register-I/O worker thread."""
    # emitted once when the register link drops and reconnection gives up.
    # argument: a human-readable reason string.
    connection_lost = QtCore.Signal(str)
    # emitted after a successful (GUI-driven) reconnection.
    reconnected = QtCore.Signal()

# input is the wrong function in python 2
try:
    raw_input
except NameError:  # Python 3
    raw_input = input

# default parameters for redpitaya object creation
defaultparameters = dict(
    hostname='', #'192.168.1.100', # the ip or hostname of the board, '' triggers gui
    port=2222,  # port for PyRPL datacommunication
    sshport=22,  # port of ssh server - default 22
    user='root',
    password='root',
    delay=0.05,  # delay between ssh commands - console is too slow otherwise
    autostart=True,  # autostart the client?
    reloadserver=False,  # reinstall the server at startup if not necessary?
    reloadfpga=True,  # manage the fpga bitfile at startup? (now md5-gated: only
                      # uploads/flashes when the on-board bitstream differs or
                      # the board rebooted — see force_reload to override)
    force_reload=False,  # force a full FPGA reflash AND monitor_server restart
                         # even when the on-board md5 already matches and a
                         # server is running. Off => skip-on-match (lets several
                         # clients share one board without clobbering each other)
    serverbinfilename='fpga.bit.bin',  # name of the binfile on the server
    serverdirname = "/opt/pyrpl/",  # server directory for server app and bitfile
    leds_off=True,  # turn off all GPIO lets at startup (improves analog performance)
    frequency_correction=1.0,  # actual FPGA frequency is 125 MHz * frequency_correction
    timeout=1,  # timeout in seconds for ssh communication
    monitor_server_name='monitor_server',  # name of the server program on redpitaya
    silence_env=False,   # suppress all environment variables that may override the configuration?
    gui=True,  # show graphical user interface or work on command-line only?
    disabled_modules=[],  # module names absent from the FPGA bitstream, e.g.
                          # ['iir','pid1','pid2','iq1','iq2']. Listed modules are
                          # not instantiated, so the client never touches their
                          # (unmapped) register space. Accepts a list or a
                          # comma/space-separated string (for env/config use).
    reserve_dma_memory=True,  # verify on startup that the kernel leaves the DMA
                              # ring window (0x1E000000+) unmanaged (u-boot
                              # 'mem=480M'); if not, patch /boot/u-boot.scr on
                              # the board and REBOOT it once to apply. See
                              # docs/DmaStreaming.md §9. Set False to skip the
                              # check (e.g. boards without the DMA bitstream).
    reconnect_retries=-1,  # runtime register-link reconnection budget. When the
                           # live TCP register link drops (board reboot / cable
                           # pull / network blip) the client tries to reconnect.
                           # -1 (default) = retry forever (legacy behaviour). A
                           # positive N gives up after N failed reconnects, emits
                           # signal_launcher.connection_lost(reason) and aborts,
                           # so a GUI can prompt the user instead of the client
                           # spinning indefinitely.
    )


class RedPitaya(object):
    cls_modules = [rp.HK, rp.AMS, rp.Scope, rp.Sampler, rp.Asg0, rp.Asg1, rp.Asg2, rp.Asg3] + \
                  [rp.Pwm] * 4 + [rp.Iq] * 3 + [rp.Pid] * 3 + [rp.Trig] + [rp.IIR]

    def __init__(self, config=None,  # configfile is needed to store parameters. None simulates one
                 **kwargs):
        """ this class provides the basic interface to the redpitaya board

        The constructor installs and starts the communication interface on the RedPitaya
        at 'hostname' that allows remote control and readout

        'config' is the config file or MemoryTree of the config file. All keyword arguments
        may be specified in the branch 'redpitaya' of this config file. Alternatively,
        they can be overwritten by keyword arguments at the function call.

        'config=None' specifies that no persistent config file is saved on the disc.

        Possible keyword arguments and their defaults are:
            hostname='192.168.1.100', # the ip or hostname of the board
            port=2222,  # port for PyRPL datacommunication
            sshport=22,  # port of ssh server - default 22
            user='root',
            password='root',
            delay=0.05,  # delay between ssh commands - console is too slow otherwise
            autostart=True,  # autostart the client?
            reloadserver=False,  # reinstall the server at startup if not necessary?
            reloadfpga=True,  # reload the fpga bitfile at startup?
            filename='fpga//red_pitaya.bin',  # name of the bitfile for the fpga, None is default file
            serverbinfilename='fpga.bin',  # name of the binfile on the server
            serverdirname = "//opt//pyrpl//",  # server directory for server app and bitfile
            leds_off=True,  # turn off all GPIO lets at startup (improves analog performance)
            frequency_correction=1.0,  # actual FPGA frequency is 125 MHz * frequency_correction
            timeout=3,  # timeout in seconds for ssh communication
            monitor_server_name='monitor_server',  # name of the server program on redpitaya
            silence_env=False,   # suppress all environment variables that may override the configuration?
            gui=True  # show graphical user interface or work on command-line only?

        if you are experiencing problems, try to increase delay, or try
        logging.getLogger().setLevel(logging.DEBUG)"""
        self.logger = logging.getLogger(name=__name__)
        #self.license()
        # make or retrieve the config file
        if isinstance(config, MemoryTree):
            self.c = config
        else:
            self.c = MemoryTree(config)
        # get the parameters right (in order of increasing priority):
        # 1. defaults
        # 2. environment variables
        # 3. config file
        # 4. command line arguments
        # 5. (if missing information) request from GUI or command-line
        self.parameters = defaultparameters # BEWARE: By not copying the
        # dictionary, defaultparameters are modified in the session (which
        # can be advantageous for instance with hostname in unit_tests)

        # get parameters from os.environment variables
        if not self.parameters['silence_env']:
            for k in self.parameters.keys():
                if "REDPITAYA_"+k.upper() in os.environ:
                    newvalue = os.environ["REDPITAYA_"+k.upper()]
                    oldvalue = self.parameters[k]
                    if isinstance(oldvalue, list):
                        # list-valued params (e.g. disabled_modules) are given
                        # as a comma/space-separated string in the environment
                        self.parameters[k] = [s.strip() for s in
                                              newvalue.replace(',', ' ').split()]
                    else:
                        self.parameters[k] = type(oldvalue)(newvalue)
                    if k == "password": # do not show the password on the screen
                        oldvalue = "********"
                        newvalue = "********"
                    self.logger.debug("Variable %s with value %s overwritten "
                                      "by environment variable REDPITAYA_%s "
                                      "with value %s. Use argument "
                                      "'silence_env=True' if this is not "
                                      "desired!",
                                      k, oldvalue, k.upper(), newvalue)
        # settings from config file
        try:
            update_with_typeconversion(self.parameters, self.c._get_or_create('redpitaya')._data)
        except BaseException as e:
            self.logger.warning("An error occured during the loading of your "
                                "Red Pitaya settings from the config file: %s",
                                e)
        # settings from class initialisation / command line
        update_with_typeconversion(self.parameters, kwargs)
        # get missing connection settings from gui/command line
        if self.parameters['hostname'] is None or self.parameters['hostname']=='':
            gui = 'gui' not in self.c._keys() or self.c.gui
            if gui:
                self.logger.info("Please choose the hostname of "
                                 "your Red Pitaya in the hostname "
                                 "selector window!")
                startup_widget = HostnameSelectorWidget(config=self.parameters)
                hostname_kwds = startup_widget.get_kwds()
            else:
                hostname = raw_input('Enter hostname [192.168.1.100]: ')
                hostname = '192.168.1.100' if hostname == '' else hostname
                hostname_kwds = dict(hostname=hostname)
                if not "sshport" in kwargs:
                    sshport = raw_input('Enter sshport [22]: ')
                    sshport = 22 if sshport == '' else int(sshport)
                    hostname_kwds['sshport'] = sshport
                if not 'user' in kwargs:
                    user = raw_input('Enter username [root]: ')
                    user = 'root' if user == '' else user
                    hostname_kwds['user'] = user
                if not 'password' in kwargs:
                    password = raw_input('Enter password [root]: ')
                    password = 'root' if password == '' else password
                    hostname_kwds['password'] = password
            self.parameters.update(hostname_kwds)

        # optional: write configuration back to config file
        self.c["redpitaya"] = self.parameters

        # save default port definition for possible automatic port change
        self.parameters['defaultport'] = self.parameters['port']
        # frequency_correction is accessed by child modules
        self.frequency_correction = self.parameters['frequency_correction']
        # memorize whether server is running - nearly obsolete
        self._serverrunning = False
        self.client = None  # client class
        self._slaves = []  # slave interfaces to same redpitaya
        self.modules = OrderedDict()  # all submodules
        # QObject carrying board-level Qt signals (connection_lost / reconnected).
        # Created here on the GUI thread so cross-thread emission works.
        self.signal_launcher = RedPitayaSignalLauncher()

        # provide option to simulate a RedPitaya
        if self.parameters['hostname'] in ['_FAKE_REDPITAYA_', '_FAKE_']:
            self.startdummyclient()
            self.logger.warning("Simulating RedPitaya because (hostname=="
                                +self.parameters["hostname"]+"). Incomplete "
                                "functionality possible. ")
            return
        elif self.parameters['hostname'] in ['_NONE_']:
            self.modules = []
            self.logger.warning("No RedPitaya created (hostname=="
                                + self.parameters["hostname"] + ")."
                                " No hardware modules are available. ")
            return
        # connect to the redpitaya board
        self.start_ssh()
        # make sure the kernel leaves the FPGA DMA ring window unmanaged; may
        # patch u-boot.scr and reboot the board ONCE (before any flashing, so
        # the reboot doesn't waste a flash — the marker is boot-scoped anyway)
        if self.parameters.get('reserve_dma_memory', True):
            self._ensure_dma_mem_reservation()
        # if the PL was parked (held in reset / blanked by pl_reset.sh), release
        # it, drop the stale monitor_server, and reflash before anything else
        recovered_pl = self._recover_pl_reset()
        # start other stuff
        if self.parameters['reloadfpga'] and not recovered_pl:  # flash fpga
            self.update_fpga()
        if self.parameters['reloadserver']:  # reinstall server app
            self.installserver()
        if self.parameters['autostart']:  # start client
            self.start()
        self.logger.info('Successfully connected to Redpitaya with hostname '
                         '%s.'%self.ssh.hostname)
        self.parent = self

    def start_ssh(self, attempt=0):
        """
        Extablishes an ssh connection to the RedPitaya board

        returns True if a successful connection has been established
        """
        try:
            # close pre-existing connection if necessary
            self.end_ssh()
        except:
            pass
        if self.parameters['hostname'] == "_FAKE_REDPITAYA_":
            # simulation mode - start without connecting
            self.logger.warning("(Re-)starting client in dummy mode...")
            self.startdummyclient()
            return True
        else:  # normal mode - establish ssh connection and
            try:
                # start ssh connection
                self.ssh = SshShell(hostname=self.parameters['hostname'],
                                    sshport=self.parameters['sshport'],
                                    user=self.parameters['user'],
                                    password=self.parameters['password'],
                                    delay=self.parameters['delay'],
                                    timeout=self.parameters['timeout'])
                # test ssh connection for exceptions
                self.ssh.ask()
            except BaseException as e:  # connection problem
                if attempt < 3:
                    # try to connect up to 3 times
                    return self.start_ssh(attempt=attempt+1)
                else:  # even multiple attempts did not work
                    raise ExpectedPyrplError(
                        "\nCould not connect to the Red Pitaya device with "
                        "the following parameters: \n\n"
                        "\thostname: %s\n"
                        "\tssh port: %s\n"
                        "\tusername: %s\n"
                        "\tpassword: ****\n\n"
                        "Please confirm that the device is reachable by typing "
                        "its hostname/ip address into a web browser and "
                        "checking that a page is displayed. \n\n"
                        "Error message: %s" % (self.parameters["hostname"],
                                               self.parameters["sshport"],
                                               self.parameters["user"],
                                               e))
            else:
                # everything went well, connection is established
                # also establish scp connection
                self.ssh.startscp()
                return True

    def switch_led(self, gpiopin=0, state=False):
        self.ssh.ask("echo " + str(gpiopin) + " > /sys/class/gpio/export")
        sleep(self.parameters['delay'])
        self.ssh.ask(
            "echo out > /sys/class/gpio/gpio" +
            str(gpiopin) +
            "/direction")
        sleep(self.parameters['delay'])
        if state:
            state = "1"
        else:
            state = "0"
        self.ssh.ask("echo " + state + " > /sys/class/gpio/gpio" +
            str(gpiopin) + "/value")
        sleep(self.parameters['delay'])

    @staticmethod
    def _file_md5(path):
        """md5 hex digest of a local file."""
        with open(path, 'rb') as f:
            return hashlib.md5(f.read()).hexdigest()

    def _board_boot_id(self):
        """The board's current-boot UUID (changes on every reboot). Used to
        invalidate the 'fpga flashed' marker after a power-cycle, because the
        loaded PL is volatile and reverts to the boot-default bitstream."""
        _, out = self.ssh.run('cat /proc/sys/kernel/random/boot_id 2>/dev/null')
        return out.strip()

    def _remount_root_rw(self):
        """Remount the root fs read-write for an upload/flash/marker write.

        /opt/pyrpl lives on the root fs, which on legacy Red Pitaya images is
        kept read-only (SD-card protection). But newer images boot root rw (the
        kernel mounts ro, then systemd-remount-fs makes it rw per an fstab with
        no 'ro'). Forcing root back to ro afterwards strands /tmp and the whole
        rootfs read-only on those images, breaking every later write (incl. the
        next flash's marker) until reboot. So we cache the ORIGINAL state — first
        observed this connection, before pyrpl's own remounts corrupt it — and
        _restore_root_mount() only re-applies ro when it was genuinely ro.
        """
        if getattr(self, '_root_orig_ro', None) is None:
            _, opts = self.ssh.run("awk '$2==\"/\"{print $4; exit}' /proc/mounts")
            self._root_orig_ro = (opts.strip().split(',')[0] == 'ro')
        self.ssh.run('mount -o remount,rw /')

    def _restore_root_mount(self):
        """Restore the root fs to its original state: only remount read-only if
        it was read-only when first observed this connection (see _remount_root_rw)."""
        if getattr(self, '_root_orig_ro', None):
            self.ssh.run('mount -o remount,ro /')

    def _onboard_fpga_md5(self, serverbinfilename):
        """md5 of the bitstream file currently stored on the board ('' if none).
        Drives the scp decision: we only re-upload when this differs from the
        local bitstream (the file is kept on the board, not deleted after flash)."""
        _, out = self.ssh.run('md5sum ' + serverbinfilename + ' 2>/dev/null')
        out = out.strip()
        return out.split()[0] if out else ''

    def _fpga_flashed(self, serverbinfilename, md5):
        """True if the running PL was flashed from this exact bitstream during the
        current boot. Reads the '<bin>.version' marker ('<md5> <boot_id>'); a
        boot_id mismatch means a reboot reverted the FPGA, so a reflash is due."""
        _, marker = self.ssh.run('cat ' + serverbinfilename + '.version 2>/dev/null')
        parts = marker.split()
        return len(parts) >= 2 and parts[0] == md5 and parts[1] == self._board_boot_id()

    def _record_fpga_flashed(self, serverbinfilename, md5):
        """Write the '<bin>.version' marker with the just-flashed md5 + boot_id
        (caller holds the rw remount). Verify + retry once: the marker echo over
        pyrpl's interactive shell occasionally fails ('write error: Invalid
        argument') right after the flash; a dropped marker silently costs a
        redundant reflash next connect, so confirm it landed."""
        marker = '%s %s' % (md5, self._board_boot_id())
        for _ in range(2):
            self.ssh.run('echo %s > %s.version' % (marker, serverbinfilename))
            _, back = self.ssh.run('cat %s.version 2>/dev/null' % serverbinfilename)
            if back.strip() == marker:
                return
        self.logger.warning("Could not confirm the FPGA flash marker on the board; "
                            "the next connection may redundantly reflash.")

    def _scp_put_retry(self, src, dest):
        """scp a file to the board, retrying (with reconnect) up to 3 times."""
        for i in range(3):
            try:
                self.ssh.scp_put(src, dest)
            except (SCPException, SSHException):
                self.start_ssh()
                sleep(self.parameters['delay'])
            else:
                return

    # PS registers that report the PL (fabric) state, poked via the on-board
    # `monitor` tool (same ones pl_reset.sh uses to park the board for low power).
    _SLCR_FPGA_RST_CTRL = '0xF8000240'   # bits[3:0]: the 4 PL FCLK resets
    _DEVCFG_STATUS      = '0xF800700C'   # bit2 PCFG_DONE: 0 -> PL not configured

    def _read_ps_reg(self, addr):
        """Read a 32-bit PS register via /opt/redpitaya/bin/monitor. Returns the
        int value, or None if the read failed (tool missing, ssh hiccup)."""
        try:
            ret, out = self.ssh.run('/opt/redpitaya/bin/monitor ' + addr)
        except Exception:
            self.logger.debug('PS register read failed (%s)', addr, exc_info=True)
            return None
        if ret != 0:
            return None
        try:
            return int(out.strip().split()[0], 16)
        except (ValueError, IndexError):
            return None

    def _pl_held_in_reset(self):
        """True when the PL fabric is parked — held in reset (FPGA_RST_CTRL bits
        set) or blanked (devcfg PROG_B low) — so the custom FFT/DMA bitstream is
        not actually running. Best-effort: returns False when the state can't be
        read, so a monitor-tool-less board is never disrupted."""
        rst = self._read_ps_reg(self._SLCR_FPGA_RST_CTRL)
        sts = self._read_ps_reg(self._DEVCFG_STATUS)
        if rst is None or sts is None:
            return False
        held  = (rst & 0xf) != 0        # any FCLK reset asserted
        blank = (sts & 0x4) == 0        # PCFG_DONE low -> fabric not configured
        if held or blank:
            self.logger.warning(
                "PL fabric is %s (FPGA_RST_CTRL=0x%08x, devcfg STATUS=0x%08x); "
                "releasing and reflashing.",
                "held in reset" if held else "blank/unconfigured", rst, sts)
        return held or blank

    def _recover_pl_reset(self):
        """If the PL was parked (e.g. by pl_reset.sh to cut heat/power), release
        the reset, drop any monitor_server left spinning on the now-dead DMA
        fabric, and force a fresh FPGA flash so the bitstream is actually loaded.
        Returns True if a recovery was performed (the caller then skips its own
        update_fpga, which _recover already ran). Called on (re)connect."""
        if not self._pl_held_in_reset():
            return False
        # release the 4 PL FCLK resets (pl_reset.sh 'stop' asserts these)
        self.ssh.run('/opt/redpitaya/bin/monitor %s 0x0'
                     % self._SLCR_FPGA_RST_CTRL)
        # stop the server that was reading a reset/blank fabric (the DMA thread
        # spins in its framing re-align when the fabric produces no valid data)
        self._kill_monitor_server()
        # invalidate the boot-flash marker so update_fpga actually reflashes even
        # if it thinks the current bitstream was already flashed this boot
        serverbinfilename = os.path.join(self.parameters['serverdirname'],
                                         self.parameters['serverbinfilename'])
        self._remount_root_rw()
        self.ssh.run('rm -f %s.version' % serverbinfilename)
        self._restore_root_mount()
        self.update_fpga()   # reflashes (marker gone) and kills any server again
        return True

    # ---- DMA DDR reservation (docs/DmaStreaming.md §9) ----------------------
    # The FPGA DMA writes its point-cloud ring at a fixed physical address; the
    # kernel must not manage that RAM or the DMA and kernel corrupt each other.
    # The reservation is a one-line u-boot bootargs cap ('mem=480M') carried by
    # a PRE-BUILT /boot/u-boot.scr bundled with pyrpl (pyrpl/uboot/), one per
    # supported board revision. Nothing is built or patched at runtime — the
    # host (which may be Windows, no u-boot-tools) just uploads the matching
    # script; an unknown hw_rev only warns.
    _DMA_BUF_BASE  = 0x1E000000   # DMA ring base = 480 MiB
    _DMA_MEM_TOKEN = 'mem=480M'   # kernel RAM cap that frees 0x1E000000+
    _RAM_512M_TOP  = 0x1FFFFFFF   # unreserved 512 MiB board (the only layout
                                  # the fixed 480M cap is valid for)
    # board hw_rev (factory EEPROM, the value u-boot branches on) -> bundled
    # pre-built boot script (relative to pyrpl/uboot/). Add an entry only once
    # the script has been built AND boot-verified on that revision.
    # Gen 2 note: those branches also set 'high' (-> fdt_high/initrd_high),
    # the ceiling u-boot relocates the devicetree below. Stock is 0x20000000,
    # i.e. above the capped RAM top, which would leave the 480 MiB kernel
    # unable to reach its own DTB — the bundled Gen 2 script lowers it to
    # 0x1E000000 alongside the mem= change. Gen 1 branches set no 'high'.
    _UBOOT_PREBUILT = {
        'STEM_125-14_Z7020_LN_v1.1':  'u-boot.scr.STEM_125-14_Z7020_LN_v1.1',
        'STEM_125-14_Z7020_Pro_v2.0': 'u-boot.scr.STEM_125-14_Z7020_Pro_v2.0',
    }

    def _system_ram_top(self):
        """Highest 'System RAM' end address from the board's /proc/iomem, or
        None when it can't be read."""
        ret, out = self.ssh.run("grep 'System RAM' /proc/iomem")
        if ret != 0:
            return None
        top = None
        for line in out.splitlines():
            try:
                end = int(line.split(':')[0].strip().split('-')[1], 16)
            except (IndexError, ValueError):
                continue
            top = end if top is None else max(top, end)
        return top

    def _dma_mem_reserved(self):
        """True if the kernel's RAM ends below the DMA ring window, False if it
        covers it, None when the state can't be read."""
        top = self._system_ram_top()
        if top is None:
            return None
        return top < self._DMA_BUF_BASE

    def _board_hw_rev(self):
        """The board's hardware revision string from the factory EEPROM (the
        same value u-boot branches on), or '' when unreadable."""
        ret, out = self.ssh.run(
            "strings /sys/bus/i2c/devices/0-0050/eeprom 2>/dev/null"
            " | grep -a '^hw_rev='")
        if ret != 0:
            return ''
        return out.strip().splitlines()[0].partition('=')[2] if out.strip() else ''

    def _reboot_and_reconnect(self, timeout=180):
        """Reboot the board and poll for ssh to come back (True on success)."""
        self.logger.warning("Rebooting the Red Pitaya to apply the DMA memory "
                            "reservation...")
        try:
            self.ssh.run('reboot')
        except BaseException:
            pass  # the connection dropping mid-command is expected
        try:
            self.end_ssh()
        except BaseException:
            pass
        deadline = time.time() + timeout
        sleep(10)  # let it actually go down before probing
        while time.time() < deadline:
            try:
                self.start_ssh()
                return True
            except BaseException:
                sleep(5)
        self.logger.error("Board did not come back within %d s after the "
                          "reboot.", timeout)
        return False

    def _ensure_dma_mem_reservation(self):
        """Verify the kernel leaves the DMA ring window (0x1E000000+)
        unmanaged; if not, install the bundled PRE-BUILT /boot/u-boot.scr for
        this board's hw_rev and reboot ONCE to apply. No boot-script tooling
        is required on the host (may be Windows) or the board — the matching
        script is simply uploaded, verified by md5, and installed with a
        backup. Any check failing (unknown hw_rev, no bundled script,
        non-512 MiB layout) leaves the board untouched and
        warns with a pointer to the manual procedure.

        Returns True when the reservation is in place when we're done."""
        reserved = self._dma_mem_reserved()
        if reserved:
            self.logger.debug("DMA memory reservation in place (kernel RAM "
                              "ends below 0x%08X).", self._DMA_BUF_BASE)
            return True
        if reserved is None:
            self.logger.debug("Could not read /proc/iomem; skipping the DMA "
                              "memory reservation check.")
            return False
        manual = ("fix it manually per docs/DmaStreaming.md 'DDR reservation'"
                  " or set reserve_dma_memory=False to silence this.")
        top = self._system_ram_top()
        if top != self._RAM_512M_TOP:
            self.logger.warning(
                "Kernel RAM covers the DMA ring window but the board is not an "
                "unreserved 512 MiB layout (RAM top 0x%08X); the fixed %s cap "
                "does not apply — %s", top, self._DMA_MEM_TOKEN, manual)
            return False
        hw_rev = self._board_hw_rev()
        prebuilt = self._UBOOT_PREBUILT.get(hw_rev)
        if prebuilt is None:
            self.logger.warning(
                "DMA ring window is NOT reserved and no pre-built boot script "
                "is bundled for this board revision (hw_rev %r); only %s are "
                "covered so far. Leaving the board untouched; %s",
                hw_rev or '<unreadable>',
                sorted(self._UBOOT_PREBUILT), manual)
            return False
        local = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                             'uboot', prebuilt)
        if not os.path.isfile(local):
            self.logger.warning("Bundled boot script %s is missing from this "
                                "pyrpl installation; %s", local, manual)
            return False
        with open(local, 'rb') as f:
            data = f.read()
        if self._DMA_MEM_TOKEN.encode() not in data:
            self.logger.error("Bundled boot script %s does not carry the '%s' "
                              "reservation — refusing to install it; %s",
                              prebuilt, self._DMA_MEM_TOKEN, manual)
            return False
        md5 = hashlib.md5(data).hexdigest()
        _, onboard = self.ssh.run('md5sum /boot/u-boot.scr 2>/dev/null')
        onboard = onboard.split()[0] if onboard.strip() else ''
        if onboard == md5:
            # the right script is already installed but the running kernel
            # doesn't reflect it: a reboot is pending. Don't reboot
            # automatically here — if the script were ineffective this would
            # loop a reboot on every connect.
            self.logger.warning(
                "The matching boot script is already installed but the "
                "reservation is not active — reboot the board to apply it "
                "(not rebooting automatically to avoid a reboot loop).")
            return False
        self.logger.warning(
            "Kernel RAM covers the FPGA DMA ring window (no '%s' in "
            "bootargs); installing the pre-built boot script for hw_rev %s "
            "and rebooting the board.", self._DMA_MEM_TOKEN, hw_rev)
        # upload, then back up + install (/boot may be mounted read-only)
        self._scp_put_retry(local, '/tmp/u-boot.scr.pyrpl')
        ret, out = self.ssh.run(
            'mount -o remount,rw /boot && '
            'cp -a /boot/u-boot.scr /boot/u-boot.scr.bak-$(date +%Y%m%d-%H%M%S)'
            ' && cp /tmp/u-boot.scr.pyrpl /boot/u-boot.scr && sync'
            ' && mount -o remount,ro /boot')
        if ret != 0:
            self.logger.error("Installing the boot script failed: %s", out)
            return False
        _, back = self.ssh.run('md5sum /boot/u-boot.scr')
        if not back.startswith(md5):
            self.logger.error("Installed /boot/u-boot.scr does not match the "
                              "bundled script (md5 mismatch) — %s", manual)
            return False
        self.logger.warning("Pre-built /boot/u-boot.scr installed (backup "
                            "kept next to it).")
        if not self._reboot_and_reconnect():
            return False
        reserved = self._dma_mem_reserved()
        if reserved:
            self.logger.warning("DMA memory reservation applied and active "
                                "after reboot.")
        else:
            self.logger.error(
                "DMA memory reservation still not active after the reboot "
                "(RAM top 0x%08X) — not retrying automatically; %s",
                self._system_ram_top() or 0, manual)
        return bool(reserved)

    _FPGAUTIL = '/opt/redpitaya/bin/fpgautil'

    def _board_has_fpgautil(self):
        """True when the board provides `fpgautil` (Red Pitaya OS 2.x and
        newer), i.e. the bitstream must be loaded through update_fpga.sh
        rather than by writing it to the legacy /dev/xdevcfg.

        Probed by capability, NOT by parsing /root/.version: that used to be
        `version.find('2.') != -1`, which silently went FALSE on OS 3.00 and
        sent the flash down the legacy xdevcfg path, so the bitstream never
        actually loaded (and the flash marker was still recorded, hiding it).
        """
        try:
            return self.ssh.run('test -x ' + self._FPGAUTIL)[0] == 0
        except Exception:
            self.logger.debug('fpgautil probe failed', exc_info=True)
            return False

    def update_fpga(self, filename=None):
        serverdirname = self.parameters['serverdirname']
        serverbinfilename = os.path.join(serverdirname, self.parameters['serverbinfilename'])
        update_cmdfile = os.path.join(serverdirname, 'update_fpga.sh')
        # For version 2.0 and higher to load a custom fpga use the update_fpga.sh script
        update_cmd = f'bash -x {update_cmdfile} pyrpl {serverbinfilename}'
        local_update_sh = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                                       'update_fpga.sh')
        force = self.parameters['force_reload']
        source = filename
        if filename is None:
            try:
                source = self.parameters['filename']
            except KeyError:
                source = None
        if not source or not os.path.isfile(source):
            if source:
                self.logger.warning('Desired bitfile "%s" does not exist. Using default installation.',
                                    source)
            source = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'fpga', 'red_pitaya.bin')
        if not os.path.isfile(source):
            raise IOError("Wrong filename",
              "The fpga bitfile was not found at the expected location. Try passing the arguments "
              "dirname=\"c://github//pyrpl//pyrpl//\" adapted to your installation directory of pyrpl "
              "and filename=\"red_pitaya.bin\"! Current dirname: "
              + self.parameters['dirname'] +
              " current filename: "+self.parameters['filename'])

        md5 = self._file_md5(source)
        # Cheap exec-channel checks (no rw remount / no service disruption yet):
        #   upload only when the kept on-board bitstream differs from ours
        #   flash  only when the running PL isn't this bitstream from this boot
        need_upload = force or (self._onboard_fpga_md5(serverbinfilename) != md5)
        need_flash = force or not self._fpga_flashed(serverbinfilename, md5)
        if not need_upload and not need_flash:
            self.logger.info("On-board FPGA bitstream up to date (md5 %s..) and "
                             "already flashed this boot; skipping reflash.", md5[:8])
            return False

        self.end()
        sleep(self.parameters['delay'])
        # /opt/pyrpl lives on the root fs, which may be mounted read-only. Use
        # the real `mount` command (not the rw/ro helpers in /opt/redpitaya/sbin,
        # which are only on the PATH in a login shell — ssh.ask() uses a non-login
        # interactive shell, so `rw` -> "command not found", the fs stays ro, and
        # every scp_put would fail and retry). _remount_root_rw() caches the
        # original state so we don't strand a normally-rw rootfs read-only.
        self._remount_root_rw()
        sleep(self.parameters['delay'])
        self.ssh.ask('mkdir -p ' + serverdirname)
        sleep(self.parameters['delay'])

        if need_upload:
            self.logger.debug("Uploading FPGA bitstream (md5 %s..).", md5[:8])
            self._scp_put_retry(local_update_sh, update_cmdfile)
            self._scp_put_retry(source, serverbinfilename)
        elif self.ssh.run('test -e ' + update_cmdfile)[0] != 0:
            # reflashing the already-present bitstream (e.g. after a reboot):
            # only the small flash script may be missing — upload just that, not
            # the multi-MB bitstream.
            self._scp_put_retry(local_update_sh, update_cmdfile)

        if need_flash:
            # reflashing changes the register map under any monitor_server, so
            # stop every instance (incl. a detached one from another client) and
            # the web app before flashing.
            self._kill_monitor_server()
            self.endclient()
            self.ssh.ask('killall nginx')
            self.ssh.ask('systemctl stop redpitaya_nginx') # for 0.94 and higher
            sleep(3) # sleep after stopping service
            version = self.ssh.ask('cat /root/.version')
            self.logger.debug('cat /root/.version: {}'.format(version))
            # Flash, capturing any failure rather than raising straight away:
            # nginx is stopped and / may be remounted rw at this point, so the
            # cleanup below must run before we propagate the error.
            flash_error = None
            if self._board_has_fpgautil():
                ret, out = self.ssh.run(update_cmd)
                if ret != 0 or 'loaded through FPGA manager successfully' not in out:
                    flash_error = (
                        "Flashing the FPGA bitstream FAILED.\n\n"
                        "Command : %s\nExit code: %s\n"
                        "Board OS: %s\n\nOutput:\n%s"
                        % (update_cmd, ret, version.strip(), out.strip()[-800:]))
            elif self.ssh.run('test -w /dev/xdevcfg')[0] == 0:
                self.ssh.ask('cat ' + serverbinfilename + ' > //dev//xdevcfg')
            else:
                flash_error = (
                    "Cannot flash the FPGA bitstream: this board has neither "
                    "%s nor a writable /dev/xdevcfg, so there is no supported "
                    "way to load the PL.\n\nBoard OS: %s"
                    % (self._FPGAUTIL, version.strip()))
            sleep(self.parameters['delay'])
            # Only claim the PL holds this bitstream when it actually does —
            # a marker written after a failed flash hides the failure from
            # every later connection.
            if flash_error is None:
                self._record_fpga_flashed(serverbinfilename, md5)
            self.logger.debug('About to restart the redpitaya service')
            self.ssh.ask("nginx -p //opt//www//")
            self.ssh.ask('systemctl start redpitaya_nginx')  # for 0.94 and higher #needs test
            sleep(self.parameters['delay'])
            if flash_error is not None:
                self._restore_root_mount()
                self.logger.error(flash_error)
                raise FpgaFlashError(flash_error)

        # NB: the bitstream + flash script are intentionally KEPT on the board
        # (no rm) so the next connection can md5-skip the upload and reflash
        # from the on-board copy after a reboot.
        self._restore_root_mount()
        # True when the PL was actually reflashed (the register link and any
        # monitor_server were torn down and must be brought back up by the
        # caller — __init__ does it via start(), change_fpga_image explicitly).
        return need_flash

    def change_fpga_image(self, filename=None):
        """Select the FPGA bitstream the board should run.

        `filename` is a local bitstream path (None/'' selects the bundled
        default fpga/red_pitaya.bin). The choice is persisted as 'filename' in
        the 'redpitaya' branch of the config file, so the NEXT connection (a
        fresh Pyrpl start) flashes it — md5-gated, exactly like every startup
        flash. Returns the recorded path (or None for the default).

        This only RECORDS the choice; it does NOT hot-swap the running fabric.
        Runtime hot-switching used to snapshot every module's live registers,
        reflash, reconnect, and replay them onto the fresh bitstream, but that
        register re-sync was incomplete (only declared setup-attributes were
        covered, so anything poked outside them reverted to the bitstream
        default). Callers now record the choice with this method, save the
        config, and restart the application; the normal startup path then
        flashes the image and restores all modules from the config in one
        well-tested step."""
        filename = filename or None
        if filename is not None:
            filename = os.path.expanduser(filename)
            if not os.path.isfile(filename):
                raise ExpectedPyrplError(
                    "FPGA bitstream not found: %s" % filename)
        self.parameters['filename'] = filename
        try:  # persist so the next startup flashes the selected image
            self.c._get_or_create('redpitaya')['filename'] = filename or ''
        except BaseException:
            self.logger.warning("Could not persist the FPGA image choice to "
                                "the config file.", exc_info=True)
        return filename

    def fpgarecentlyflashed(self):   # NB: currently unused
        self.ssh.ask()
        result = self.ssh.ask('cat /root/.version')
        if self._board_has_fpgautil():
            result = self.ssh.ask("echo $(($(date +%s) - $(date +%s -r \""
                                  + "//opt//redpitaya//fpga//z10_125//pyrpl//fpga.bit.bin" +"\")))")
            result = result.replace('\r', os.linesep)
            self.logger.debug('fpga age result: {}'.format(result.split(os.linesep)))
        else:
            result = self.ssh.ask("echo $(($(date +%s) - $(date +%s -r \""
                                  + os.path.join(self.parameters['serverdirname'], self.parameters['serverbinfilename']) +"\")))")
        age = None
        for line in result.split(os.linesep):
            try:
                age = int(line.strip())
            except:
                pass
            else:
                break
        if not age:
            self.logger.debug("Could not retrieve bitfile age from: %s",
                            result)
            return False
        elif age > 10:
            self.logger.debug("Found expired bitfile. Age: %s", age)
            return False
        else:
            self.logger.debug("Found recent bitfile. Age: %s", age)
            return True

    def _local_monitor_server_version(self):
        """(md5_hex, source_path) of the bundled monitor_server.c.

        The md5 of the source serves as the on-board build version marker: any
        change to monitor_server.c triggers a fresh native compile on the next
        connection (precompiled binaries are not portable across RedPitaya OS
        versions, so we never trust a binary built from a different source).
        """
        src = os.path.join(os.path.abspath(os.path.dirname(__file__)),
                           'monitor_server', 'monitor_server.c')
        with open(src, 'rb') as f:
            return hashlib.md5(f.read()).hexdigest(), src

    def _monitor_server_matches(self):
        """True if the board has a monitor_server compiled from the local source
        (its '.version' marker matches the local monitor_server.c md5)."""
        version, _ = self._local_monitor_server_version()
        binpath = self.parameters['serverdirname'] + self.parameters['monitor_server_name']
        _, marker = self.ssh.run('cat ' + binpath + '.version 2>/dev/null')
        exists, _ = self.ssh.run('test -x ' + binpath)
        return (version in marker) and (exists == 0)

    def compile_monitor_server(self):
        """Compile monitor_server natively on the RedPitaya when the on-board build
        is out of date, and return its path (or None on no-compiler / build error).

        Compares the local monitor_server.c md5 against a '.version' marker next to
        the binary; on a mismatch (or missing binary) it uploads the source and
        builds it with the board's gcc. The version marker is written by
        installserver() once the freshly built server has launched successfully.
        """
        version, src = self._local_monitor_server_version()
        serverdir = self.parameters['serverdirname']
        name = self.parameters['monitor_server_name']
        binpath = serverdir + name

        if self._monitor_server_matches():
            self.logger.debug("monitor_server on board matches local source (%s).",
                              version)
            return binpath

        ccret, ccout = self.ssh.run('command -v gcc || command -v cc')
        if ccret != 0 or not ccout.strip():
            self.logger.warning("monitor_server is out of date but the board has no "
                                "C compiler; falling back to precompiled binaries.")
            return None
        cc = ccout.strip().splitlines()[0]

        self.logger.info("Compiling monitor_server on the RedPitaya "
                         "(on-board build differs from local monitor_server.c)...")
        self._remount_root_rw()
        self.ssh.run('mkdir -p ' + serverdir)
        for i in range(3):
            try:
                self.ssh.scp_put(src, serverdir + 'monitor_server.c')
            except (SCPException, SSHException):
                self.start_ssh()
                sleep(self.parameters['delay'])
            else:
                break
        ret, out = self.ssh.run('cd ' + serverdir + ' && ' + cc +
                                ' -O2 -o ' + name + ' monitor_server.c -lpthread')
        built, _ = self.ssh.run('test -x ' + binpath)
        if ret != 0 or built != 0:
            self.logger.error("monitor_server failed to compile on the board:\n%s", out)
            self._restore_root_mount()
            return None
        self.ssh.run('chmod 755 ' + binpath)
        self._restore_root_mount()
        self.logger.info("monitor_server compiled on the board (version %s).", version)
        return binpath

    def _record_monitor_server_version(self, version):
        """Record the source md5 the running monitor_server was built/installed
        from (next to the binary), so the next connection can skip reinstalling an
        up-to-date server. /opt/pyrpl is on the read-only root fs, so remount it
        rw to write the marker. Uses the real `mount` command (the rw/ro helpers in
        /opt/redpitaya/sbin are on the PATH only in a login shell) so it works on
        the exec channel even while the server holds the interactive shell.
        """
        verpath = self.parameters['serverdirname'] + self.parameters['monitor_server_name'] + '.version'
        self._remount_root_rw()
        self.ssh.run('echo ' + version + ' > ' + verpath)
        self._restore_root_mount()

    def _monitor_server_running(self):
        """True if a monitor_server process is alive on the board (any client)."""
        ret, _ = self.ssh.run('pgrep -x ' + self.parameters['monitor_server_name'])
        return ret == 0

    def _kill_monitor_server(self):
        """Forcibly stop any monitor_server on the board, including a detached one
        started by another client. Used before a forced or replacement restart."""
        self.endserver()  # Ctrl-C an instance running in our own interactive shell
        self.ssh.run('killall -q ' + self.parameters['monitor_server_name'])
        sleep(self.parameters['delay'])

    def _launch_monitor_server(self, binpath):
        """Start an installed monitor_server DETACHED (setsid + background, I/O to
        /dev/null) so it outlives this ssh session: its DMA multicast and single
        register link then survive a client disconnect and can be reused by other
        clients. Returns the port on success, None if it failed to start."""
        self._restore_root_mount()  # fs back to its original state before running
        self.ssh.run('setsid ' + binpath + ' ' + str(self.parameters['port'])
                     + ' </dev/null >/dev/null 2>&1 &')
        sleep(self.parameters['delay'])
        if self._monitor_server_running():
            self.logger.debug("Server application started on port %d (detached)",
                              self.parameters['port'])
            self._serverrunning = True
            return self.parameters['port']
        self.endserver()
        return None

    def installserver(self):
        self._kill_monitor_server()  # replace any running/detached server
        sleep(self.parameters['delay'])
        version, _ = self._local_monitor_server_version()
        # Preferred: a native build matching the local monitor_server.c. Precompiled
        # binaries are not portable across RedPitaya OS versions, so compile on the
        # board (gcc) whenever the on-board build is out of date; the bundled
        # binaries below are only a fallback for boards without a compiler.
        binpath = self.compile_monitor_server()
        if binpath is not None:
            port = self._launch_monitor_server(binpath)
            if port is not None:
                self._record_monitor_server_version(version)
                return port
            self.logger.warning("Freshly compiled monitor_server did not start; "
                                "trying the precompiled binaries.")
        self._remount_root_rw()  # rw alias is login-shell only; use real mount
        sleep(self.parameters['delay'])
        self.ssh.ask('mkdir ' + self.parameters['serverdirname'])
        sleep(self.parameters['delay'])
        self.ssh.ask("cd " + self.parameters['serverdirname'])
        #try both versions
        for serverfile in ['monitor_server','monitor_server_0.95','monitor_server_2.07']:
            sleep(self.parameters['delay'])
            try:
                self.ssh.scp_put(
                    os.path.join(os.path.abspath(os.path.dirname(__file__)), 'monitor_server', serverfile),
                    self.parameters['serverdirname'] + self.parameters['monitor_server_name'])
            except (SCPException, SSHException):
                self.logger.exception("Upload error. Try again after rebooting your RedPitaya..")
            sleep(self.parameters['delay'])
            self.ssh.ask('chmod 755 ./'+self.parameters['monitor_server_name'])
            sleep(self.parameters['delay'])
            binpath = self.parameters['serverdirname'] + self.parameters['monitor_server_name']
            port = self._launch_monitor_server(binpath)  # detached; remounts ro
            if port is not None:
                self._record_monitor_server_version(version)
                return port
            # wrong binary version -> make sure it is not running and try the next
            self._kill_monitor_server()
            self._remount_root_rw()  # next scp needs the fs writable again

        #try once more on a different port
        if self.parameters['port'] == self.parameters['defaultport']:
            self.parameters['port'] = random.randint(self.parameters['defaultport'],50000)
            self.logger.warning("Problems to start the server application. Trying again with a different port number %d",self.parameters['port'])
            return self.installserver()

        self.logger.error("Server application could not be started. Try to recompile monitor_server on your RedPitaya (see manual). ")
        return None

    def startserver(self):
        force = self.parameters['force_reload']
        matches = self._monitor_server_matches()
        running = self._monitor_server_running()
        if not force and matches and running:
            # An up-to-date server is already running (perhaps started by another
            # client). Reuse it as-is — don't kill/relaunch — so concurrent
            # clients keep working and the single register link isn't disrupted.
            self.logger.debug("monitor_server already running and up to date; "
                              "reusing it on port %d.", self.parameters['port'])
            self._serverrunning = True
            return self.parameters['port']
        if force or not matches:
            # Forced, or the on-board binary was not built from the local source:
            # stop any running server and (re)install (compiles natively, then
            # relaunches detached).
            return self.installserver()
        # Correct binary present but not running -> just launch it (no recompile).
        sleep(2)
        port = self._launch_monitor_server(self.parameters['serverdirname']
                                            + self.parameters['monitor_server_name'])
        if port is not None:
            return port
        # something went wrong -> fall back to a full (re)install
        return self.installserver()

    def endserver(self):
        try:
            self.ssh.ask('\x03') #exit running server application
        except:
            self.logger.exception("Server not responding...")
        if 'pitaya' in self.ssh.ask():
            self.logger.debug('>') # formerly 'console ready'
        sleep(self.parameters['delay'])
        # make sure no other monitor_server blocks the port
        #  self.ssh.ask('killall ' + self.parameters['monitor_server_name'])
        self._serverrunning = False

    def endclient(self):
        del self.client
        self.client = None

    def start(self):
        if self.parameters['leds_off']:
            self.switch_led(gpiopin=0, state=False)
            self.switch_led(gpiopin=7, state=False)
        self.startserver()
        sleep(self.parameters['delay'])
        self.startclient()

    def end(self):
        self.endserver()
        self.endclient()

    def end_ssh(self):
        self.ssh.channel.close()

    def end_all(self):
        self.end()
        self.end_ssh()

    def restart(self):
        self.end()
        self.start()

    def restartserver(self, port=None):
        """restart the server. usually executed when client encounters an error"""
        if port is not None:
            if port < 0: #code to try a random port
                self.parameters['port'] = random.randint(2223,50000)
            else:
                self.parameters['port'] = port
        return self.startserver()

    def license(self):
        self.logger.info("""\r\n    pyrpl  Copyright (C) 2014-2017 Leonhard Neuhaus
    This program comes with ABSOLUTELY NO WARRANTY; for details read the file
    "LICENSE" in the source directory. This is free software, and you are
    welcome to redistribute it under certain conditions; read the file
    "LICENSE" in the source directory for details.\r\n""")

    def startclient(self):
        self.client = redpitaya_client.MonitorClient(
            self.parameters['hostname'], self.parameters['port'],
            restartserver=self.restartserver,
            reconnect_retries=self.parameters.get('reconnect_retries', -1),
            on_connection_lost=self._on_connection_lost,
            on_reconnected=self._on_reconnected)
        self.makemodules()
        self.logger.debug("Client started successfully. ")

    def _on_reconnected(self):
        """Called by MonitorClient when a background (automatic) socket reconnect
        succeeds — i.e. a transient link blip healed itself without user action.
        Re-emit as a Qt signal so a GUI can clear any "reconnecting" indicator.
        Safe to call from the reconnect worker thread (queued to GUI slots)."""
        self.logger.info("Register link auto-reconnected.")
        # The fabric may have been reloaded while the link was down (board
        # reboot), so anything cached about FPGA contents is now a guess.
        self._invalidate_hw_caches()
        try:
            self.signal_launcher.reconnected.emit()
        except BaseException:
            self.logger.exception("Failed to emit reconnected signal")

    def _invalidate_hw_caches(self):
        """Drop every host-side cache of what the FPGA currently holds, so the
        next write pushes for real. Today that is the ASGs' waveform tables
        (whose 64 KB push is skipped when unchanged)."""
        modules = getattr(self, 'modules', None) or {}
        for name, module in list(modules.items()):
            invalidate = getattr(module, 'invalidate_data_cache', None)
            if invalidate is None:
                continue
            try:
                invalidate()
            except BaseException:
                self.logger.debug("could not invalidate %s data cache",
                                  name, exc_info=True)

    def _on_connection_lost(self, reason):
        """Called by MonitorClient when the register link drops and the bounded
        reconnection gives up. Re-emit as a Qt signal so a GUI (e.g. the lidar
        widget) can prompt the user. Safe to call from a worker thread — the
        signal is delivered to GUI-thread slots via a queued connection."""
        self.logger.error("Register link lost; reconnection gave up: %s", reason)
        try:
            self.signal_launcher.connection_lost.emit(str(reason))
        except BaseException:
            self.logger.exception("Failed to emit connection_lost signal")

    def reconnect(self, hostname=None):
        """Re-establish a dropped connection, optionally switching to a new
        `hostname`. Meant to be driven by a GUI after a connection_lost signal.

        Rebuilds the ssh link, ensures the monitor_server is up, then reconnects
        the register link *in place* (the existing MonitorClient object is
        re-initialised, so cached module references stay valid). Raises on
        failure (ExpectedPyrplError / socket errors); returns True on success."""
        if hostname:
            self.parameters['hostname'] = hostname
        # fresh ssh channel (raises ExpectedPyrplError after a few tries)
        self.start_ssh()
        # if the board was parked (PL held in reset / blanked) while we were away,
        # release + reflash and drop the stale monitor_server before reconnecting
        self._recover_pl_reset()
        # make sure an up-to-date monitor_server is running on the board
        port = self.startserver()
        if self.client is None:
            self.startclient()
        else:
            # reconnect the SAME client object (keeps module._client refs valid)
            if not self.client.restart(hostname=self.parameters['hostname'],
                                       port=port):
                raise ExpectedPyrplError(
                    "Could not reconnect the register link to %s"
                    % self.parameters['hostname'])
        self.logger.info("Reconnected to Red Pitaya at %s.",
                         self.parameters['hostname'])
        # the board may have rebooted while we were away — see _on_reconnected
        self._invalidate_hw_caches()
        self.signal_launcher.reconnected.emit()
        return True

    def startdummyclient(self):
        self.client = redpitaya_client.DummyClient()
        self.makemodules()

    def makemodule(self, name, cls):
        module = cls(self, name)
        setattr(self, name, module)
        self.modules[name] = module

    def makemodules(self):
        """
        Automatically generates modules from the list RedPitaya.cls_modules

        Modules whose name is listed in the 'disabled_modules' parameter are
        skipped (not instantiated and not added to self.modules), which makes
        the client safe to run against reduced FPGA bitstreams that omit them
        (e.g. ['iir','pid1','pid2','iq1','iq2']). Names are generated from the
        full class list first, so numbering is preserved (skipping 'pid1' still
        leaves the remaining module named 'pid2').
        """
        names = get_unique_name_list_from_class_list(self.cls_modules)
        disabled = self.parameters.get('disabled_modules', []) or []
        if isinstance(disabled, str):  # tolerate "iir,pid1" from env/config
            disabled = [s.strip() for s in disabled.replace(',', ' ').split()]
        disabled = set(disabled)
        for cls, name in zip(self.cls_modules, names):
            if name in disabled:
                self.logger.info("Skipping module '%s' (listed in "
                                 "disabled_modules).", name)
                continue
            self.makemodule(name, cls)

    def make_a_slave(self, port=None, monitor_server_name=None, gui=False):
        if port is None:
            port = self.parameters['port'] + len(self._slaves)*10 + 1
        if monitor_server_name is None:
            monitor_server_name = self.parameters['monitor_server_name'] + str(port)
        slaveparameters = dict(self.parameters)
        slaveparameters.update(dict(
                         port=port,
                         autostart=True,
                         reloadfpga=False,
                         reloadserver=False,
                         monitor_server_name=monitor_server_name,
                         silence_env=True))
        r = RedPitaya(**slaveparameters) #gui=gui)
        r._master = self
        self._slaves.append(r)
        return r
