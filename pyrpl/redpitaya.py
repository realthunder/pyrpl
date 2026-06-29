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
from time import sleep
import numpy as np

from paramiko import SSHException
from scp import SCPClient, SCPException
from collections import OrderedDict

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
        # start other stuff
        if self.parameters['reloadfpga']:  # flash fpga
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
        if source is None or not os.path.isfile(source):
            if source is not None:
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
            return

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
            result = self.ssh.ask('cat /root/.version')
            self.logger.debug('cat /root/.version: {}'.format(result))
            if result.find('2.') != -1:
                self.ssh.run(update_cmd)
            else:
                self.ssh.ask('cat ' + serverbinfilename + ' > //dev//xdevcfg')
            sleep(self.parameters['delay'])
            self._record_fpga_flashed(serverbinfilename, md5)
            self.logger.debug('About to restart the redpitaya service')
            self.ssh.ask("nginx -p //opt//www//")
            self.ssh.ask('systemctl start redpitaya_nginx')  # for 0.94 and higher #needs test
            sleep(self.parameters['delay'])

        # NB: the bitstream + flash script are intentionally KEPT on the board
        # (no rm) so the next connection can md5-skip the upload and reflash
        # from the on-board copy after a reboot.
        self._restore_root_mount()

    def fpgarecentlyflashed(self):
        self.ssh.ask()
        result = self.ssh.ask('cat /root/.version')
        if result.find('2.') != -1:
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
            self.parameters['hostname'], self.parameters['port'], restartserver=self.restartserver)
        self.makemodules()
        self.logger.debug("Client started successfully. ")

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
