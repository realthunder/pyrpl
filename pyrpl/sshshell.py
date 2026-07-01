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


import paramiko
import socket
from time import sleep, time
from scp import SCPClient
import logging

# Generous upper bound (s) for a single ssh command in run(): long enough for a
# legitimate on-board gcc compile / FPGA flash, short enough that a dead board
# can't hang the caller forever. Callers pass an explicit timeout to override.
_RUN_TIMEOUT = 60


class SshShell(object):
    """ This is a wrapper around paramiko.SSHClient and scp.SCPClient
    I provides a ssh connection with the ability to transfer files over it"""
    def __init__(
            self,
            hostname='localhost',
            user='root',
            password='root',
            delay=0.05, 
            timeout=3,
            sshport=22,
            shell=True):
        self._logger = logging.getLogger(name=__name__)
        self.delay = delay
        self.apprunning = False
        self.hostname = hostname
        self.sshport=sshport
        self.user = user
        self.password = password
        self.timeout= timeout
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(
            hostname,
            username=self.user,
            password=self.password,
            port=self.sshport,
            timeout=timeout,
            look_for_keys=False,
            allow_agent=False)
        if shell:
            self.channel = self.ssh.invoke_shell()
        self.startscp()

    def scp_put(self, src, dst, *args, **kargs):
        self._logger.debug(f'scp "{src}" -> "{dst}"')
        return self.scp.put(src, dst, *args, **kargs)

    def startscp(self):
        self.scp = SCPClient(self.ssh.get_transport())

    def write(self, text):
        if self.channel.send_ready() and not text == "":
            self._logger.debug(f'< {text}')
            return self.channel.send(text)
        else:
            return -1

    def read_nbytes(self, nbytes):
        if self.channel.recv_ready():
            return self.channel.recv(nbytes)
        else:
            return b""

    def read(self):
        sumstring = ""
        while True:
            string = self.read_nbytes(1024).decode('utf-8')
            sumstring += string
            if not string:
                break
        self._logger.debug(sumstring)
        return sumstring

    def askraw(self, question=""):
        self.write(question)
        sleep(self.delay)
        return self.read()

    def ask(self, question="", block=False):
        return self.askraw(question + '\n')

    def run(self, cmd, timeout=None):
        # A wall-clock deadline is essential: exec_command on a stale transport
        # (e.g. the board was unplugged) otherwise busy-loops here forever waiting
        # for an exit status that never comes, freezing whatever thread called it.
        # The bound is generous (not self.timeout, the 3 s connect timeout): some
        # commands run through here legitimately take seconds — the on-board gcc
        # compile of monitor_server, FPGA/update commands — so we only guard
        # against an unbounded hang, not against slow-but-progressing commands.
        if timeout is None:
            timeout = _RUN_TIMEOUT
        self._logger.debug(f'< {cmd}')
        stdin_, stdout_, stderr_ = self.ssh.exec_command(cmd, timeout=timeout)
        channel = stdout_.channel
        channel.set_combine_stderr(True)
        exited = False
        lines = []
        deadline = time() + timeout
        while True:
            exited = channel.exit_status_ready()
            if channel.recv_ready():
                while True:
                    line = stdout_.readline(1024)
                    if not line:
                        break
                    line = line.strip()
                    self._logger.debug(f'> {line}')
                    lines.append(line)
            if exited:
                ret = channel.recv_exit_status()
                channel.close()
                break
            if time() > deadline:
                self._logger.warning("ssh run() timed out after %ss: %s",
                                     timeout, cmd)
                try:
                    channel.close()
                except Exception:
                    pass
                raise socket.timeout("ssh command timed out: %s" % cmd)
            sleep(0.01)  # yield instead of busy-spinning while waiting for exit
        return ret, '\n'.join(lines)

    def __del__(self):
        self.endapp()
        try:
            self.channel.close()
        except AttributeError:
            pass  # already broken
        self.ssh.close()

    def endapp(self):
        pass

    def reboot(self):
        self.endapp()
        self.ask("shutdown -r now")
        self.__del__()

    def shutdown(self):
        self.endapp()
        self.ask("shutdown now")
        self.__del__()

    def get_mac_addresses(self):
        """
        returns all MAC addresses of the SSH device.
        """
        self.ask()  # empty the shell before asking something
        macs = list()
        nextgood = False
        for token in self.ask('ifconfig | grep HWaddr').split():
            if nextgood and len(token.split(':'))==6:
                macs.append(token)
            if token == 'HWaddr':
                nextgood = True
            else:
                nextgood = False
        if macs == []:  # problem on more recent redpitaya os
            nextgood = False
            for token in self.ask('ip address').split():
                if nextgood and len(token.split(':'))==6:
                    macs.append(token)
                if token == 'link/ether':
                    nextgood = True
                else:
                    nextgood = False
        return macs
