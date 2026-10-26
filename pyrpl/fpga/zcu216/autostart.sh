#!/bin/sh
# ZCU216 Red Pitaya port: boot-time hook, run by /etc/init.d/trd-autostart
# (PetaLinux 2020.1 RF DC Evaluation Tool image) from the SD card.
#
#   - eth0 gets the static address below (the router reserves it by MAC);
#   - HPM0_LPD is set to 32 bits (LPD_SLCR AFI_FS[9:8] = 0), the width the
#     register bus in zcu216.bit.bin expects;
#   - zcu216.bit.bin is loaded through the FPGA manager;
#   - monitor_server_zcu216 is started on port 2222, but only once the load
#     has succeeded: an access to 0x8000_0000 with no design loaded hangs the
#     bus.
#
# The stock start-up (autostart.sh.orig: 169.254.10.2 + rftool) runs instead
# when rftool.mode exists on the card. Do not start rftool with zcu216.bit.bin
# loaded: the evaluation design's devices at 0xA000_0000+ are then gone.
# Without zcu216.bit.bin on the card only the network is set up.
# Log: autostart.log on the card, rewritten at every boot.

SD=/media/sd-mmcblk0p1
IP=192.168.1.216
GW=192.168.1.1
PORT=2222

if [ -f $SD/rftool.mode ]; then
	exec sh $SD/autostart.sh.orig
fi

exec >$SD/autostart.log 2>&1
echo "autostart: $(date)"

# static address; stop the DHCP client so a lease cannot replace it. On the
# way out udhcpc releases the lease and its deconfig hook flushes every eth0
# address, so wait until it has gone before setting ours: setting it straight
# after the kill lost the race and left eth0 with no address at all.
killall udhcpc 2>/dev/null
for i in 1 2 3 4 5 6 7 8 9 10; do
	pidof udhcpc >/dev/null || break
	sleep 1
done
pidof udhcpc >/dev/null && { echo "udhcpc still running: SIGKILL"; killall -9 udhcpc; }
ifconfig eth0 $IP netmask 255.255.255.0 up
route add default gw $GW
echo "nameserver $GW" >/etc/resolv.conf
ifconfig eth0

# key login for the host tools (the rootfs is a ramdisk: redone every boot).
# dropbear 2019.78 predates ed25519, so the keys must be RSA.
if [ -f $SD/authorized_keys ]; then
	mkdir -p /home/root/.ssh
	cp $SD/authorized_keys /home/root/.ssh/authorized_keys
	chmod 700 /home/root/.ssh
	chmod 600 /home/root/.ssh/authorized_keys
fi

# fpgautil copies the whole bitstream into one coherent DMA buffer, which
# needs CMA; log what the kernel actually got
echo "cmdline: $(cat /proc/cmdline)"
grep -i cma /proc/meminfo
dmesg | grep -i cma

if [ ! -f $SD/zcu216.bit.bin ]; then
	echo "no zcu216.bit.bin: network only"
	exit 0
fi

fs=$(devmem 0xFF419000 32)
devmem 0xFF419000 32 $(( fs & ~0x300 ))
echo "AFI_FS (LPD) $fs -> $(devmem 0xFF419000 32)"

fpgautil -b $SD/zcu216.bit.bin
state=$(cat /sys/class/fpga_manager/fpga0/state)
echo "fpga0 state: $state"
if [ "$state" != "operating" ]; then
	echo "bitstream load failed: monitor_server not started"
	dmesg | grep -iE "fpga|cma|dma" | tail -20
	exit 1
fi

echo "hk id $(devmem 0x80000000 32) board $(devmem 0x80000040 32)"

cp $SD/monitor_server_zcu216 /tmp/monitor_server
chmod +x /tmp/monitor_server
setsid /tmp/monitor_server $PORT </dev/null >/tmp/monitor_server.log 2>&1 &
sleep 1
if kill -0 $! 2>/dev/null; then
	echo "monitor_server running on port $PORT (pid $!)"
else
	echo "monitor_server exited:"; cat /tmp/monitor_server.log
fi
