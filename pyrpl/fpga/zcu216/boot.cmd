# ZCU216 Red Pitaya port: the stock PetaLinux 2020.1 RF DC Evaluation Tool
# boot.scr with one change: bootargs set here, so they replace the DT ones.
#
# cma=256M: the image's cma=1700M never reserves anything ("cma: Failed to
# reserve 1700 MiB"; the largest free block under 4 GB is ~1650 MB), and
# without CMA the FPGA manager cannot allocate the bitstream buffer, so
# fpgautil fails with ENOMEM.
#
# Encode with: python3 mkscr.py --encode boot.cmd boot.scr
setenv bootargs "console=ttyPS0,115200n8 earlycon clk_ignore_unused initcall_blacklist=si570_driver_init root=/dev/ram rw cma=256M cpuidle.off=1"

# This is a boot script for U-Boot
# Generate boot.scr:
# mkimage -c none -A arm -T script -d boot.cmd.default boot.scr
#
################


for boot_target in ${boot_targets};
do
	if test "${boot_target}" = "jtag" ; then
		booti 0x00200000 0x04000000 0x00100000
		exit;
	fi
	if test "${boot_target}" = "mmc0" || test "${boot_target}" = "mmc1" ; then
		if test -e ${devtype} ${devnum}:${distro_bootpart} /image.ub; then
			fatload ${devtype} ${devnum}:${distro_bootpart} 0x10000000 image.ub;
			bootm 0x10000000;
			exit;
                fi
		if test -e ${devtype} ${devnum}:${distro_bootpart} /Image; then
			fatload ${devtype} ${devnum}:${distro_bootpart} 0x00200000 Image;;
		fi
		if test -e ${devtype} ${devnum}:${distro_bootpart} /system.dtb; then
			fatload ${devtype} ${devnum}:${distro_bootpart} 0x00100000 system.dtb;
		fi
		if test -e ${devtype} ${devnum}:${distro_bootpart} /rootfs.cpio.gz.u-boot; then
			fatload ${devtype} ${devnum}:${distro_bootpart} 0x04000000 rootfs.cpio.gz.u-boot;
			booti 0x00200000 0x04000000 0x00100000
			exit;
		fi
		booti 0x00200000 - 0x00100000
		exit;
	fi
	if test "${boot_target}" = "xspi0" || test "${boot_target}" = "qspi" || test "${boot_target}" = "qspi0"; then
		sf probe 0 0 0;
		if test "image.ub" = "image.ub"; then
			sf read 0x10000000 0x3F00000 0x3F00000;
			bootm 0x10000000;
			exit;
		fi
		if test "image.ub" = "Image"; then
			sf read 0x00200000 0x3F00000 0x1D00000;
			sf read 0x04000000 0x5D00000 0x1D00000
			booti 0x00200000 0x04000000 0x00100000
			exit;
		fi
		exit;
	fi
	if test "${boot_target}" = "nand" || test "${boot_target}" = "nand0"; then
		nand info
		if test "image.ub" = "image.ub"; then
			nand read 0x10000000 0x4100000 0x6400000;
			bootm 0x10000000;
			exit;
		fi
		if test "image.ub" = "Image"; then
			nand read 0x00200000 0x4100000 0x3200000;
			nand read 0x04000000 0x7800000  0x3200000;
			booti 0x00200000 0x04000000 0x00100000
			exit;
		fi
	fi
done
