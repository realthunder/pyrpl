# Red Pitaya U-Boot boot script

`u-boot.scr` is the compiled U-Boot boot script read at startup from the FAT
boot partition (`/dev/mmcblk0p1`, mounted at `/boot` on the running board).
It detects the board via the EEPROM `hw_rev` and assembles the kernel command
line (`bootargs`), loads the FPGA bitstream, kernel, and devicetree, then boots.

The `u-boot.scr` here is the Gen 1 board's script: `hw_rev =
STEM_125-14_Z7020_LN_v1.1` (Zynq-7020, 512 MB), Red Pitaya ecosystem 2.00,
U-Boot redpitaya-v2022.1.

The scripts pyrpl actually *installs* at runtime live in `pyrpl/uboot/`, one
per supported revision, named `u-boot.scr.<hw_rev>` and selected by
`RedPitaya._UBOOT_PREBUILT`:

| `hw_rev`                     | Board                        | Ecosystem | U-Boot            |
| ---------------------------- | ---------------------------- | --------- | ----------------- |
| `STEM_125-14_Z7020_LN_v1.1`  | Gen 1 LN, Zynq-7020, 512 MB  | 2.00 (37) | redpitaya-v2022.1 |
| `STEM_125-14_Z7020_Pro_v2.0` | Gen 2 Pro, Zynq-7020, 512 MB | 2.07 (43) | redpitaya-v2022.3 |

## Why we carry our own copy

The lidar point-cloud DMA (`fft_proc` / `dma_s2mm`, HP2 write path) writes into
a fixed-address ring buffer at physical **`0x1e000000`** (128 KB), which
`monitor_server` reads back via `/dev/mem`. Linux must not allocate that RAM.

The stock script appends **no** `mem=` for the `STEM_125-14_Z7020_*` boards, so
Linux owned the full 512 MB and could hand out our buffer. We add one line to
our board's block:

```
if test ${hw_rev} == 'STEM_125-14_Z7020_LN_v1.1'
then
  setenv bootargs ${bootargs} mem=480M      # reserve top 32 MB for DMA buffer
  setenv dts_path z20_125
  setenv zynq z7020
fi
```

`480 MiB = 0x1E000000` exactly, so Linux uses `0x00000000–0x1dffffff` and the
top 32 MB (`0x1e000000–0x1fffffff`) is a hole we own — reachable via `/dev/mem`
and mapped uncached, which is what the poll loop needs.

Note that `read()`/`write()` on `/dev/mem` cannot reach that hole on ARM (the
kernel clamps them to `high_memory`, so `dd` returns 0 bytes); only `mmap()`
can, which is what `monitor_server` uses. Test the window with a small
`mmap` program, not with `dd`.

## Gen 2 differences (`STEM_125-14_Z7020_Pro_v2.0`)

Two things differ from the Gen 1 recipe, and the second one bricks the boot if
missed:

1. The Gen 2 branch **already carries `mem=512M`** (a no-op cap — the board has
   exactly 512 MB). So the edit is a *replacement*, `mem=512M` → `mem=480M`,
   not the append that Gen 1 needed.
2. The Gen 2 branch also sets `high`, which the script later assigns to
   `fdt_high`/`initrd_high` — the ceiling U-Boot relocates the devicetree
   below. Stock is `0x20000000`, i.e. *above* the new RAM cap, so a 480 MiB
   kernel could not reach its own DTB and the board would not boot. It must
   drop to `0x1E000000` in the same block. Gen 1 branches set no `high` at
   all, which is why the Gen 1 recipe never mentioned it.

```
if test ${hw_rev} == 'STEM_125-14_Z7020_Pro_v2.0'
then
  setenv bootargs ${bootargs} mem=480M      # was mem=512M
  setenv high     0x1E000000                # was 0x20000000 (fdt_high!)
  setenv dts_path z20_125_v2
  setenv zynq z7020
fi
```

Unrelated to us, Gen 2 also reserves memory in the *devicetree* rather than in
bootargs: `linux,cma` (16 MB) plus `buffer@1000000` / `labuf@1000000`, two
aliases (`dma_region`, `rprx_reserverd`) of the same 32 MB window at
`0x01000000–0x02FFFFFF` used by Deep Memory Acquisition and the logic
analyzer. That window is disjoint from ours and is left alone. Because
devicetree `reserved-memory` stays inside `System RAM`, it does not affect the
`/proc/iomem`-top check pyrpl uses to detect our reservation.

Gen 2 rollback has an extra net: `/boot/uboot/u-boot_512Mb_ram.scr` is a
pristine factory copy of the stock script (and `u-boot_1Gb_ram.scr` for the
1 GB variants), alongside the `.bak-<timestamp>` the installer writes.

A device-tree `reserved-memory` overlay was tried first and removed: a runtime
configfs overlay cannot reserve memory (the kernel scans `reserved-memory` far
too early in boot), and our buffer is a fixed-address `/dev/mem` region with no
DTB consumer, so `mem=` is both sufficient and simpler.

## How to update the script on the board

The `.scr` is a `mkimage`-wrapped script: a 64-byte legacy header + an 8-byte
length table = **72 bytes** precede the editable text. `mkimage` is already on
the board (`/usr/bin/mkimage`). Do this over SSH (`root@<board-ip>`):

```sh
# 1. Extract the editable source (strip the 72-byte mkimage header)
tail -c +73 /boot/u-boot.scr > /tmp/boot.cmd

# 2. Edit /tmp/boot.cmd — change the bootargs in the matching hw_rev block.
#    Confirm your hw_rev first:  fw_printenv hw_rev

# 3. Rebuild (keep the image name "boot Debian")
mkimage -A arm -O linux -T script -C none -n "boot Debian" \
        -d /tmp/boot.cmd /tmp/u-boot.scr

# 4. Install, with a backup (the FAT partition is mounted read-only)
mount -o remount,rw /boot
cp -p /boot/u-boot.scr /boot/u-boot.scr.bak
cp /tmp/u-boot.scr /boot/u-boot.scr
sync
mount -o remount,ro /boot

# 5. Reboot and verify
reboot
# after it comes back:
cat /proc/cmdline            # ...uio_pdrv_genirq.of_id=generic-uio mem=480M
grep 'System RAM' /proc/iomem  # 00000000-1dffffff : System RAM
```

To deploy the exact script tracked here instead of editing on the board, copy
this file over and reboot:

```sh
scp uboot/u-boot.scr root@<board-ip>:/tmp/u-boot.scr
ssh root@<board-ip> '
  mount -o remount,rw /boot &&
  cp -p /boot/u-boot.scr /boot/u-boot.scr.bak &&
  cp /tmp/u-boot.scr /boot/u-boot.scr &&
  sync && mount -o remount,ro /boot && reboot'
```

## Rollback

A broken `u-boot.scr` means the board won't boot, so always keep the `.bak`:

```sh
mount -o remount,rw /boot &&
cp /boot/u-boot.scr.bak /boot/u-boot.scr &&
sync && mount -o remount,ro /boot && reboot
```

If even that fails, pull the SD card and restore `u-boot.scr` from a reader, or
recover via the serial console (`ttyPS0`, 115200) by setting `bootargs` in the
U-Boot environment manually.
