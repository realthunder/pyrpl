/*
 * clk104: program the ZCU216's CLK104 clock module for the RF build and read its
 * lock indicators. Runs on the board (Linux i2c-dev); static musl build:
 *   make -C zcu216 clk104
 *
 *   clk104 [-b bus] program   LMK04828 "250M_PL_125M_SYSREF_10M" (250 MHz refs,
 *                             PL_CLK 125 MHz, SYSREF 10 MHz), then both LMX2594s
 *                             to 2.5 GHz (the ADC and DAC sample clocks)
 *   clk104 [-b bus] status    lock-detect line of each chip (needs the RF
 *                             bitstream: its slot-1 register 0x24 picks the chip)
 *   clk104 [-b bus]           program, then status
 *
 * The CLK104 hangs off I2C1 switch U20 (0x74) port 5, which Linux shows as
 * /dev/i2c-11 (UG1390 table 11); its SC18IS602 I2C-SPI bridge sits at 0x2F.
 * A bridge write [ss, b2, b1, b0] clocks 24 bits out on the chip selects in ss:
 * LMX2594 #1 = SS3 (0x08), LMX2594 #2 = SS2 (0x04), LMK04828 = SS1 (0x02), as
 * in AMD's xrfclk. The register sequences are AMD's (clk104_tables.h).
 *
 * Lock status: the chips' SDO lines are muxed to the bridge by two PL pins
 * (spi_sdo_sel, chip id 0/1/2). With these configurations the LMX MUXOUT and the
 * LMK Status_LD1 pin drive lock detect, so a 3-byte transfer reads back all ones
 * from a locked chip and all zeros from an unlocked one.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "clk104_tables.h"

#define BRIDGE_ADDR   0x2f
#define SS_LMX1       0x08
#define SS_LMX2       0x04
#define SS_LMK        0x02
#define SLOT1_BASE    0x80200000UL     /* zcu216_fft_test */
#define SLOT1_ID_TAG  0xf0f6u
#define REG_SDO_SEL   0x24

static int fd;

static void die(const char *what)
{
    fprintf(stderr, "clk104: %s: %s\n", what, strerror(errno));
    exit(1);
}

static int bridge_write(const uint8_t *buf, int n)
{
    /* the bridge NAKs while it is still shifting the previous transfer out */
    for (int tries = 0; tries < 20; tries++) {
        if (write(fd, buf, n) == n) {
            usleep(1000);
            return 0;
        }
        usleep(1000);
    }
    return -1;
}

static void spi_word(uint8_t ss, unsigned w)
{
    uint8_t b[4] = { ss, (w >> 16) & 0xff, (w >> 8) & 0xff, w & 0xff };
    if (bridge_write(b, 4))
        die("SPI write through the bridge");
}

static void program(void)
{
    static const struct { uint8_t ss; const char *name; } lmx[] = {
        { SS_LMX1, "LMX2594 #1" }, { SS_LMX2, "LMX2594 #2" } };

    printf("LMK04828: %zu words (250M_PL_125M_SYSREF_10M)\n",
           sizeof lmk_250m / sizeof lmk_250m[0]);
    for (size_t i = 0; i < sizeof lmk_250m / sizeof lmk_250m[0]; i++)
        spi_word(SS_LMK, lmk_250m[i]);
    usleep(20000);                      /* LMK PLLs lock before the LMXs calibrate */

    for (int c = 0; c < 2; c++) {
        printf("%s: %zu words (2.5 GHz)\n", lmx[c].name,
               sizeof lmx_2500m / sizeof lmx_2500m[0]);
        for (size_t i = 0; i < sizeof lmx_2500m / sizeof lmx_2500m[0]; i++)
            spi_word(lmx[c].ss, lmx_2500m[i]);
    }
    usleep(20000);
}

static void status(void)
{
    static const struct { uint8_t ss, id; const char *name; } chip[] = {
        { SS_LMX1, 0, "LMX2594 #1" }, { SS_LMX2, 1, "LMX2594 #2" },
        { SS_LMK, 2, "LMK04828  " } };
    int mfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mfd < 0)
        die("/dev/mem");
    volatile uint32_t *slot1 = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED,
                                    mfd, SLOT1_BASE);
    if (slot1 == MAP_FAILED)
        die("mmap slot 1");
    if ((slot1[0] >> 16) != SLOT1_ID_TAG) {
        fprintf(stderr, "clk104: slot 1 id %08x: RF bitstream not loaded, no status\n",
                slot1[0]);
        exit(1);
    }

    int all = 1;
    for (int c = 0; c < 3; c++) {
        uint8_t tx[4] = { chip[c].ss, 0x80, 0x00, 0x00 }, rx[3] = { 0 };
        slot1[REG_SDO_SEL / 4] = chip[c].id;
        usleep(100);
        if (bridge_write(tx, 4) || read(fd, rx, 3) != 3)
            die("bridge read");
        int locked = rx[0] == 0xff && rx[1] == 0xff && rx[2] == 0xff;
        all &= locked;
        printf("%s: %02x %02x %02x  %s\n", chip[c].name, rx[0], rx[1], rx[2],
               locked ? "locked" : (rx[0] | rx[1] | rx[2]) ? "?" : "NOT locked");
    }
    slot1[REG_SDO_SEL / 4] = 0;
    munmap((void *)slot1, 4096);
    close(mfd);
    if (!all)
        exit(2);
}

int main(int argc, char **argv)
{
    int bus = 11, opt;
    while ((opt = getopt(argc, argv, "b:")) != -1) {
        if (opt == 'b') bus = atoi(optarg);
        else { fprintf(stderr, "usage: clk104 [-b bus] [program|status]\n"); return 1; }
    }
    const char *cmd = optind < argc ? argv[optind] : "all";

    char dev[32];
    snprintf(dev, sizeof dev, "/dev/i2c-%d", bus);
    if ((fd = open(dev, O_RDWR)) < 0)
        die(dev);
    if (ioctl(fd, I2C_SLAVE, BRIDGE_ADDR) < 0 && ioctl(fd, I2C_SLAVE_FORCE, BRIDGE_ADDR) < 0)
        die("select the I2C-SPI bridge");

    /* SPI: MSB first, mode 0, 58 kHz (as xrfclk). Also proves the bridge answers. */
    uint8_t cfg[2] = { 0xf0, 0x03 };
    if (bridge_write(cfg, 2)) {
        fprintf(stderr, "clk104: no I2C-SPI bridge at 0x%02x on %s\n", BRIDGE_ADDR, dev);
        return 1;
    }

    if (!strcmp(cmd, "program") || !strcmp(cmd, "all"))
        program();
    if (!strcmp(cmd, "status") || !strcmp(cmd, "all"))
        status();
    close(fd);
    return 0;
}
