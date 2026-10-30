/*
 * vadj: read and set the ZCU216's VADJ_FMC rail (FMC+ J28 VADJ pins, PL banks
 * 66 / 67) through its regulator, output C of the IRPS5401 PMIC U53 (PMBus
 * 0x44, UG1390 table 25). Runs on the board (Linux i2c-dev); static musl build:
 *   make -C zcu216 vadj
 *
 *   vadj [-b bus] status    every output of the PMIC: OPERATION, ON_OFF_CONFIG,
 *                           VOUT_COMMAND and READ_VOUT in volts, STATUS_WORD
 *   vadj [-b bus] set V     output C to V volts (1.2, 1.5 or 1.8) and on
 *
 * Why: the system controller (MSP430) sets VADJ_FMC from the FMC card's IPMI
 * EEPROM at power-on, and to 0 V when there is none, as on the PZ-FL9767D DAC
 * card; its board app would override that, this does it from Linux at boot
 * (autostart.sh), before the bitstream.
 *
 * The PMIC is on I2C0 switch port 2, /dev/i2c-4 in Linux. The kernel's irps5401
 * driver is bound to it (it exposes nothing), so the address is taken with
 * I2C_SLAVE_FORCE. PMBus pages 0-3 are switchers A-D, 4 the LDO (D is unused
 * on the ZCU216, its page refused); PAGE is left at 0 afterwards.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define PMIC_ADDR      0x44
#define VADJ_PAGE      2               /* output C */

#define PM_PAGE        0x00
#define PM_OPERATION   0x01
#define PM_ON_OFF_CFG  0x02
#define PM_CLEAR_FAULTS 0x03
#define PM_VOUT_MODE   0x20
#define PM_VOUT_CMD    0x21
#define PM_STATUS_WORD 0x79
#define PM_READ_VOUT   0x8b

static int fd;

static void die(const char *what)
{
    fprintf(stderr, "vadj: %s: %s\n", what, strerror(errno));
    exit(1);
}

static int smbus(char rw, uint8_t cmd, int size, union i2c_smbus_data *d)
{
    struct i2c_smbus_ioctl_data a = { .read_write = rw, .command = cmd, .size = size, .data = d };
    return ioctl(fd, I2C_SMBUS, &a);
}

static int rd_byte(uint8_t cmd)
{
    union i2c_smbus_data d;
    if (smbus(I2C_SMBUS_READ, cmd, I2C_SMBUS_BYTE_DATA, &d) < 0) return -1;
    return d.byte;
}

static int rd_word(uint8_t cmd)
{
    union i2c_smbus_data d;
    if (smbus(I2C_SMBUS_READ, cmd, I2C_SMBUS_WORD_DATA, &d) < 0) return -1;
    return d.word;
}

static void wr_byte(uint8_t cmd, uint8_t v)
{
    union i2c_smbus_data d = { .byte = v };
    if (smbus(I2C_SMBUS_WRITE, cmd, I2C_SMBUS_BYTE_DATA, &d) < 0) die("write");
}

static void wr_word(uint8_t cmd, uint16_t v)
{
    union i2c_smbus_data d = { .word = v };
    if (smbus(I2C_SMBUS_WRITE, cmd, I2C_SMBUS_WORD_DATA, &d) < 0) die("write");
}

/* VOUT_MODE linear: bits 4:0 the exponent (two's complement), volts = m * 2^e */
static int vout_exp(int mode)
{
    int e = mode & 0x1f;
    return e >= 16 ? e - 32 : e;
}

static void status(void)
{
    printf("page  operation  on_off_cfg  vout_mode  vout_cmd     read_vout    status_word\n");
    for (int p = 0; p <= 4; p++) {
        if (p == 3) continue;          /* output D is not used: the PMIC refuses
                                        * the page and flags a CML fault */
        union i2c_smbus_data d = { .byte = p };
        if (smbus(I2C_SMBUS_WRITE, PM_PAGE, I2C_SMBUS_BYTE_DATA, &d) < 0) {
            printf("%d            page not accepted (%s)\n", p, strerror(errno));
            continue;
        }
        int op = rd_byte(PM_OPERATION), oo = rd_byte(PM_ON_OFF_CFG);
        int mode = rd_byte(PM_VOUT_MODE), cmd = rd_word(PM_VOUT_CMD);
        int vo = rd_word(PM_READ_VOUT), st = rd_word(PM_STATUS_WORD);
        int e = mode >= 0 ? vout_exp(mode) : 0;
        printf("%d %s  0x%02x       0x%02x        0x%02x       %6.3f V     %6.3f V     0x%04x\n",
               p, p == VADJ_PAGE ? "(C, VADJ)" : p == 4 ? "(LDO)    " : "         ",
               op & 0xff, oo & 0xff, mode & 0xff,
               cmd >= 0 ? cmd * ldexp(1.0, e) : -1.0, vo >= 0 ? vo * ldexp(1.0, e) : -1.0,
               st & 0xffff);
    }
    wr_byte(PM_PAGE, 0);
}

static void set(double v)
{
    if (v != 1.2 && v != 1.5 && v != 1.8) {
        fprintf(stderr, "vadj: %.3f V: VADJ_FMC takes 1.2, 1.5 or 1.8 V\n", v);
        exit(1);
    }
    wr_byte(PM_PAGE, VADJ_PAGE);
    int mode = rd_byte(PM_VOUT_MODE);
    if (mode < 0 || (mode & 0xe0) != 0) {
        fprintf(stderr, "vadj: VOUT_MODE 0x%02x is not linear\n", mode);
        wr_byte(PM_PAGE, 0);
        exit(1);
    }
    wr_word(PM_VOUT_CMD, (uint16_t)lround(v / ldexp(1.0, vout_exp(mode))));
    wr_byte(PM_OPERATION, 0x80);       /* on, nominal margins */
    usleep(20000);
    /* the turn-on inrush (the FMC card, the banks' decoupling) latches an
     * over-current warning (limit 6 A) that the steady 10-20 mA never would */
    union i2c_smbus_data d;
    struct i2c_smbus_ioctl_data a = { .read_write = I2C_SMBUS_WRITE, .command = PM_CLEAR_FAULTS,
                                      .size = I2C_SMBUS_BYTE, .data = &d };
    ioctl(fd, I2C_SMBUS, &a);
    int vo = rd_word(PM_READ_VOUT);
    wr_byte(PM_PAGE, 0);
    double got = vo >= 0 ? vo * ldexp(1.0, vout_exp(mode)) : -1.0;
    printf("vadj: output C set to %.2f V, reads %.3f V\n", v, got);
    if (got < v * 0.95 || got > v * 1.05) exit(2);
}

int main(int argc, char **argv)
{
    int bus = 4;
    if (argc > 2 && !strcmp(argv[1], "-b")) { bus = atoi(argv[2]); argc -= 2; argv += 2; }
    if (argc < 2 || (strcmp(argv[1], "status") && strcmp(argv[1], "set")) ||
        (!strcmp(argv[1], "set") && argc < 3)) {
        fprintf(stderr, "usage: vadj [-b bus] status | set V\n");
        return 1;
    }
    char dev[32];
    snprintf(dev, sizeof dev, "/dev/i2c-%d", bus);
    if ((fd = open(dev, O_RDWR)) < 0) die(dev);
    if (ioctl(fd, I2C_SLAVE_FORCE, PMIC_ADDR) < 0) die("I2C_SLAVE_FORCE");
    if (!strcmp(argv[1], "status")) status();
    else set(atof(argv[2]));
    return 0;
}
