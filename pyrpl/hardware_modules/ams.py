from ..modules import HardwareModule
from ..attributes import PWMRegister, BoolRegister, IntRegister


class AMS(HardwareModule):
    """mostly deprecated module (redpitaya has removed adc support).
    only here for dac2 and dac3 — and the XADC die temperature (see temperature)."""
    addr_base = 0x40800000

    # attention: writing to dac0 and dac1 has no effect
    # only write to dac2 and 3 to set output voltages
    # to modify dac0 and dac1, connect a r.pwm0.input='pid0'
    # and let the pid module determine the voltage
    dac0 = PWMRegister(0x20, doc="PWM output 0 [V]")
    dac1 = PWMRegister(0x24, doc="PWM output 1 [V]")
    dac2 = PWMRegister(0x28, doc="PWM output 2 [V]")
    dac3 = PWMRegister(0x2C, doc="PWM output 3 [V]")
    pwm0_manual = BoolRegister(0x30, 0, doc='Enable PWM manual level output 0')
    pwm1_manual = BoolRegister(0x30, 1, doc='Enable PWM manual level output 1')
    pwm2_manual = BoolRegister(0x30, 2, doc='Enable PWM manual level output 2')
    pwm3_manual = BoolRegister(0x30, 3, doc='Enable PWM manual level output 3')

    # Zynq PL die temperature. The XADC sequencer samples the on-die temperature
    # sensor; adc_temp_r holds the 12-bit code, exposed read-only at 0x14 (a fast
    # register read over the monitor_server socket — no SSH/sysfs round-trip). Reads
    # back 0 on bitstreams built before this readout was added.
    xadc_temp = IntRegister(0x14, bits=12,
                            doc="raw 12-bit XADC die-temperature code (0 = not available)")

    @property
    def temperature(self):
        """Zynq-7 PL die temperature in degrees C, from the XADC.

        UG480: T[C] = code * 503.975 / 4096 - 273.15. Returns NaN when the running
        bitstream does not expose the code (reg 0x14 reads back 0)."""
        raw = self.xadc_temp
        if raw == 0:
            return float('nan')
        return raw * 503.975 / 4096.0 - 273.15

    def _setup(self): # the function is here for its docstring to be used by the metaclass.
        """
        sets up the AMS (just setting the attributes is OK)
        """
        pass
