from ..modules import HardwareModule
from ..attributes import PWMRegister, BoolRegister


class AMS(HardwareModule):
    """mostly deprecated module (redpitaya has removed adc support).
    only here for dac2 and dac3"""
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

    def _setup(self): # the function is here for its docstring to be used by the metaclass.
        """
        sets up the AMS (just setting the attributes is OK)
        """
        pass
