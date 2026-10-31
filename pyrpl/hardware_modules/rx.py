"""
rx: one RF-ADC channel at the system clock's rate, any band of 0 - 1.25 GHz,
properly filtered (ZCU216, fpga/rtl/zcu216_rx.sv). Its outputs are hub signals,
rx0_i and rx0_q, which any module, the scope or the FFT can take as input, next to
in1-4, the fast but unfiltered route to the same channels:

    r.rx0.setup(input='in1', frequency=700e6, mode='real')
    r.iq0.input = 'rx0_i'           # sees the band around 700 MHz at 62.5 MHz

    frequency  the middle of the band; 0 is a plain low-pass of the channel
    mode       'iq': +-100 MHz around frequency on rx0_i and rx0_q, about 59 dB
               from aliases; rx0_i alone cannot tell f + d from f - d
               'real': +-37.5 MHz on rx0_i alone, moved to 62.5 MHz, about 83 dB,
               so frequency + d comes out at 62.5 MHz + d
"""
from ..attributes import BoolRegister, FloatRegister, SelectRegister, \
    SelectProperty
from ..modules import HardwareModule
from .dsp import RX_NUMBERS


class Rx(HardwareModule):
    _setup_attributes = ['input', 'frequency', 'mode']
    _gui_attributes = _setup_attributes

    input = SelectRegister(0x0, options={'in1': 12, 'in2': 13, 'in3': 22, 'in4': 23},
                           doc="the RF-ADC channel")

    _real = BoolRegister(0x104, bit=0, doc="the narrow filter, rx_i only")
    _gain2 = BoolRegister(0x104, bit=1, doc="on unless frequency is 0: mixing "
                                            "halves a tone, a low-pass does not")

    frequency = FloatRegister(0x100, bits=32, norm=2 ** 32 / 2.5e9, signed=False,
                              min=0, max=1.25e9, call_setup=True,
                              doc="the middle of the band [Hz]")

    mode = SelectProperty(options=['iq', 'real'], default='iq', call_setup=True,
                          doc="iq: +-100 MHz on rx_i and rx_q, real: +-37.5 MHz "
                              "on rx_i alone, moved to 62.5 MHz")

    def __init__(self, parent, name=None):
        self.addr_base = 0x40600000 + RX_NUMBERS[name] * 0x10000
        super(Rx, self).__init__(parent, name=name)

    def _setup(self):
        self._real = self.mode == 'real'
        self._gain2 = self.frequency != 0
