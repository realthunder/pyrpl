"""
What the FPGA reports about its board: converter rates and widths, the bus
clock and which modules the bitstream holds, read from hk's capability block
(fpga/rtl/red_pitaya_hk.v):

    0x40  board id: 0 = Red Pitaya, 1 = ZCU216
    0x48  {16'hCA95, version}; 0 on bitstreams without the block
    0x4C  ADC sample rate [Hz]      0x50  DAC sample rate [Hz]
    0x54  bus / hk clock [Hz]       0x58  {DAC ch, DAC bits, ADC ch, ADC bits}
    0x5C  module bitmap (MODULE_BITS, FEATURE_BITS)

A bitstream without the block gets the defaults of its board id, so older
Red Pitaya images keep working as before (every module present; the
'disabled_modules' parameter still removes the ones it lacks).
"""
import logging

logger = logging.getLogger(__name__)

HK_BASE = 0x40000000
CAPS_MAGIC = 0xCA95

# bit -> module name (pyrpl's names: RedPitaya.cls_modules, numbered)
MODULE_BITS = {'ams': 0, 'scope': 1, 'sampler': 2,
               'asg0': 3, 'asg1': 4, 'asg2': 5, 'asg3': 6,
               'pwm0': 7, 'pwm1': 8, 'pwm2': 9, 'pwm3': 10,
               'iq0': 11, 'iq1': 12, 'iq2': 13,
               'pid0': 14, 'pid1': 15, 'pid2': 16,
               'trig': 17, 'iir': 18}
FEATURE_BITS = {'scope_fft': 24,   # FFT / peak / CFAR inside the scope (Red Pitaya)
                'scope_dma': 25,   # the scope's DMA point stream (Red Pitaya)
                'rf_fft': 26,      # the RF FFT block (ZCU216 slot 1)
                'ddr_scope': 27}   # the scope keeps its trace in DDR (ZCU216)

BOARD_NAMES = {0: 'Red Pitaya', 1: 'ZCU216'}


def _bits(names, table):
    m = 0
    for n in names:
        m |= 1 << table[n]
    return m


# what a bitstream without the capability block is assumed to be, by board id
LEGACY = {
    0: dict(adc_hz=125e6, dac_hz=125e6, sys_hz=125e6, adc_bits=14, adc_ch=2,
            dac_bits=14, dac_ch=2,
            modules=_bits(MODULE_BITS, MODULE_BITS)
            | _bits(('scope_fft', 'scope_dma'), FEATURE_BITS)),
    1: dict(adc_hz=2.5e9, dac_hz=2.5e9, sys_hz=100e6, adc_bits=14, adc_ch=4,
            dac_bits=14, dac_ch=4,
            modules=_bits(('scope',), MODULE_BITS)
            | _bits(('rf_fft', 'ddr_scope'), FEATURE_BITS)),
}


class BoardCaps(object):
    """the board as the FPGA describes it; see the module docstring"""

    def __init__(self, board=0, version=0, adc_hz=125e6, dac_hz=125e6,
                 sys_hz=125e6, adc_bits=14, adc_ch=2, dac_bits=14, dac_ch=2,
                 modules=None):
        self.board = board
        self.version = version          # 0: defaults, not read from the FPGA
        self.adc_hz = float(adc_hz)
        self.dac_hz = float(dac_hz)
        self.sys_hz = float(sys_hz)
        self.adc_bits = adc_bits
        self.adc_ch = adc_ch
        self.dac_bits = dac_bits
        self.dac_ch = dac_ch
        self.modules = LEGACY[0]['modules'] if modules is None else modules

    @classmethod
    def legacy(cls, board=0):
        return cls(board=board, **LEGACY.get(board, LEGACY[0]))

    @classmethod
    def read(cls, client, base=HK_BASE):
        """read the block through a MonitorClient; defaults if it is absent"""
        try:
            w = client.reads(base + 0x40, 8)
        except Exception:
            w = None
        if w is None:
            logger.warning("could not read the board capabilities; assuming a "
                           "Red Pitaya")
            return cls.legacy(0)
        w = [int(x) & 0xFFFFFFFF for x in w]
        board = w[0] & 0xF
        if (w[2] >> 16) != CAPS_MAGIC:
            caps = cls.legacy(board)
            logger.info("bitstream without a capability block: %s", caps)
            return caps
        conv = w[6]
        caps = cls(board=board, version=w[2] & 0xFFFF,
                   adc_hz=w[3], dac_hz=w[4], sys_hz=w[5],
                   adc_bits=conv & 0xFF, adc_ch=(conv >> 8) & 0xFF,
                   dac_bits=(conv >> 16) & 0xFF, dac_ch=(conv >> 24) & 0xFF,
                   modules=w[7])
        logger.info("FPGA capabilities: %s", caps)
        return caps

    def has(self, name):
        """a module (MODULE_BITS) or feature (FEATURE_BITS) of this bitstream"""
        bit = MODULE_BITS.get(name, FEATURE_BITS.get(name))
        if bit is None:
            return name == 'hk'
        return bool(self.modules >> bit & 1)

    @property
    def board_name(self):
        return BOARD_NAMES.get(self.board, 'board %d' % self.board)

    @property
    def adc_dt(self):
        """ADC sample period [s]"""
        return 1.0 / self.adc_hz

    @property
    def adc_full_scale(self):
        """largest ADC code magnitude (2**(bits-1))"""
        return 2 ** (self.adc_bits - 1)

    def module_names(self):
        return [n for n in MODULE_BITS if self.has(n)]

    def __repr__(self):
        feats = [n for n in FEATURE_BITS if self.has(n)]
        return ("%s (caps v%d): ADC %g MS/s %d-bit x%d, DAC %g MS/s %d-bit x%d, "
                "bus %g MHz, modules %s, features %s"
                % (self.board_name, self.version, self.adc_hz / 1e6,
                   self.adc_bits, self.adc_ch, self.dac_hz / 1e6, self.dac_bits,
                   self.dac_ch, self.sys_hz / 1e6, ','.join(self.module_names()),
                   ','.join(feats) or '-'))
