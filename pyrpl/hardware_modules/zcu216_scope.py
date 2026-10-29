"""
The ZCU216's oscilloscope (fpga/rtl/zcu216_scope.sv): the Red Pitaya scope's
registers and acquisition flow, with the trace in DDR instead of block RAM.

- the sample period from the FPGA (register 0x1FC, ps: 400 at 2.5 GS/s), decimation from the FPGA's recommended
  minimum (register 0x1F8: 4, half of the scope's own DDR port; 2 would be a
  word per clock, more than the port's bursts carry) to 65536; a trace that
  lost words anyway (register 0x1DC) is logged;
- data_length from the FPGA (register 0x1E0; 2**18 samples per channel);
- the samples sit in a ring of 2 * data_length in the reserved DDR, four
  consecutive samples of each channel per 16-byte word ([63:0] channel 1,
  [127:64] channel 2); a trace is fetched with one bulk read ('b') through
  monitor_server's memory window, not register reads;
- inputs: RF-ADC tile 224 channels 0-3, the input of FFT channel 0/1, or
  ASG channel 0-3 (bitstreams with the ASG; 100 MS/s, each sample held);
- triggers: off, immediately, channel 1/2 rising or falling edge, and the
  external input (PMOD0_0, J46 pin 1) rising or falling edge (no ASG or DSP
  triggers on this board);
- time average (fpga/rtl/zcu216_scope_tavg.sv, bitstreams from 2026-10-29):
  every edge of the external input (the laser's chirp sync) starts a record,
  and each decimated sample becomes the running mean of the samples at its
  place in the records, over ~2**time_average_log2 chirps, in the FPGA (every
  chirp, not only the traces the PC reads). The samples then carry two
  fraction bits, which _get_trace scales back.

The registers sit in the upper half of FPGA slot 1 (client 0x4030_0000); the
lower half is the FFT block.
"""
import numpy as np

from .scope import Scope, DecimationRegister
from ..attributes import SelectProperty, SelectRegister, IntRegister, \
    BoolProperty, IntProperty, BoolRegister
from ..pyrpl_utils import sorted_dict, time

# monitor_server's memory window: client address <-> physical address
MEM_WIN_CLIENT = 0x48000000
MEM_WIN_PHYS = 0x1E000000


class _TimeAverageRegister(BoolRegister):
    """time_average, which also sets the trace scale: the means carry two
    fraction bits (4x the samples' scale)"""
    def get_value(self, obj):
        v = super().get_value(obj)
        obj._ta_on = bool(v)
        return v

    def set_value(self, obj, val):
        super().set_value(obj, val)
        obj._ta_on = bool(val)


class Zcu216Scope(Scope):
    addr_base = 0x40300000
    name = 'scope'

    _ta_attributes = ["time_average", "time_average_log2",
                      "time_average_falling"]
    _gui_attributes = Scope._gui_attributes + _ta_attributes
    _setup_attributes = Scope._gui_attributes + ["rolling_mode"] + \
        _ta_attributes

    _ta_on = False
    time_average = _TimeAverageRegister(
        0x1C8, 0, default=False,
        doc="FPGA time average: each sample becomes the running mean of the "
            "samples at its place after the external input's edges (one "
            "record per chirp, up to time_average_max_samples), over "
            "~2**time_average_log2 chirps. Switching it on starts over; "
            "the first edge after that loads the mean. Trigger on the "
            "external input as well, so a trace starts on a record.")
    time_average_log2 = IntRegister(
        0x1D0, bits=4, default=8,
        doc="log2 of the chirps in the time average: the mean weighs each "
            "new chirp 2**-k (after a warm-up of 2**k chirps), so the "
            "uncorrelated noise power drops by ~2**(k+1)")
    time_average_falling = BoolRegister(
        0x1CC, 0, default=False,
        doc="time-average records start on the external input's falling "
            "edge (else rising)")
    _ta_records = IntRegister(0x1D4, doc="records since the restart (saturating)")
    _ta_period = IntRegister(0x1D8, doc="[16:0] samples in the last whole record, "
                                        "[31:24] log2 of the most per record")

    def time_average_restart(self):
        """start the time average over (the next edge loads the mean)"""
        self._write(0x1D4, 1)

    @property
    def time_average_records(self):
        """chirps in the time average since its restart (saturates at 65535)"""
        return int(self._ta_records)

    @property
    def time_average_max_samples(self):
        """decimated samples per record the time average keeps (the rest of
        a longer chirp is 0); 0: the bitstream has no time average"""
        tpl = (int(self._ta_period) >> 24) & 0xFF
        return 1 << tpl if tpl else 0

    @property
    def time_average_period(self):
        """decimated samples in the last whole record (the chirp period);
        time_average_max_samples means as long or longer, or no edge yet"""
        return int(self._ta_period) & 0x1FFFF

    @property
    def _sample_scale(self):
        return 2 ** 15 if self._ta_on else 2 ** 13

    # Scope's FFT / point-stream settings are merged into every subclass's
    # setup; here they are software only, so setup() never writes register 0x0
    # (bits 4/5 there, read-modify-write) or 0x88 for them
    fft_enable = BoolProperty(default=False, doc="no FFT in this scope")
    fft_parallel = BoolProperty(default=False, doc="no FFT in this scope")
    nfft = IntProperty(default=0, doc="no FFT in this scope")

    _inputs = sorted_dict({"adc0": 0, "adc1": 1, "adc2": 2, "adc3": 3,
                           "fft0": 4, "fft1": 5,
                           "asg0": 8, "asg1": 9, "asg2": 10, "asg3": 11},
                          sort_by_values=True)
    inputs = _inputs.keys()
    input1 = SelectRegister(0x1F0, options=_inputs, default="adc0",
                            doc="channel 1: RF-ADC tile 224 channel 0-3, the "
                                "input of FFT channel 0/1, or ASG channel 0-3")
    input2 = SelectRegister(0x1F4, options=_inputs, default="adc1",
                            doc="channel 2: RF-ADC tile 224 channel 0-3, the "
                                "input of FFT channel 0/1, or ASG channel 0-3")

    _trigger_sources = sorted_dict({"off": 0,
                                    "immediately": 1,
                                    "ch1_positive_edge": 2,
                                    "ch1_negative_edge": 3,
                                    "ch2_positive_edge": 4,
                                    "ch2_negative_edge": 5,
                                    "ext_positive_edge": 6,   # PMOD0_0
                                    "ext_negative_edge": 7},
                                   sort_by_values=True)
    trigger_sources = _trigger_sources.keys()
    _trigger_source_register = SelectRegister(0x4, doc="Trigger source",
                                              options=_trigger_sources)
    trigger_source = SelectProperty(default='immediately',
                                    options=_trigger_sources.keys(),
                                    doc="Trigger source for the scope. Use "
                                        "'immediately' if no synchronisation "
                                        "is required. Ignored in rolling_mode.",
                                    call_setup=True)

    _decimations = sorted_dict({2 ** n: 2 ** n for n in range(1, 17)},
                               sort_by_values=True)
    decimations = _decimations.keys()      # all the FPGA takes; see _decimation_options
    _min_decimation = IntRegister(0x1F8, doc="recommended minimum decimation")
    _ring_status = IntRegister(0x1DC, doc="[0] words lost since the arm, [1] AXI error")

    def _decimation_options(self):
        """decimations from the FPGA's recommended minimum on (read once)"""
        m = getattr(self, '_min_decimation_cache', None)
        if m is None:
            try:
                v = int(self._min_decimation)
            except (TypeError, ValueError):     # no link (yet): do not cache
                v = 0
            else:
                m = self._min_decimation_cache = v if v in self._decimations else 2
            m = m or 2
        return sorted_dict({d: d for d in self._decimations if d >= m},
                           sort_by_values=True)

    decimation = DecimationRegister(0x14, doc="decimation factor",
                                    default=2 ** 13,
                                    options=lambda obj: obj._decimation_options(),
                                    call_setup=True)

    _ring_base = IntRegister(0x1E4, doc="physical address of the sample ring")
    _ring_length = IntRegister(0x1E8, doc="sample ring length [samples]")

    def _ring(self):
        """(client address of the ring, its length in samples), read once"""
        r = getattr(self, '_ring_cache', None)
        if r is None:
            r = (MEM_WIN_CLIENT + int(self._ring_base) - MEM_WIN_PHYS,
                 int(self._ring_length))
            self._ring_cache = r
        return r

    def _ring_samples(self, end, n):
        """the n samples of both channels that end at ring index end (int16)"""
        addr, length = self._ring()
        start = (end - n + 1) % length
        # whole 16-byte words, fetched in 1 KB units (the ring is a multiple of 1 KB)
        w0 = start // 4
        nw = (start % 4 + n + 3) // 4
        words_total = length // 4
        chunks, w, left = [], w0, nw
        while left > 0:
            take = min(left, words_total - w)       # up to the ring's end, then wrap
            b0 = (w * 16) // 1024 * 1024
            b1 = -(-((w + take) * 16) // 1024) * 1024
            d = self._client.bulk(addr + b0, b1 - b0)
            if d is None:
                raise IOError("bulk read of the scope ring failed")
            d = np.frombuffer(bytes(d), dtype='<i2').reshape(-1, 8)
            chunks.append(d[(w * 16 - b0) // 16:(w * 16 - b0) // 16 + take])
            left -= take
            w = 0
        d = np.concatenate(chunks)
        first = start % 4
        ch1 = d[:, :4].reshape(-1)[first:first + n]
        ch2 = d[:, 4:].reshape(-1)[first:first + n]
        return ch1, ch2

    def _trace_end(self):
        """ring index of a triggered trace's last sample"""
        return (self._write_pointer_trigger + self._trigger_delay_register) \
            % self._ring()[1]

    def _get_trace(self):
        st = self._ring_status
        if st & 1:
            self._logger.warning("scope trace lost words: its DDR port fell behind "
                                 "(decimation %d)", self.decimation)
        if st & 2:
            self._logger.warning("scope: a DDR write was refused (AXI error)")
        ch1, ch2 = self._ring_samples(self._trace_end(), self.data_length)
        return np.array((ch1, ch2), dtype=float) / self._sample_scale

    def _get_ch(self, ch):
        if ch not in [1, 2]:
            raise ValueError("channel should be 1 or 2, got " + str(ch))
        return self._get_trace()[ch - 1]

    @property
    def _data_ch1(self):
        return self._get_ch(1)

    @property
    def _data_ch2(self):
        return self._get_ch(2)

    @property
    def _rawdata_ch1(self):
        return self._ring_samples(self._write_pointer_current, self.data_length)[0]

    @property
    def _rawdata_ch2(self):
        return self._ring_samples(self._write_pointer_current, self.data_length)[1]

    def _get_rolling_curve(self):
        """the data_length samples up to the newest one in DDR (the ring is
        twice that long, so the writer does not reach them during the read)"""
        times = self.times
        times -= times[-1]
        ch1, ch2 = self._ring_samples(self._write_pointer_current, self.data_length)
        datas = np.zeros((2, len(times)))
        if self.ch1_active:
            datas[0] = ch1 / self._sample_scale
        if self.ch2_active:
            datas[1] = ch2 / self._sample_scale
        return times, datas

    def _start_trace_acquisition(self):
        """Scope._start_trace_acquisition without the FFT point stream"""
        autosave_backup = self._autosave_active
        self._autosave_active = False      # nothing into the config file during setup
        self._acquisition_started = True
        self._reset_writestate_machine = True
        if self.trigger_source == 'immediately':
            self._trigger_delay_register = self.data_length
        else:
            delay = int(np.round(self.trigger_delay / self.sampling_time)) \
                + self.data_length // 2
            self._trigger_delay_register = min(max(delay, 1), 2 ** 32 - 1)
        self._trigger_armed = True
        self._trigger_source_register = self.trigger_source
        self._autosave_active = autosave_backup
        self._last_time_setup = time()
