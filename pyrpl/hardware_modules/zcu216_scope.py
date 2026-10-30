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
  triggers on this board).

The registers sit in the upper half of FPGA slot 1 (client 0x4030_0000); the
lower half is the FFT block (fpga/rtl/zcu216_fft_test.sv), which this module
also drives for the lidar's FPGA FFT mode: a real FFT of an RF-ADC channel at
the full 2.5 GS/s (not the scope's decimated rate), magnitude frames fetched
through a frame snapshot into DDR without waiting (_fftdata).
Bitstreams with chirp framing (lidar/docs/Zcu216ChirpFft.md) start the frames
on the laser's chirp sync: channel 0 transforms the up window, channel 1 the
down window, set with the Red Pitaya scope's fft window registers
(fft_wait1_cnt .. fft_acq2_cnt, fft_trig_delay: here in ADC samples) and
fft_framing; older ones frame free-running. With fft_average = n each
channel's frames are the RMS of 2**n frames' power (zcu216_fft_avg).
"""
import numpy as np

from .scope import Scope, DecimationRegister
from ..attributes import SelectProperty, SelectRegister, IntRegister, \
    BoolProperty, IntProperty, BoolRegister
from ..pyrpl_utils import sorted_dict, time
from .dsp import dsp_addr_base

# monitor_server's memory window: client address <-> physical address
MEM_WIN_CLIENT = 0x48000000
MEM_WIN_PHYS = 0x1E000000

# the FFT block (lower half of slot 1) and its frame snapshot registers
FFT_BASE = 0x40200000
FFT_SNAP = FFT_BASE + 0x140
DSP_FFT_SEL = 0x40600000 + 28 * 0x10000            # the FFT sinks: DSP numbers 28, 29
DSP_TONE = 30                                      # ... their tone generator
FFT_CORDIC_K = 1.64676                    # zcu216_fft_mag: mag = K |X| / 2
FFT_IN_W, FFT_DW = 14, 27                 # input bits, the cores' output cut


class Zcu216Scope(Scope):
    addr_base = 0x40300000
    name = 'scope'

    _setup_attributes = Scope._gui_attributes + ["rolling_mode"]

    # Scope's FFT / point-stream settings are merged into every subclass's
    # setup; here they are software only, so setup() never writes register 0x0
    # (bits 4/5 there, read-modify-write) or 0x88 for them
    fft_enable = BoolProperty(default=False, doc="no FFT in this scope")
    fft_parallel = BoolProperty(default=False, doc="no FFT in this scope")
    nfft = IntProperty(default=0, doc="no FFT in this scope")

    # the signal hub (slot 3, rtl/zcu216_dsp.sv): the Red Pitaya DSP's numbers
    # and addresses; the signals with no Red Pitaya equivalent have numbers it
    # leaves free
    _inputs = sorted_dict({"asg0": 8, "asg1": 9, "asg2": 10, "asg3": 11,
                           "adc0": 12, "adc1": 13, "adc2": 22, "adc3": 23,
                           "out1": 14, "out2": 15, "out3": 24, "out4": 25,
                           "fft0": 26, "fft1": 27, "off": 21,
                           "pid0": 0, "iir": 4, "iq0": 5},
                          sort_by_values=False)  # adc0 first: the default
    inputs = _inputs.keys()
    input1 = SelectRegister(dsp_addr_base('asg0') - addr_base, options=_inputs,
                            default="adc0",
                            doc="channel 1: an RF-ADC channel, an ASG channel, "
                                "a DAC output, a DSP module, or the input of "
                                "FFT channel 0/1")
    input2 = SelectRegister(dsp_addr_base('asg1') - addr_base, options=_inputs,
                            default="adc1",
                            doc="channel 2: an RF-ADC channel, an ASG channel, "
                                "a DAC output, a DSP module, or the input of "
                                "FFT channel 0/1")

    _trigger_sources = sorted_dict({"off": 0,
                                    "immediately": 1,
                                    "ch1_positive_edge": 2,
                                    "ch1_negative_edge": 3,
                                    "ch2_positive_edge": 4,
                                    "ch2_negative_edge": 5,
                                    "ext_positive_edge": 6,   # PMOD0_0
                                    "ext_negative_edge": 7,
                                    "asg0": 8,    # each channel's chirp start
                                    "asg1": 9,
                                    "asg2": 11,
                                    "asg3": 12},
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
        return np.array((ch1, ch2), dtype=float) / 2 ** 13

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
            datas[0] = ch1 / 2 ** 13
        if self.ch2_active:
            datas[1] = ch2 / 2 ** 13
        return times, datas

    # ---- the FFT block (FPGA FFT mode) -----------------------------------

    def _fft_id(self):
        """(log2 N, channels) of the FFT block, read once; (0, 0) without one"""
        v = getattr(self, '_fft_id_cache', None)
        if v is None:
            w = int(self._client.reads(FFT_BASE, 1)[0])
            v = ((w >> 8) & 0xFF, w & 0xFF) if w >> 16 == 0xF0F6 else (0, 0)
            self._fft_id_cache = v
        return v

    @property
    def fft_length(self):
        """the FFT block's transform length (real input; fft_length / 2
        bins), 0 without one"""
        lg = self._fft_id()[0]
        return 1 << lg if lg else 0

    # chirp framing (zcu216_fft_frame); fft_wait1_cnt .. fft_acq2_cnt (0x58 ..
    # 0x64) and fft_trig_delay (0x1A0) are the inherited Scope registers, in
    # ADC samples here
    fft_framing = BoolRegister(0x1C8, 0, doc="FFT frames start on their "
                               "trigger: channel 0 the up window, channel 1 "
                               "the down window; off: free-running frames")
    fft_sync_falling = BoolRegister(0x1C8, 1, doc="chirp sync on the external "
                                    "input's falling edge (else rising)")
    fft_trigger_sync = BoolRegister(0x1C8, 2, doc="FFT frames start on the "
                                    "scope's trigger (else on "
                                    "fft_trigger_source), as on the Red Pitaya")
    fft_trigger_source = SelectRegister(0x1C8, bitmask=0xF0,
                                        options={"ext": 0x60, "asg0": 0x80,
                                                 "asg1": 0x90, "asg2": 0xB0,
                                                 "asg3": 0xC0},
                                        doc="what starts the FFT frames, in "
                                            "the scope's trigger codes: the "
                                            "external input or an ASG "
                                            "channel's chirp start")
    fft_syncs = IntRegister(0x1CC, doc="chirp syncs seen")
    fft_sync_period = IntRegister(0x1D0, doc="ADC samples between the last two "
                                  "chirp syncs")
    _fft_skipped = IntRegister(0x1D4, doc="frames skipped (engine busy): [15:0] "
                               "up, [31:16] down")
    _fft_len_reg = IntRegister(0x8C, doc="FFT length; 0 on bitstreams without "
                               "chirp framing")
    fft_average = IntRegister(0x1D8, bits=4, min=0, max=8,
                              doc="the FFT frames are the RMS of 2**n frames' "
                                  "power (a per-bin power average over 2**n "
                                  "chirps when framed); 0 = every frame as is. "
                                  "Writing restarts the average.")

    def fft_average_restart(self):
        """start the power average over (the next frame begins a group)"""
        self._write(0x1D8, int(self.fft_average))

    @property
    def fft_framing_available(self):
        """the bitstream frames on the chirp sync (read once)"""
        v = getattr(self, '_fft_framing_cache', None)
        if v is None:
            v = self._fft_framing_cache = int(self._fft_len_reg) != 0
        return v

    @property
    def fft_skipped(self):
        """(up, down) frames skipped since reset: the engine was busy"""
        v = int(self._fft_skipped)
        return v & 0xFFFF, v >> 16

    @property
    def fft_sample_time(self):
        """the FFT's sample period [s]: the RF-ADC rate, not decimated"""
        return self._base_sampling_time

    def fft_setup(self, ch, source):
        """FFT channel ch transforms `source`, a scope input name (anything
        else: its tone generator), and the block runs"""
        self._client.writes(DSP_FFT_SEL + 0x10000 * ch,
                            [self._inputs.get(source, DSP_TONE)])
        self._client.writes(FFT_BASE + 0x04, [1])

    def _fft_arm(self, ch):
        """arm a one-frame snapshot of channel ch's magnitudes (taken at the
        next frame start)"""
        c = self._client
        if int(c.reads(FFT_SNAP + 0x0C, 1)[0]) & 1:     # a capture still pending
            c.writes(FFT_SNAP + 0x00, [0x80000000])     # drop it
        c.writes(FFT_SNAP + 0x04, [ch & 3])         # |X| of channel ch
        c.writes(FFT_SNAP + 0x08, [1])              # one frame
        c.writes(FFT_SNAP + 0x00, [1])              # arm

    def _fft_fetch(self):
        """the finished snapshot, fetched in one bulk read, as the amplitude
        of each bin (fft_length / 2 bins, natural order): a tone of amplitude
        A (full scale 1) reads A / 2, as the lidar's normalized host FFT. None
        if the read failed."""
        c = self._client
        base = int(c.reads(FFT_SNAP + 0x14, 1)[0])
        nbytes = int(c.reads(FFT_SNAP + 0x18, 1)[0])
        d = c.bulk(MEM_WIN_CLIENT + base - MEM_WIN_PHYS, nbytes)
        if d is None:
            return None
        mag = np.frombuffer(bytes(d), dtype='<u4').astype(float)
        # mag = K |X'| / 2, X' = X / 2**cut (the sub-FFT outputs cut to FFT_DW
        # bits); |X| of a tone of amplitude A (2**13 = full scale) is
        # A 2**13 N / 2
        lg = self._fft_id()[0]
        cut = max(0, FFT_IN_W + (lg - 3) - FFT_DW)
        return mag * (2.0 * 2 ** cut / FFT_CORDIC_K / 2 ** 13 / (1 << lg))

    def fft_frame(self, ch=0, timeout=0.5):
        """One whole frame of FFT channel ch, waiting for it: arm, poll, fetch
        (see _fft_fetch). None if it did not finish. For scripts: the lidar
        reads _fftdata, which never waits."""
        self._fft_armed = None                      # takes over the snapshot
        self._fft_arm(ch)
        t0 = time()
        while int(self._client.reads(FFT_SNAP + 0x0C, 1)[0]) & 1:
            if time() - t0 > timeout:
                self._logger.warning("FFT frame snapshot not done after %g s", timeout)
                return None
        return self._fft_fetch()

    _fft_armed = None           # channel of the snapshot in progress
    _fft_rows = None            # the latest frame of each channel
    _fft_syncs_seen = None      # fft_syncs at the last _fftdata
    _fft_no_sync = False        # warned that the syncs stopped

    def _fft_watch_syncs(self):
        """warn once when framing runs without chirp syncs: no frame starts,
        the rows stay as they are"""
        syncs = int(self.fft_syncs)
        moving = syncs != self._fft_syncs_seen
        self._fft_syncs_seen = syncs
        if moving == (not self._fft_no_sync):
            return
        self._fft_no_sync = not moving
        if moving:
            self._logger.info("FFT: chirp syncs are back")
        else:
            self._logger.warning("FFT: no chirp syncs, so no frames: the "
                                 "spectrum holds its last frame")

    @property
    def _fftdata(self):
        """the lidar's FPGA-mode rows (down, up), the latest frame of each,
        without waiting: the FFT runs on its own triggers, not the scope's.
        With chirp framing channel 1's and channel 0's frames; free-running
        frames are not split into the chirp's halves, so the whole spectrum
        is the up row and the down row is empty. One snapshot is in progress
        at a time, so this collects it if it finished and arms the next
        channel's; a row keeps its frame until a newer one is in, and the two
        rows may be a frame or more apart."""
        n = self.fft_length // 2
        if self._fft_rows is None or len(self._fft_rows[0]) != n:
            self._fft_rows = [np.zeros(n), np.zeros(n)]
            self._fft_armed = None
        framed = self.fft_framing_available and self.fft_framing
        if framed:
            self._fft_watch_syncs()
        else:
            self._fft_rows[1] = np.zeros(n)
        ch = self._fft_armed
        if ch is not None:
            if int(self._client.reads(FFT_SNAP + 0x0C, 1)[0]) & 1:
                return [self._fft_rows[1], self._fft_rows[0]]
            mag = self._fft_fetch()
            if mag is not None and len(mag) == n:
                self._fft_rows[ch] = mag
        self._fft_armed = (1 - ch if ch is not None else 0) if framed else 0
        self._fft_arm(self._fft_armed)
        return [self._fft_rows[1], self._fft_rows[0]]

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
