# PLAN — CFAR baseline ramp (slanted-shoulder flattening)

Status: PROPOSED, nothing implemented. Alternative/complement to the CFAR
`retry` mechanism (see `HANDOFF_cfar_retry.md`, `docs/CfarPeakDetector.md`).

## Problem restated

Candidate starvation is an **argmax selection** failure. With a strong
reflection just below the cutoff, the frame's global argmax pins to the
cutoff-adjacent skirt shoulder every frame. The CFAR test then *correctly*
rejects it (edge guard / z-test), and because the detector emits exactly one
candidate per frame, a genuine target further out is never tested.

Existing mitigations:

| Mechanism        | Reg    | What it does                                     |
| ---------------- | ------ | ------------------------------------------------ |
| `cfar_retries`   | `0xA0` | re-sweep for the next candidate after a failure  |
| `cfar_onesided`  | `0xA4` | one-sided test inside the edge-guard dead zone   |
| (this plan) ramp | new    | stop the shoulder from winning the argmax at all |

## Core idea

Subtract a user-drawn piecewise-linear baseline (drawn in **dB**, see below)
from the magnitudes used for **candidate selection only**. The shoulder's slant
is removed, so the argmax lands on the genuine peak instead of the skirt.

    L(bin) = A1 + slope * (bin - b1)     for b1 <= bin <= b2   [dB units]
           = A2                          for bin > b2      (HOLD-LAST)
           = (not applicable)            for bin < b1 = start_index

    s_sel = log2approx(s) - L(bin)       // selection compare only, LOG domain
    s     = unchanged                    // window stats, interp, output value

### Why this is safe

The CFAR z-test is invariant to a locally-constant offset: over a window of
~2*(G+T) bins the ramp is essentially constant, and `diff = P - mean` and
`stdev` shift together. Applying the correction to the **selection compare
only** makes that invariance exact rather than approximate — `mag[]`,
`mag2[]`, the reference-band accumulate, the parabolic interpolation, and the
reported `value` all still see raw magnitudes. Consequence: **the ramp cannot
change the false-alarm rate**, only which bin gets tested. This is the same
discipline the software retry already uses (masked SELECTION copy, tests read
unmasked data — `lidar/lidar.py:1623`).

`data_min` stays on the RAW magnitude (it is an absolute amplitude floor).

### Why HOLD-LAST beyond b2, not "correction ends"

If the correction drops to 0 at `b2`, the effective baseline steps UP by
`A2` at that bin, manufacturing a new argmax attractor immediately to its
right — the original failure mode, relocated. The second node therefore ends
the **slope**, not the correction.

### The correction is linear in dB, i.e. LOG domain

The spectrum plot has a linear Y *axis* but the plotted *data* is already in
dB: `display_unit` defaults to `dB(Vpk^2)` = `10*log10(power)` = `20*log10(s)`
(`lidar/lidar.py:498`, `1354-1356`). A straight line drawn on that display is
therefore **constant dB per bin** — geometric/exponential decay in raw
magnitude, NOT linear. This also matches the physics: an internal-reflection
pedestal falls off roughly exponentially, which is what plots straight in dB.

A geometric ramp applied in the raw domain would need a per-lane multiply with
a LOOP-CARRIED recurrence in the II=1 STREAM loop — precisely the structure
that measured 9.7-16.1 ns and killed the top-K attempt (`HANDOFF_cfar_retry.md`).
Rejected.

Instead, move the COMPARE into an approximate-log2 domain, where the
correction is a plain subtract again:

    code(s) = log2approx(s)          // Mitchell: LZC + normalize, NO DSP
    s_sel   = code(s) - L(bin)       // L linear in bin -> per-lane accumulator

`log2approx` is the classic Mitchell approximation: leading-zero count gives
the exponent, the normalized remainder is the mantissa, `log2(x) ~= e + f` for
`x = 2^e (1+f)`. Error <= ~0.086 in log2 (~0.26 dB) — irrelevant for candidate
selection.

Why this is the right shape for this design:

- **The ramp stays an adder.** Linear-in-dB is linear in log2, so the per-lane
  accumulator survives; no multiplier anywhere.
- **Purely feed-forward.** `code(s)` depends only on `s` and `bin`, with no
  loop-carried dependency, so it registers into its OWN pipeline stage ahead of
  the existing two-cycle argmax merge. It costs latency, not II, and never
  touches the recurrence. This is the core timing argument.
- **Monotone => argmax-safe.** `log2approx` is monotone increasing, so with the
  ramp off it selects the same bin as a raw compare. Still MUX back to the raw
  compare when `step == 0` so the regression is bit-exact rather than argued
  (mantissa truncation could otherwise alter tie-breaking between equal-ish bins).
- **Registers in drawn units.** Slope is programmed as dB/bin; the host converts
  only by `dlog2 = dB / 6.0206` (since display dB = 20*log10(s)).
- Selection-only still holds: the CFAR test, window stats, interpolation and the
  reported `value` never see the log domain.

Cost estimate: LZC(DSZ=28) + barrel shift + subtract ~ 100-150 LUT per lane,
~600 LUT at SSR=4, zero DSP. Tight at 82% LUT but an order of magnitude below
the UF=4 variant that failed placement (7.3k LUT/ch).

### No multiplier for the ramp itself

After the `BEAT` loop `UNROLL`, lane `ch` sees `bin = beat_idx*FSSR + ch`, so
`L(bin)` is affine in the loop counter. Each lane keeps an accumulator
initialised to `A1 + slope*ch` and incremented by `step_per_beat =
slope*FSSR` once per beat — one add per lane per cycle, a trivially short
loop-carried recurrence. The host programs `step_per_beat` directly as signed
fixed point (Q(int).(frac), frac ~10-12 so small slopes are representable);
accumulator width `DSZ + FRAC`.

## Phase 1 — software replica first (no FPGA build)

Prove the idea on live/recorded spectra before spending a bitstream.

- `lidar/lidar.py` `_cfar_peak(self, data, cutoff_idx)`: build the selection
  array `sel` (line ~1623) from a ramp-corrected copy; leave `mag`/`data`
  (lines ~1610, floor test ~1626, z-test ~1660) untouched. In software this is
  literally `sel_dB = 10*np.log10(data) - L(f)` (or equivalently
  `sel = data / 10**(L/10)`) — no log2 approximation needed; the hardware's
  Mitchell error (~0.26 dB) is the only sw/hw divergence and is below the noise.
- Bin-scale caveat: the sw spectrum is zero-padded by `scale =
  round(padding / 2**nfft)` (~8x, lidar.py:1616). The ramp is defined in
  FREQUENCY (Hz), so convert per path rather than reusing hardware bin
  numbers — same trap that guard/train hit.
- New `Lidar` attributes (Hz / amplitude, not bins), alongside `cutoff`:
  `ramp_a1`, `ramp_f2`, `ramp_a2`, plus `ramp_enable` (default OFF).
  Follow `cutoff`'s no-`call_setup` pattern so dragging stays cheap.
- Success metric: on the recorded 14.5 MHz starvation capture, detections go
  10/10 with `cfar_retries = 0`.

## Phase 2 — GUI: two-segment cutoff control

Replace/extend `DragLine` cutoff (`lidar_widget.py:130-160`) with a custom
`pg.GraphicsObject` modelled on **`FftWindowBands`** (`lidar_widget.py:229`),
NOT `PolyLineROI` (no polyline/ROI items exist in either repo; `FftWindowBands`
already solves per-part `_hit` picking, `EDGE_PX`, Ctrl-gating, ESC-abort
restore, rate-limited commit, `setZValue`, and `dataBounds -> [None, None]`).

Geometry:

```
   |                            y = amplitude
   |  <- vertical segment, top open/infinite (unchanged cutoff semantics)
   |
   N1 ------____                N1 = (cutoff_freq, A1)   drag X and Y
                ----____ N2     N2 = (f2, A2)            drag X and Y
                          ~~~~~ hold-last beyond N2
```

- N1's X keeps the existing live `module.cutoff` write; its Y adds `ramp_a1`.
- N2 sets `ramp_f2` / `ramp_a2`; constrain `f2 > cutoff`.
- Dragging the vertical segment body = move cutoff (today's behaviour, so
  muscle memory survives). Node grabs are the new gesture.
- Ramp disabled (`A1 = A2 = 0`) should render as today's plain vertical line so
  the control does not look more complicated than it is when unused.
- Y units: the nodes live in whatever `display_unit` is showing (default
  `dB(Vpk^2)`). Store `ramp_a1`/`ramp_a2` in dB and convert in a SINGLE helper
  used by both the widget and the register push (`dlog2 = dB / 6.0206`).
  If `display_unit` is switched to a linear unit, either convert the node
  positions for display or grey the ramp nodes out — do not let a dB-defined
  ramp be dragged against a linear axis.

## Phase 3 — hardware

### HLS (`hls/peak_detector_cfar.cpp`, `hls/peak_detector.h`)

- New `cfar_stream_stage` args (under `#ifdef PEAK_CFAR`): `ramp_a1`,
  `ramp_step` (signed, log2 units/bin), `ramp_end_bin`.
- New `log2approx()` helper: LZC over `DSZ=28` bits + barrel shift to normalize;
  output `LOG_W = 5 + LOG_FRAC` bits (`LOG_FRAC` ~ 10-12). Placed in its own
  pipeline stage (feed-forward, no recurrence).
- Per-lane ramp accumulator as above; `s_sel = log2approx(s) - L(bin)`
  (signed, no clamp needed — it is only ever compared); the existing candidate
  compare uses `s_sel`, while `mag[ch][beat] = s` / `mag2[...] = s` keep the RAW
  value (lines ~136-141). `data_min` continues to test raw `s`.
- `ramp_step == 0` MUXes the compare back to raw `s` for a bit-exact regression
  path.
- Hold-last: freeze the accumulator once `bin >= ramp_end_bin` (per-lane
  compare, feed-forward, no new recurrence).
- Retry interaction: the re-sweeps in `cfar_detect_stage` also select by
  argmax and must use the SAME corrected metric, or attempt 2+ will re-pin to
  the shoulder. Either recompute the ramp in the sweep (cheap, affine again) or
  have the stream stage buffer `s_sel` in a third small copy. **Recompute** —
  BRAM already went 32 -> 48 RAMB18-eq for `mag2`.
- Verify STREAM II=1 is preserved and the reported worst path is still the
  pre-existing 81-bit k^2*V multiply (6.978 ns), not the new adder.

### RTL / registers

- `rtl/red_pitaya_scope.sv`: 3 new sysbus regs in the free gap (0xA0/0xA4
  taken): **`0xA8` ramp_a1**, **`0xAC` ramp_step (signed)**, **`0xB0`
  ramp_end_bin**; reset all 0 = feature off = bit-exact current behaviour.
  Write + readback + per-channel copies to both `fft_proc` instances.
- `rtl/fft_proc.sv`: new inputs + `*_arg` registers, wired under
  `` `ifdef PEAK_CFAR `` alongside `fft_cfar_retry_in` (lines ~27-28, 287-289,
  994-1019).
- `ip/peak_detector_bd.tcl`: 3 new BD ports in the cfar block.
- `pyrpl/hardware_modules/scope.py`: `fft_ramp_a1 = IntRegister(0xA8)`,
  `fft_ramp_step`, `fft_ramp_end` (near `fft_cfar_*`, scope.py:506-510).
- Host push next to guard/train/retries in `Lidar._setup()` (lidar.py:1968-1976),
  converting Hz -> hardware bins and amplitude -> the `data_min` scale
  (`min_amplitude * 2**(width-3)` idiom, lidar.py:1968).

### Testbench (`hls/peak_detector_cfar_tb.cpp`)

- [7] slanted-shoulder spectrum, ramp OFF, retry 0 -> argmax pins to cutoff,
  no detection (reproduces the bug).
- [8] same spectrum, ramp ON, retry 0 -> genuine peak detected, and the
  reported `value` equals the RAW magnitude (proves selection-only).
- [9] ramp at defaults (all zero) -> bit-identical to [1]-[6] (regression).
- [10] hold-last: a bin just past `ramp_end_bin` must not become a spurious
  argmax.

## Build / rollout

- `PROFILE=ssr4n9 ./make.sh` (ask first — build approval is per-instance).
  Watch LUT: the retry detector is already 5.4k LUT/ch at 82% total.
- Expect the ramp to make retries rare -> `cfar_retries` can drop to 1-2,
  retiring the ~8% worst-case point-rate derate at retry=3.
- Report WNS **and** WHS. Snapshot `out.d/<slug>` on success.

## Calibration procedure (the shoulder is static)

The shoulder is an **internal reflection**: at constant laser power it is a
fixed feature of the optical head, not a per-scene quantity. So the ramp is a
CALIBRATION CONSTANT, stored in `lidar.yml`, not something to retune per target.
That largely removes the "manual static fit vs adaptive retry" objection.

Suggested procedure:

1. Block the external return so only the internal reflection is present.
2. Capture / average the spectrum (dB display).
3. Drag N1 to the cutoff and N2 out to where the pedestal reaches the noise
   floor; the segment should lie just ON the pedestal.
4. Unblock, confirm real targets now win the argmax with `cfar_retries = 0`.
5. Re-run only when laser power or the optical head changes.

## Open questions

1. **Auto-fit.** Fit the line to a rolling percentile of the observed noise
   floor and set both nodes automatically (the generalisation of the
   "adaptive cutoff" idea already in the notes). Zero FPGA cost, and with a
   static shoulder it is a convenience rather than a necessity. Phase 4.
2. **Per-channel ramps.** `cutoff` is single-valued while up/down chirps have
   different skirts. v1 inherits that; revisit if the two shoulders differ
   materially in captures.
3. **`LOG_FRAC` width.** Trades LUT against tie-breaking fidelity and Mitchell
   error. Start at 12 and check the LZC+shifter LUT cost in csynth before
   trusting the ~600 LUT estimate.
4. **Curvature.** If a single dB-linear segment turns out to under-fit the
   pedestal, the same machinery extends to K segments (a small per-segment
   step LUT + boundary compare) with no change to the timing story. Only do
   this if the captures demand it.
