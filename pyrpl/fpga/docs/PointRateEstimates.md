# FMCW Point-Rate Estimates

Per-channel (fft_a) point-cloud rate as a function of FFT config (SSR, NFFT, FFT
clock). Single ADC channel = **125 Msps**, fixed. FFT is `FFT_IMPL=5` (direct
`hls::fft`), per-channel (fft_a + fft_b are independent instances).

## Model

- **1 point = 1 chirp.** With a **triangular** FMCW chirp, each FFT frame covers one
  ramp (half the chirp period); a point needs the beat from **both** ramps to solve
  range and velocity (R ∝ f_up+f_down, v ∝ f_up−f_down) → **2 FFT frames per point.**
- The FFT transforms N = 2^NFFT points per frame; with zero-padding only part of the
  window need be real ADC samples (a "ramp"), the rest is padding.

Two regimes per frame, then ÷2 for up+down:

| Quantity | Formula |
|---|---|
| FFT intake capacity | `SSR · f_fft` |
| FFT-throughput ceiling (points/s) | `SSR·f_fft / (2N)` — short padded ramp, FFT-pipeline-bound |
| Acquisition-bound (points/s) | `f_adc / (2N)` = `62.5e6 / N` — full-length ramp, ADC-bound |
| "Short" threshold (ramp ≤ this × N real samples) | `f_adc / (SSR·f_fft)` |

The **acquisition-bound** rate depends only on the ADC and N — it is **identical for
every FFT clock / SSR choice**. The FFT clock/SSR only raise the FFT-throughput
ceiling, which you reach only with **short, zero-padded ramps**.

## "Short" threshold = fraction of one FFT period (= one ramp = ½ chirp)

A ramp is "short" (FFT-bound) when its real samples fill ≤ this % of the FFT window;
equivalently, when the ramp lasts less than the FFT takes to process one frame
(`T_ramp < N/(SSR·f_fft)`). Below it → FFT-bound (max rate); above → ADC-bound.

| Config | Intake (SSR·f_fft) | Short threshold |
|---|---|---|
| SSR=4 / 125 MHz | 500 Msps | 25.0 % |
| SSR=2 / 200 MHz | 400 Msps | 31.25 % |
| SSR=4 / 178 MHz | 712 Msps | 17.6 % |
| SSR=4 / 200 MHz | 800 Msps | 15.6 % |

(Faster FFT ⇒ *lower* threshold: a faster pipeline raises the ceiling but needs
proportionally shorter / more-padded ramps to actually reach it.)

## Point rates (per channel, 2 FFT/point)

### SSR=2 / 200 MHz  (intake 400 Msps, short = 31.25 %)
| NFFT | N | FFT-bound pts/s | Acq-bound pts/s (full ramp) |
|---|---|---|---|
| 13 | 8192 | 24.4 k | 7.6 k |
| 12 | 4096 | 48.8 k | 15.3 k |
| 11 | 2048 | 97.7 k | 30.5 k |

### SSR=4 / 178 MHz  (intake 712 Msps, short = 17.6 %)
| NFFT | N | FFT-bound pts/s | Acq-bound pts/s (full ramp) |
|---|---|---|---|
| 12 | 4096 | 86.9 k | 15.3 k |
| 11 | 2048 | 173.8 k | 30.5 k |

### SSR=4 / 125 MHz  (intake 500 Msps, short = 25.0 %; ships today)
| NFFT | N | FFT-bound pts/s | Acq-bound pts/s (full ramp) |
|---|---|---|---|
| 12 | 4096 | 61.0 k | 15.3 k |
| 11 | 2048 | 122.1 k | 30.5 k |

## FFT-bound ceiling comparison (with 10 % derate for per-frame bubbles)

| Config | Intake | N=12 pts/s | N=12 −10 % | N=11 pts/s | N=11 −10 % |
|---|---|---|---|---|---|
| SSR=4 / 125 MHz (ships) | 500 Msps | 61.0 k | 54.9 k | 122.1 k | 109.9 k |
| SSR=2 / 200 MHz | 400 Msps | 48.8 k | 43.9 k | 97.7 k | 87.9 k |
| SSR=4 / 178 MHz | 712 Msps | 86.9 k | 78.2 k | 173.8 k | 156.4 k |

Each NFFT step down doubles both rates (half the samples per frame). The FFT-bound
ceiling scales with `SSR·f_fft`; e.g. SSR=4/125 (500 Msps) already beats SSR=2/200
(400 Msps), and SSR=4/178 (712 Msps) is highest. Timing closure on the xc7z020
(see `BuildLog.md`): **SSR=2/N13/200 closes** (`fft200ssr2`); **SSR=4/N11/178 closes**
(`fft178ssr4n11`, the highest-throughput closing build); SSR=4/N12 does NOT close
(congestion-bound, ~30 ps short at any clock).

## fft_parallel — dual-engine, 1 FFT/point (doubles the FFT-bound ceiling)

`scope.fft_parallel` (reg `0x0[4]`) is a **runtime** mode bit, not a build knob — it
re-tasks the two existing FFT engines instead of changing the FFT architecture:

| | `fft_parallel=0` (sequential, the model above) | `fft_parallel=1` (parallel) |
|---|---|---|
| fft_a | adc_A, **up + down** ramps (2 frames) | adc_A, **up ramp only** (1 frame) |
| fft_b | adc_B, **up + down** ramps (2 frames) | **adc_A**, **down ramp only** (1 frame) |
| frames streamed / point / engine | **2N** (`fft_length2=2N`) | **N** (`fft_length2=N`) |
| ADC channels | 2 independent | **1** (both engines on adc_A) |

Why it doubles the ceiling: in sequential mode one engine must stream the up-frame
**and** the down-frame back-to-back — **2N** beats/point. The frame is the full padded
window (`fft_proc.sv` injects the zero padding locally, so the engine clocks all N
beats regardless of how few are real ADC samples). In parallel mode fft_a streams the
up-frame (N) while fft_b streams the down-frame (N) **concurrently**, so the per-point
FFT-pipeline cost is **N**, not 2N:

| Regime | Sequential | Parallel | Gain |
|---|---|---|---|
| FFT-bound (short/padded ramp) | `SSR·f_fft / (2N)` | `SSR·f_fft / N` | **2×** |
| Acquisition-bound (full ramp) | `f_adc / (2N)` | `f_adc / (2N)` | **1× (none)** |

The acq floor is **unchanged**: in parallel mode both engines still pull real samples
from the **single adc_A stream**, and the up-ramp and down-ramp samples physically
arrive in sequence — so a full-length up+down pair still costs `2N/f_adc`. Parallel
only buys the FFT-pipeline term, i.e. it helps **exactly when you are FFT-bound** (the
short-ramp / zero-padded regime, below the "short threshold" %). Point rate is the
slower of the two: `min(SSR·f_fft/N, f_adc/(2N))`.

**Sanity anchor (your "double N for free" intuition):** parallel-N12 ≡ sequential-N11
at the same config — e.g. SSR=4/125: parallel N12 = `500e6/4096` = **122.1 k** =
sequential N11 = **122.1 k**. Same wall-clock, double the FFT window.

### Parallel-mode point rates — all dual-channel configs that support fft_parallel

fft_parallel requires **fft_b to exist**, i.e. any dual-channel (`FFT_SINGLE=0`) build:
SSR ∈ {1,2,4}. **SSR=8 does NOT support it** (fft_b is dropped to fit — see below).
FFT-bound column = `SSR·f_fft/N` (already the 2× value); acq floor = `62.5e6/N`,
config-independent. `−10%` = 10 % derate for per-frame bubbles.

| Config (SSR / FFT clk) | Intake | NFFT | N | FFT-bound pts/s | −10 % | Acq floor (full ramp) | Closes? |
|---|---|---|---|---|---|---|---|
| SSR=4 / 125 MHz (ships) | 500 Msps | 12 | 4096 | 122.1 k | 109.9 k | 15.3 k | ✅ |
| SSR=4 / 125 MHz (ships) | 500 Msps | 11 | 2048 | 244.1 k | 219.7 k | 30.5 k | ✅ |
| SSR=4 / 178.571 MHz | 712 Msps | 12 | 4096 | 173.8 k | 156.4 k | 15.3 k | ❌ (N12 congestion) |
| SSR=4 / 178.571 MHz | 712 Msps | 11 | 2048 | 347.7 k | 312.9 k | 30.5 k | ✅ (`fft178ssr4n11`) |
| SSR=2 / 200 MHz | 400 Msps | 13 | 8192 | 48.8 k | 43.9 k | 7.6 k | ✅ (`fft200ssr2`) |
| SSR=2 / 200 MHz | 400 Msps | 12 | 4096 | 97.7 k | 87.9 k | 15.3 k | ✅ |
| SSR=2 / 200 MHz | 400 Msps | 11 | 2048 | 195.3 k | 175.8 k | 30.5 k | ✅ |

These FFT-bound rates are **2× the sequential figures** in the tables above. To actually
reach them you must be below the config's short threshold (SSR=4/125 → 25 %, SSR=4/178 →
17.6 %, SSR=2/200 → 31.25 %); at full-length ramps every config collapses to the shared
acq floor (15.3 k @ N12, 30.5 k @ N11) and parallel mode gives no rate gain.

**Cost of parallel mode:** it consumes the **second ADC channel** (both engines on adc_A),
so you lose independent dual-channel acquisition; and it gives **zero** benefit in the
acquisition-bound (full-ramp / resolution-maximised) regime. It also halves per-engine
input buffering (N vs 2N) and lowers per-point latency. **SSR=8** can't use it at all —
that build drops fft_b, so for SSR=8 the sequential 2-frame/point model is the only option
(see next section).

## Caveats

- **2 FFT/point** assumes a continuous triangle with **no flyback/settling dead time**
  between up and down ramps. Any laser-retune gap adds to the per-point period and
  lowers the rate.
- **Per-frame readout** (peak-detector + histogram) is per *frame*, not per point. At
  the high-N11/SSR4 ceilings the FFT emits a frame every few µs — that readout must
  keep up or it becomes the real bottleneck.
- **Zero-padding gives finer FFT bin spacing, NOT finer range resolution.** Range
  resolution is set by the real chirp **bandwidth** ∝ real-sample count, not N. A
  short padded ramp trades resolution for rate.
- **Acquisition-bound rates (15.3 k / 30.5 k pts/s at N=12 / N=11) are config-
  independent** — if you run full-length ramps for resolution, the FFT clock/SSR
  choice does not change your point rate; it only matters in the short-padded regime.

## SSR=8 (single channel, NFFT=11) — BUILT & CLOSES (`PROFILE=fft178ssr8n11`)

Intake = SSR·f_fft = 8·f_fft. FFT-bound pts/s = `8·f_fft/(2N)` = `f_fft/512` (N=2048).

| FFT clock | Intake | FFT-bound pts/s | −10% | Short threshold | Acq-bound (full ramp) |
|---|---|---|---|---|---|
| 125 MHz | 1000 Msps | 244.1 k | 219.7 k | 12.5 % | 30.5 k |
| **178.571 MHz** (built) | 1429 Msps | 348.8 k | **313.9 k** | 8.75 % | 30.5 k |
| 200 MHz | 1600 Msps | 390.6 k | 351.6 k | 7.81 % | 30.5 k |

**Status: REAL — closes at 178.571 MHz** (adc +0.097, ser +0.123, 0 fail; LUT 68% /
DSP 77% / BRAM 61%; csim PASS — see `BuildLog.md`). Built via the `FFT_SINGLE=1` knob
that drops the second FFT channel (fft_b), so one SSR=8 FFT fits where two SSR=4
channels did — DSP came in at **77%, not the ~98% first estimated** (dropping fft_b
frees more than a per-instance model predicts). So this **~314 k pts/s** (N11, −10%)
config is achievable, **single channel only** (no fft_b). Per-frame readout / detector
capacity is the remaining gate (see below) — and the detector copes (~6% margin).

## Peak detector capacity (from `hls/peak_detector.cpp` + csynth)

The detector's `STREAM` loop is **II=1 with the BEAT loop unrolled over SSR** → it
consumes **SSR bins/clock = the FFT's exact output line rate**. It **emits 1 word per
frame** (the single max peak: value+valid+bin). Per-frame cost ≈ `N/SSR + ~24` cycles
(the ~24 = `ap_ctrl_hs` per-frame restart).

- **SSR=8 / N11 / 178 MHz:** 256 beats + 24 = ~280 cyc = 1.57 µs/frame → 638 k frames/s
  → **~319 k pts/s** detector capacity. So it **copes with ~300 k pts/s (~6 % margin)**.
- Output bandwidth = 1 word/frame ≈ **~5 MB/s** — trivial for the HP DMA; readout is not
  the bottleneck.
- **The detector is matched to FFT line rate by construction** — it is not the throughput
  limiter. Real limits: (a) **one peak per frame** (max bin per ramp; cannot report
  multi-target-per-ramp); (b) the ~24-cycle restart overhead grows proportionally at
  small N; (c) the 8-wide BEAT unroll closes timing at 178 MHz — **proven** in the
  built `fft178ssr8n11` (whole design closes with the detector included).

### Detector capacity vs parallel-mode frame rate (dual-channel configs)

Per-detector capacity = `f_fft / (N/SSR + 24)` frames/s (one detector per engine). The
crucial point: **frame rate per engine is identical in both modes.** Parallel mode runs
each engine at 1 frame/point but **2× the point rate**; sequential runs each engine at
2 frames/point but **½ the point rate** — both demand `SSR·f_fft/N` frames/s/engine =
the FFT line rate. So parallel mode adds **no new per-detector stress**; it just turns
the detector's frame capacity into a **1:1** point-rate cap (vs ½:1 sequential).

The detector sits a few % under the raw FFT-bound ceiling — the gap is purely the
restart overhead `24·SSR / (N + 24·SSR)`. So the realistic **parallel** point-rate cap
is the detector frame capacity below (slightly under the `SSR·f_fft/N` figures in the
fft_parallel table):

| Config (SSR / FFT clk) | NFFT | cyc/frame (N/SSR+24) | Detector cap = parallel pts/s | Restart derate | Sequential pts/s (½) |
|---|---|---|---|---|---|
| SSR=4 / 125 MHz (ships) | 12 | 1048 | 119.3 k | −2.3 % | 59.6 k |
| SSR=4 / 125 MHz (ships) | 11 | 536 | 233.2 k | −4.5 % | 116.6 k |
| SSR=4 / 178.571 MHz | 12 | 1048 | 170.4 k | −2.3 % | 85.2 k |
| SSR=4 / 178.571 MHz | 11 | 536 | 333.2 k | −4.5 % | 166.6 k |
| SSR=2 / 200 MHz | 13 | 4120 | 48.5 k | −0.6 % | 24.3 k |
| SSR=2 / 200 MHz | 12 | 2072 | 96.5 k | −1.2 % | 48.3 k |
| SSR=2 / 200 MHz | 11 | 1048 | 190.8 k | −2.3 % | 95.4 k |

Takeaways:
- **The detector keeps up in parallel mode** at every dual-channel config — it caps the
  point rate only by the restart derate (≤4.5 %, worst at small N / high SSR). It is the
  binding limit, but barely; it does not collapse the 2× parallel gain.
- The derate **shrinks with larger N** (24·SSR amortised over more bins): N=13 → 0.6 %.
  So heavily-padded large-N parallel configs are essentially detector-transparent.
- Output bandwidth doubles vs sequential (parallel emits **2 words/point** — fft_a up +
  fft_b down — instead of 1 combined word) but is still **~1 word/frame ≈ a few MB/s**,
  trivial for the HP DMA at every rate in the table.
- These are **FFT-bound** numbers. In the acq-bound (full-ramp) regime the frame rate
  drops to 15.3 k / 30.5 k (N12 / N11), where the detector has >5× margin — never a
  factor.

## Reference numbers

- ADC: 125 Msps · 14-bit. FFT clocks: 125 (FFT_CLK_SEL=0), 178.571 (FFT_CLK_178, VCO
  1250/7), 200 (FFT_CLK_200, VCO 1000/5), 250 MHz (default ser).
- "178 MHz" figures above use the nominal 712 Msps intake; the implemented clock is
  178.571 MHz (intake 714.3 Msps), a ~0.3 % difference — negligible for estimates.
- Closing builds: `fft200ssr2` (SSR=2/N13/200), `fft178ssr4n11` (SSR=4/N11/178),
  `fft178ssr8n11` (SSR=8/N11/178, single channel).
