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

## SSR=8 estimate (single channel, NFFT=11) — throughput only, NOT proven buildable

Intake = SSR·f_fft = 8·f_fft. FFT-bound pts/s = `8·f_fft/(2N)` = `f_fft/512` (N=2048).

| FFT clock | Intake | FFT-bound pts/s | −10% | Short threshold | Acq-bound (full ramp) |
|---|---|---|---|---|---|
| 125 MHz | 1000 Msps | 244.1 k | 219.7 k | 12.5 % | 30.5 k |
| 178.571 MHz | 1429 Msps | 348.8 k | 313.9 k | 8.75 % | 30.5 k |
| 200 MHz | 1600 Msps | 390.6 k | 351.6 k | 7.81 % | 30.5 k |

**Caveats — estimate only:** (1) **fit is marginal** — SSR=8 single-channel ≈ 98 % DSP
(DSP scales with SSR, not N; N11 only lowers BRAM), zero headroom, LUT/congestion the
wildcard. (2) **Likely won't close** — SSR=4/N12 didn't; SSR=8 doubles FFT congestion.
(3) Needs **RTL change** to instantiate only fft_a. (4) Per-frame readout / detector
capacity becomes the gate (see below).

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
  small N; (c) the 8-wide BEAT unroll must close timing at the target clock (proven at
  SSR=4/178 in `fft178ssr4n11`; unproven at SSR=8).

## Reference numbers

- ADC: 125 Msps · 14-bit. FFT clocks: 125 (FFT_CLK_SEL=0), 178.571 (FFT_CLK_178, VCO
  1250/7), 200 (FFT_CLK_200, VCO 1000/5), 250 MHz (default ser).
- "178 MHz" figures above use the nominal 712 Msps intake; the implemented clock is
  178.571 MHz (intake 714.3 Msps), a ~0.3 % difference — negligible for estimates.
- Closing builds: `fft200ssr2` (SSR=2/N13/200), `fft178ssr4n11` (SSR=4/N11/178).
