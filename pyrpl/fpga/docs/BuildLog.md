# FPGA Build Configuration Log

Record of FFT/timing configurations tried. Part `xc7z020-clg400-1`, Vivado 2025.2.
Device totals: **53,200 LUT · 106,400 FF · 140 BRAM tiles · 220 DSP48**.
WNS in ns (post-route, intra-clock). All FFT builds are `FFT_IMPL=5` (direct
`hls::fft`), per-channel (fft_a + fft_b). `pll_dac` clocks pass in every build
(≈ +0.65 @125 / +1.845 @250) and are omitted.

---

## SSR=2 · NFFT=13 · FFT @ 200 MHz  ✅ CLOSES

Settings: `FFT_SSR=2 FFT_NFFT=13 FFT_CLK_200=1 FFT_CLK_SEL=1` (runtime-nfft off).
Resources (placement-only variation): **LUT ~30.5k (57%) · FF ~35.8k (34%) · BRAM 120 (86%) · DSP 139 (63%)**.
csim PASS (II=1) for all. Build: `PROFILE=fft200ssr2 ./make.sh` → DET=5.

Deterministic placement-directive sweep (`DETERMINISTIC=n`, single-threaded, reproducible):

| DET | place_design directive | adc WNS | ser WNS | worst | closes | archive (`out.d/`) |
|---|---|---|---|---|---|---|
| 1 | Explore | +0.178 | +0.048 | +0.048 | ✓ | sweep-ssr2n13-200-det1-Explore |
| 2 | ExtraNetDelay_high | −0.154 | +0.039 | −0.154 | ✗ | sweep-ssr2n13-200-det2-ExtraNetDelay_high |
| 3 | AltSpreadLogic_high | +0.050 | +0.179 | +0.050 | ✓ | sweep-ssr2n13-200-det3-AltSpreadLogic_high |
| 4 | WLDrivenBlockPlacement | +0.085 | +0.039 | +0.039 | ✓ | sweep-ssr2n13-200-det4-WLDrivenBlockPlacement |
| **5** | **ExtraPostPlacementOpt** | **+0.083** | **+0.101** | **+0.083** | ✓ **best** | **2025.2-ssr2n13-fft200-closed-adc0.083-ser0.101** |
| 6 | EarlyBlockPlacement | +0.014 | +0.142 | +0.014 | ✓ | sweep-ssr2n13-200-det6-EarlyBlockPlacement |

---

## SSR=4 · NFFT=12 · FFT @ 178.571 MHz  ❌ does NOT close (adc congestion-bound)

Settings: `FFT_SSR=4 FFT_NFFT=12 FFT_CLK_178=1 FFT_CLK_SEL=1`.
Resources: **LUT 80% · FF 50% · BRAM 91% · DSP 98%** (near full). csim PASS.
The **FFT (ser, 5.6 ns) closes** — 178 MHz relaxed the SSR=4 reorder enough (ser
positive on most directives). **adc is the wall**: `i_dsp/sum1` loopback,
congestion-bound at 98% DSP. No place/phys_opt combo found closes it.

Placement-directive sweep (phys_opt = AggressiveExplore default) — **0/6 close**:

| DET | place directive | adc | ser | worst |
|---|---|---|---|---|
| 1 | Explore | −0.344 | +0.013 | −0.344 |
| 2 | ExtraNetDelay_high | −0.271 | −0.065 | −0.271 |
| 3 | AltSpreadLogic_high | −0.796 | +0.074 | −0.796 |
| 4 | WLDrivenBlockPlacement | −0.273 | −0.028 | −0.273 |
| 5 | ExtraPostPlacementOpt | −0.174 | −0.191 | −0.191 |
| 6 | EarlyBlockPlacement | **−0.140** | +0.072 | **−0.140** (best of sweep) |

phys_opt sweep on the best place (DET=6 EarlyBlockPlacement), via `PHYS_OPT=`:

| phys_opt directive | adc | ser | worst |
|---|---|---|---|
| **Explore** | **−0.099** | +0.002 | **−0.099** ← best overall |
| AggressiveExplore (default) | −0.140 | +0.072 | −0.140 |
| none (skipped) | −0.306 | −0.158 | −0.306 |
| AggressiveFanoutOpt | −0.319 | −0.022 | −0.319 |
| AlternateReplication | −0.319 | −0.022 | −0.319 |

place=`ExtraTimingOpt` (DET=9): + Explore −0.579, + AggressiveFanoutOpt −0.568 — worse.

**Verdict:** best achievable = EarlyBlockPlacement + `PHYS_OPT=Explore` = **worst −0.099**
(adc). Does not close. adc is genuinely congestion-bound (98% DSP, no stray
constraint — audited). Softer phys_opt (Explore) beats AggressiveExplore by +0.04,
but fanout/replication/timing-opt variants all *regress* adc: on the full die the
extra restructuring displaces the sum1 consumers. **SSR=2 is the viable fast-FFT path;
SSR=4 won't close on this device regardless of FFT clock.** Archives: `out.d/sweep-ssr4n12-178-det*`.

---

## Other attempts (not closed / rejected)

| Config | place | adc WNS | ser WNS | Result |
|---|---|---|---|---|
| SSR=2 · N13 · 200 MHz | multithread (Default) | −0.047 | −0.114 | near miss; closed later via directive sweep |
| SSR=2 · N13 · 200 MHz · **runtime-nfft ON** | multithread | −0.024 | **−0.419** | ser broken: variable reorder-loop bounds defeat const-folding (700 endpoints) |
| SSR=4 · N12 · 200 MHz | multithread | −0.157 | −0.281 | not closed; SSR=4 reorder needs ~5.4 ns |

---

## Shipping default (reference)

| Config | place | adc WNS | ser WNS | LUT | FF | BRAM | DSP |
|---|---|---|---|---|---|---|---|
| SSR=4 · N12 · 125 MHz (`FFT_CLK_SEL=0`) | multithread | ~−0.08…−0.002 | +1.845 | 78% | 50% | 91% | 98% |

Built with a plain `./make.sh`. Archive: `out.d/2025.2-default-ssr4-125mhz-adc0.002`.
