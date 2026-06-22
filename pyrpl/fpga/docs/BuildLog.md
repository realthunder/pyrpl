# FPGA Build Configuration Log

Record of FFT/timing configurations tried. Part `xc7z020-clg400-1`, Vivado 2025.2.
Device totals: **53,200 LUT · 106,400 FF · 140 BRAM tiles · 220 DSP48**.
WNS in ns (post-route, intra-clock). All FFT builds are `FFT_IMPL=5` (direct
`hls::fft`), per-channel (fft_a + fft_b). `pll_dac` clocks pass in every build
(≈ +0.65 @125 / +1.845 @250) and are omitted.

---

## SSR=2 · NFFT=13 · FFT @ 200 MHz  ✅ CLOSES

Settings: `FFT_SSR=2 FFT_NFFT=13 FFT_CLK_200=1 FFT_CLK_SEL=1` (runtime-nfft off).
Resources: **LUT 57% · FF 34% · BRAM 86% · DSP 63%** (roomy). csim PASS (II=1).
Build: `PROFILE=fft200ssr2 ./make.sh` → DET=6 + AggressiveExplore + **rep OFF**.

Full rep-OFF place × phys_opt matrix, **re-swept after the nfft-constant refactor**
(commit 091512d6 changed the netlist; the prior winner EarlyBlockPlacement regressed
to −0.154, so DET was re-picked) — **6/21 close**, worst-slack:

| place \ phys_opt | AggressiveExplore | Explore | AggressiveFanoutOpt |
|---|---|---|---|
| Explore | −0.113 | −0.113 | −0.138 |
| ExtraNetDelay_high | −0.029 | −0.031 | −0.037 |
| **ExtraNetDelay_low** | **+0.131** ✅ | +0.006 ✅ | −0.105 |
| AltSpreadLogic_medium | +0.086 ✅ | +0.121 ✅ | +0.091 ✅ |
| WLDrivenBlockPlacement | +0.103 ✅ | −0.037 | −0.035 |
| EarlyBlockPlacement | −0.154 | −0.143 | −0.156 |
| ExtraPostPlacementOpt | −0.125 | −0.061 | −0.039 |

**Winner: ExtraNetDelay_low × AggressiveExplore, rep OFF = adc +0.135, ser +0.131**
(0 failing). `PROFILE=fft200ssr2` → DET=10. Archive:
`out.d/2025.2-ssr2n13-fft200-closed-adc0.135-ser0.131`.

History: the original point-build was rep-ON/ExtraPostPlacementOpt (+0.083). The first
full rep-OFF matrix (pre-refactor netlist) closed 12/21, best EarlyBlockPlacement +0.156.
After the nfft-constant refactor the netlist shifted (EarlyBlockPlacement → −0.154, an
FFT-internal path, not an nfft defect), so re-swept to the DET=10 winner above. **rep OFF
remains best** (63% DSP is roomy; the adc sum1 force-replication is unneeded here).

---

## SSR=4 · NFFT=11 · FFT @ 178.571 MHz  ✅ CLOSES (highest-throughput closing config)

Settings: `FFT_SSR=4 FFT_NFFT=11 FFT_CLK_178=1 FFT_CLK_SEL=1 SUM1_REPLICATE=0`.
Resources: **LUT 75% · FF 47% · BRAM 82% · DSP 98%** (215/220). csim PASS.
Build: `PROFILE=fft178ssr4n11 ./make.sh` → DET=6 (EarlyBlockPlacement) + Explore phys_opt.
Dropping N12→N11 halves the FFT data mem / reorder buffer, freeing the congestion
that held N12 at −0.028. Full 7×3 place×phys_opt matrix (rep OFF) — **6/21 close**:

| place \ phys_opt | AggressiveExplore | Explore | AggressiveFanoutOpt |
|---|---|---|---|
| Explore | −0.046 | +0.029 ✅ | −0.265 |
| ExtraNetDelay_high | −0.086 | −0.080 | −0.140 |
| ExtraNetDelay_low | −0.187 | +0.002 ✅ | −0.018 |
| AltSpreadLogic_medium | −0.151 | −0.031 | −0.236 |
| WLDrivenBlockPlacement | −0.013 | −0.061 | −0.109 |
| **EarlyBlockPlacement** | +0.018 ✅ | **+0.037** ✅ | +0.029 ✅ |
| ExtraPostPlacementOpt | −0.183 | −0.261 | −0.182 |

**Winner: EarlyBlockPlacement × Explore = +0.037** (adc +0.037, ser +0.130, all dac +).
The whole EarlyBlockPlacement row closes (robust). Archive:
`out.d/2025.2-ssr4n11-fft178-closed-adc0.037-ser0.130`. Throughput: SSR=4/178 = 712 Msps
intake → ~156 k pts/s (N11, −10%), ~1.8× the fft200ssr2 build; tradeoff = 2048-pt
(coarser range res). Note N12-best cells did NOT transfer to N11 (different floorplan
→ different directive ranking); the full matrix was needed.

---

## SSR=8 · NFFT=11 · FFT @ 178.571 MHz · SINGLE  ✅ CLOSES (highest throughput)

**Single-channel** build (the only non-`fft_a+fft_b` config here): `FFT_SINGLE=1`
disables `fft_b` so one SSR=8 FFT fits where two SSR=4 channels did. SSR=8 needed a
new radix-8 DIF path in `fft_hls_direct.cpp` (3 radix-2 stages + 8 sub-FFTs); IMPL=5
previously did 1/2/4 only. Settings: `FFT_SSR=8 FFT_NFFT=11 FFT_SINGLE=1 FFT_CLK_178=1
FFT_CLK_SEL=1 SUM1_REPLICATE=0`. csim PASS (tones at N=4096/1024/64).
Build: `PROFILE=fft178ssr8n11 ./make.sh` → DET=6 (EarlyBlockPlacement) + Explore phys_opt.

Resources (whole design): **LUT 68% (36381) · FF 44% · BRAM ~61% · DSP 77% (169/220)**.
NB the HLS per-IP csynth LUT *estimate* was 120% — Vivado mapping brought it to 68%;
trust post-route, not the HLS estimate.

| FFT clock | adc WNS | ser WNS | hold | closes | archive (`out.d/`) |
|---|---|---|---|---|---|
| **178.571 MHz** | **+0.097** | **+0.123** | + | ✓ **best** | **fft178ssr8n11-closed** |
| 200 MHz | −0.114 (1 ep) | −0.229 (408 ep) | + | ✗ | (not archived) |

 ~2× the SSR=4/178 point rate (8 samples/clk @ 178 = 1.43 Gsps intake). 200 MHz fails
broadly on the FFT datapath (ser −0.229, 408 eps; HLS est. 5.091 ns > 5.0) — would need
RTL pipelining of the butterfly stages. 178 is the sweet spot.

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

sum1 force-replication A/B (DET=6 + Explore): ON adc −0.099 / OFF adc −0.177 — the
replication **helps** adc even on the full die (`SUM1_REPLICATE` knob; keep it on).

place × phys_opt matrix (7 place × 3 phys_opt, **SUM1_REPLICATE=0**), worst-slack:

| place \ phys_opt | AggressiveExplore | Explore | AggressiveFanoutOpt |
|---|---|---|---|
| Explore | −0.174 | −0.163 | −0.314 |
| ExtraNetDelay_high | −0.288 | −0.125 | −0.156 |
| ExtraNetDelay_low | −0.105 | −0.265 | −0.379 |
| **AltSpreadLogic_medium** | −0.207 | −0.049 | **−0.028** ⭐ |
| WLDrivenBlockPlacement | −0.196 | −0.452 | −0.433 |
| EarlyBlockPlacement | −0.246 | −0.177 | −0.232 |
| ExtraPostPlacementOpt | −0.180 (adc −0.041) | −0.286 | −0.194 |

Best = **AltSpreadLogic_medium × AggressiveFanoutOpt = −0.028** (adc −0.028, ser +0.012),
rep OFF — the *medium* spread relieves adc congestion without starving the FFT (where
`_high` is worst). 28 ps short.

Closing attempt — top adc-bound cells retried with **rep ON** (rep helps on
EarlyBlockPlacement, so worth a shot): all *regressed*. AltSpreadLogic_medium ×
AggressiveFanoutOpt −0.028→−0.161; × Explore −0.049→−0.454; ExtraNetDelay_low ×
AggressiveExplore −0.105→−0.329; ExtraNetDelay_high × Explore −0.125→−0.216.
⇒ replication's effect is **placement-dependent**: helps EarlyBlockPlacement,
hurts the AltSpreadLogic placements (spread directive + replicated copies =
over-spread on the full die).

**Exhaustive coverage (both replication states, 8 places × 3 phys_opt ≈ 48 builds):**
- **rep OFF** best = AltSpreadLogic_medium × AggressiveFanoutOpt = **−0.028**.
- **rep ON** full 7×3 matrix best = EarlyBlockPlacement × Explore = **−0.099** (worse than
  rep OFF in every cell). Replication is **placement-dependent**: helps EarlyBlockPlacement,
  hurts spread/net-delay placements.
- AltSpreadLogic_low (gentlest spread): rep OFF −0.282…−0.360, rep ON −0.086…−0.890 —
  no better than `_medium`. So `_medium` is the spread sweet spot.

**Verdict:** best achievable = AltSpreadLogic_medium + AggressiveFanoutOpt + **rep
OFF** = **worst −0.028** (adc). Does not close in *any* of ~48 combos. adc is genuinely
congestion-bound (98% DSP, no stray constraint — audited). At **N12** SSR=4 stays
~30 ps short regardless of placement / phys_opt / replication — **but at N11 it CLOSES**
(see the SSR=4·N11 section above). So SSR=4/178 is viable only at NFFT≤11 on this device.
Archives: `out.d/sweep-ssr4n12-178-det*`.

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
