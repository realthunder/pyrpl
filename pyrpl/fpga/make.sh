#!/bin/bash
set -euo pipefail

# Script lives in the fpga/ directory; all paths are relative to it.
ROOT=$(cd "$(dirname "$0")" && pwd)

# BUILD_DIR: run Vivado place&route in an isolated working directory (its own
# out/, .Xil, sdk) with the read-only source/cache dirs symlinked back to $ROOT.
# Lets several deterministic (single-threaded) builds run in parallel without
# colliding on out/ or Vivado temp files (see seed_sweep.sh). HLS is shared via
# $ROOT/.hls and must be pre-built — isolated workers skip the HLS step. Default:
# build in place at $ROOT (unchanged behaviour).
WORKROOT="${BUILD_DIR:-$ROOT}"

# ---- Remote build dispatch -------------------------------------------------
# Usage: make.sh remote [-p PATCH] [make.sh args...]
#   rsync the whole git repo to ${REMOTE_HOST:-oplab} at the same absolute path
#   (carrying any local modifications), optionally apply PATCH on top, then run
#   make.sh on the remote with the forwarded args. Use it to try an alternative
#   fix on another machine in parallel with the local build.
if [[ "${1:-}" == "remote" ]]; then
    shift
    REMOTE_HOST="${REMOTE_HOST:-oplab}"

    PATCH=""
    if [[ "${1:-}" == "-p" || "${1:-}" == "--patch" ]]; then
        PATCH="$2"; shift 2
        [[ -f "$PATCH" ]] || { echo "patch file not found: $PATCH" >&2; exit 1; }
    fi

    REPO_ROOT=$(cd "$ROOT" && git rev-parse --show-toplevel)
    REL=${ROOT#"$REPO_ROOT"/}      # fpga subdir, relative to the repo root
    echo "==> Remote build on ${REMOTE_HOST}:${REPO_ROOT}${PATCH:+  (patch: $PATCH)}"

    # Mirror the whole repo to the same absolute path, excluding fpga build
    # output. --delete keeps it an exact copy of the local working tree;
    # excluded dirs on the remote are left untouched (make.sh wipes them anyway).
    ssh "$REMOTE_HOST" "mkdir -p $(printf '%q' "$REPO_ROOT")"
    rsync -az --delete \
        --exclude "/$REL/out/" \
        --exclude "/$REL/out.d/" \
        --exclude "/$REL/.hls/" \
        --exclude "/$REL/.Xil/" \
        --exclude "/$REL/.srcs/" \
        --exclude "/$REL/sdk/" \
        --exclude "/$REL/build.log" \
        "$REPO_ROOT/" "$REMOTE_HOST:$REPO_ROOT/"

    # Optionally apply a patch (read from stdin), then run make.sh on the remote.
    remote_cmd="cd $(printf '%q' "$REPO_ROOT")"
    [[ -n "$PATCH" ]] && remote_cmd+=" && git apply -v"
    remote_cmd+=" && cd $(printf '%q' "$ROOT") && bash make.sh"
    for a in "$@"; do remote_cmd+=" $(printf '%q' "$a")"; done

    if [[ -n "$PATCH" ]]; then
        ssh "$REMOTE_HOST" "$remote_cmd" < "$PATCH"
    else
        ssh "$REMOTE_HOST" "$remote_cmd"
    fi
    exit $?
fi

if [[ "${1:-}" == "clean" ]]; then
    echo "==> Removing HLS stamps: $ROOT/.hls/"
    # Stamps/params are hidden files (.${name}.stamp), so the *.stamp glob alone
    # misses them — include the dotfile patterns or 'clean' won't force a rebuild.
    rm -f "$ROOT/.hls/"*.stamp "$ROOT/.hls/"*.params \
          "$ROOT/.hls/".*.stamp "$ROOT/.hls/".*.params
    shift
fi

# XILINX_VERSION: Vivado/Vitis toolchain version to use.
#   Set in environment to override (e.g. XILINX_VERSION=2020.1 ./make.sh).
#   Directory layout changed between 2020.1 and 2025.2:
#     2020.1: /opt/Xilinx/{Vivado,Vitis}/2020.1/
#     2025.2: /opt/Xilinx/2025.2/{Vivado,Vitis}/
XILINX_VERSION=${XILINX_VERSION:-2025.2}

if [[ "$XILINX_VERSION" == "2020.1" ]]; then
    export XILINX_VIVADO=/opt/Xilinx/Vivado/2020.1
    export XILINX_VITIS=/opt/Xilinx/Vitis/2020.1
    set +u; source ${XILINX_VIVADO}/settings64.sh; set -u
    VIVADO=${XILINX_VIVADO}/bin/vivado
    VIVADO_HLS=${XILINX_VITIS}/bin/vitis_hls
else
    export XILINX_VIVADO=/opt/Xilinx/${XILINX_VERSION}/Vivado
    export XILINX_VITIS=/opt/Xilinx/${XILINX_VERSION}/Vitis
    set +u; source ${XILINX_VIVADO}/settings64.sh; set -u
    VIVADO=${XILINX_VIVADO}/bin/vivado
    # In 2025.2+, vitis_hls wrapper was removed from bin/; go through loader
    # which sources setupEnv.sh/rdiArgs.sh to set RDI_DATADIR, TCL_LIBRARY, etc.
    VIVADO_HLS="${XILINX_VITIS}/bin/loader -exec vitis_hls"

    # 2025.2's rdiArgs.sh hard-codes LC_ALL=en_US.UTF-8 at startup; if that
    # locale isn't installed the C++ runtime aborts with:
    #   locale::facet::_S_create_c_locale name not valid
    # (2020.1 does not have this problem.)
    # Fix: sudo locale-gen en_US.UTF-8  (then re-run make.sh)
    if ! locale -a 2>/dev/null | grep -qi "en_US.utf8\|en_US.UTF-8"; then
        echo "ERROR: locale en_US.UTF-8 is not installed but Xilinx 2025.2 tools require it." >&2
        echo "       Run:  sudo locale-gen en_US.UTF-8" >&2
        echo "       Then re-run make.sh." >&2
        exit 1
    fi
fi

# Build parameters — uncomment and edit to override the defaults in the TCL scripts.
# All values are passed to Vivado and Vitis HLS via environment variables.

# FPGA_PART: Xilinx part number.
#   xc7z020clg400-1  Red Pitaya Z20 (default)
#   xc7z010clg400-1  Red Pitaya Z10
#export FPGA_PART=xc7z020clg400-1

# CLK_DIFF: 1 = differential clock input, 0 = single-ended.
#export CLK_DIFF=1

# CLK_MULT / CLK_ADC_DIV: PLL settings. ADC clock = ref_clk * CLK_MULT / CLK_ADC_DIV.
#   Default (8/8): 125 MHz * 8 / 8 = 125 MHz ADC clock.
#export CLK_MULT=8
#export CLK_ADC_DIV=8

# ADC_SZ: ADC sample width in bits (14 for Z20, 12 for Alinx).
#export ADC_SZ=14

# FFT_SSR: Super sample rate — parallel FFT output channels per AXI-S beat.
#   Must be a power of 2. Determines how many DSP slices are used in fft_ssr.
#export FFT_SSR=2

# FFT_NFFT: log2 of the FFT transform length (e.g. 12 → 4096-point FFT).
#export FFT_NFFT=12

# FFT_WIDTH: FFT output data word width in bits.
#export FFT_WIDTH=28

# FFT_CLK_PERIOD: HLS synthesis target clock period in ns (4.0 = 250 MHz).
#export FFT_CLK_PERIOD=4.0

# FFT_CLK_SEL: which fabric clock the FFT clock mux is pinned to for timing
#   analysis (set_case_analysis on i_scope/fft_clk_sel). This is the "FFT clock
#   period" knob for the placed/routed design:
#     0 = adc_clk, 125 MHz  -> 8 ns FFT clock (default; SSR=4 closes here)
#     1 = fft_clk, 250 MHz  -> 4 ns FFT clock (analyse/close the FFT at 250 MHz)
#export FFT_CLK_SEL=1

# FFT_MULT_LUT (FFT_IMPL=5 only): DSP->LUT trade for the FFT.
#   0 = DSP48 complex multipliers/butterflies (default).
#   1 = implement them in LUT logic (complex_mult_type/butterfly_type=use_luts).
#   Frees DSP48 (relieves the ~98% DSP wall at SSR=4) but uses many more LUTs and
#   lowers per-mult Fmax; on the near-full xc7z020 it tends to become slice/route
#   bound (see project_lut_fft_nofit). Experimental.
#export FFT_MULT_LUT=1

# FFT_IMPL: FFT implementation selector.
#   1 = plain LogiCORE IP (no HLS FFT build needed)
#   2 = Vitis HLS SSR FFT, Decimation-In-Time (DIT) — fft_ssr
#   3 = HLS DIF SSR FFT using LogiCORE sub-FFTs — fft_ip_ssr
#   5 = direct hls::fft (default; SSR=4 @ 125 MHz, see active defaults below)
#export FFT_IMPL=3

# HIST_BLOCK_SIZE: DMA packet size in detection words (excluding the 1-word header).
#   Total UDP payload = (HIST_BLOCK_SIZE + 1) * 8 bytes.
#   Default 183 → 184 * 8 = 1472 B = one standard Ethernet MTU (no fragmentation).
#export HIST_BLOCK_SIZE=183

# ---- Named build profiles -------------------------------------------------
# PROFILE=<name> ./make.sh applies a known-good combination of FFT_*/DETERMINISTIC
# settings as the defaults. Individual vars on the command line still override the
# profile (the active defaults below use ${VAR:-...}, and the profile only seeds
# those, so e.g. PROFILE=fft200 DETERMINISTIC=0 ./make.sh = the fft200 config but
# fast/non-reproducible). Profiles:
#   fft200ssr2 — SSR=2, NFFT=13 (8192-pt), FFT clock retuned to 200 MHz (FFT_CLK_200=1,
#             FFT_CLK_SEL=1), placed with ExtraPostPlacementOpt (DETERMINISTIC=5).
#             Closes timing reproducibly: pll_adc +0.083, pll_ser(FFT@200) +0.101,
#             0 failing endpoints. Archive: out.d/2025.2-ssr2n13-fft200-closed-*.
case "${PROFILE:-}" in
    ""|none) ;;
    fft200ssr2)
        export FFT_IMPL=${FFT_IMPL:-5}
        export FFT_SSR=${FFT_SSR:-2}
        export FFT_NFFT=${FFT_NFFT:-13}
        export FFT_CLK_200=${FFT_CLK_200:-1}
        export FFT_CLK_SEL=${FFT_CLK_SEL:-1}
        # Best of the rep-OFF place×phys_opt matrix (re-swept after the nfft-constant
        # refactor changed the netlist): ExtraNetDelay_low (DET=10) + AggressiveExplore
        # phys_opt + sum1 replication OFF → adc +0.135, ser +0.131, 0 failing.
        # The SSR=2 design is roomy (63% DSP) so the adc sum1 force-replication is not
        # needed here and actually hurt — keep it OFF.
        export DETERMINISTIC=${DETERMINISTIC:-10}
        export PHYS_OPT=${PHYS_OPT:-AggressiveExplore}
        echo "==> PROFILE=fft200ssr2: SSR=2 NFFT=13 FFT@200MHz, place ExtraNetDelay_low (DETERMINISTIC=10), phys_opt AggressiveExplore"
        ;;
    fft178ssr4n11)
        # Highest-throughput closing config: SSR=4 NFFT=11 (2048-pt) FFT @ 178.57 MHz.
        # N12 stays ~30 ps short on adc; dropping to N11 frees enough congestion that
        # SSR=4 closes. Placed EarlyBlockPlacement (DET=6) + Explore phys_opt, rep OFF.
        # Post-route: adc +0.037, ser +0.130, all dac positive. DSP 98%/BRAM 82%/LUT 75%.
        # Tradeoff vs fft200ssr2: 2048-pt = coarser range res but ~1.8x the point rate.
        export FFT_IMPL=${FFT_IMPL:-5}
        export FFT_SSR=${FFT_SSR:-4}
        export FFT_NFFT=${FFT_NFFT:-11}
        export FFT_CLK_178=${FFT_CLK_178:-1}
        export FFT_CLK_SEL=${FFT_CLK_SEL:-1}
        export DETERMINISTIC=${DETERMINISTIC:-6}
        export PHYS_OPT=${PHYS_OPT:-Explore}
        echo "==> PROFILE=fft178ssr4n11: SSR=4 NFFT=11 FFT@178.57MHz, place EarlyBlockPlacement (DETERMINISTIC=6), phys_opt Explore"
        ;;
    fft178ssr8n11)
        # Experimental SSR=8 single-FFT build: only fft_a is synthesised (FFT_SINGLE=1,
        # fft_b omitted) to halve FFT resource use so SSR=8 has a chance to fit. NFFT=11
        # (2048-pt) @ 178.57 MHz, mirroring fft178ssr4n11. Doubles the SSR=4 point rate
        # if it closes; timing is UNPROVEN — adjust DETERMINISTIC/PHYS_OPT as needed.
        export FFT_IMPL=${FFT_IMPL:-5}
        export FFT_SSR=${FFT_SSR:-8}
        export FFT_NFFT=${FFT_NFFT:-11}
        export FFT_SINGLE=${FFT_SINGLE:-1}
        export FFT_CLK_178=${FFT_CLK_178:-1}
        export FFT_CLK_SEL=${FFT_CLK_SEL:-1}
        export DETERMINISTIC=${DETERMINISTIC:-6}
        export PHYS_OPT=${PHYS_OPT:-Explore}
        echo "==> PROFILE=fft178ssr8n11: SSR=8 NFFT=11 FFT@178.57MHz, fft_b disabled (FFT_SINGLE=1), place EarlyBlockPlacement (DETERMINISTIC=6), phys_opt Explore — EXPERIMENTAL"
        ;;
    *)
        echo "ERROR: unknown PROFILE='$PROFILE' (known: fft200ssr2 fft178ssr4n11 fft178ssr8n11)" >&2; exit 1 ;;
esac

# ---- Active build defaults (override on the command line, e.g. FFT_IMPL=5 ./make.sh) ----
# Default build: native-SSR xfft (FFT_IMPL=4), SSR=4. The two closed operating
# points (reproduce with explicit flags, not baked in as defaults here):
#   Build 1 (best): FFT_NFFT=11 FFT_WIDTH=24 FFT_CLK_SEL=1 FFT_CLK_178=1
#                   DSP_FB_PIPELINE=1 DETERMINISTIC=10  -> adc 0.153 / ser 0.035
#   Build 2 (fast): FFT_NFFT=9  FFT_WIDTH=24 FFT_SCALED=2 FFT_CLK_SEL=0
#                   DSP_FB_PIPELINE=1 DETERMINISTIC=2 (ExtraNetDelay_high)
#                   -> adc 0.205, reproducible (see docs/BuildLog.md). FFT is on
#                   adc_clk at 125, so ser is non-binding here.
#                   Without the FB pipeline / non-det Default placer this point
#                   was adc 0.166 (not run-to-run reproducible).
# (Was FFT_IMPL=5 direct hls::fft; see HANDOFF.md / memory project-hls-direct-fft.)
export FFT_IMPL=${FFT_IMPL:-4}
export FFT_SSR=${FFT_SSR:-4}
# FFT clock for timing closure: 0 = 125 MHz / 8 ns (default), 1 = the I1 mux clock
# (250 MHz / 4 ns, or 200 MHz / 5 ns when FFT_CLK_200=1 — see below).
# Override on the command line:  FFT_CLK_SEL=1 ./make.sh
export FFT_CLK_SEL=${FFT_CLK_SEL:-0}

# FFT_CLK_200: retune the PLL CLKOUT4 (clk_ser, which currently feeds only the FFT
# clock mux) from 250 MHz to 200 MHz, giving the FFT a 5 ns budget instead of 4 ns.
# Affects RTL synthesis (red_pitaya_pll.sv `define) + the pll_ser_clk timing
# constraint (red_pitaya_vivado.tcl). Pair with FFT_CLK_SEL=1 to actually run the
# FFT at 200 MHz, e.g.:  FFT_CLK_200=1 FFT_CLK_SEL=1 ./make.sh
# (Optional: also FFT_CLK_PERIOD=5.0 to let HLS re-synth the FFT IP for 5 ns.)
# Default 0 = unchanged 250 MHz ser clock.
export FFT_CLK_200=${FFT_CLK_200:-0}

# FFT_CLK_178: retune the PLL VCO from 1000 to 1250 MHz (CLKFBOUT_MULT 8->10) and
# scale every output divider x1.25, so adc/dac/pwm clocks stay byte-identical while
# CLKOUT4 (the FFT clock mux input) becomes VCO/7 = 178.57 MHz (5.6 ns budget).
# 178.57 MHz is the only integer-divider FFT clock strictly inside 170-200 MHz that
# keeps adc/dac unchanged (the shared 1000 MHz VCO only offers 200 or 166.67).
# Affects red_pitaya_pll.sv (`define) + the pll_ser_clk constraint (red_pitaya_vivado.tcl).
# Pair with FFT_CLK_SEL=1 to actually run the FFT at 178.57 MHz, e.g.:
#   FFT_CLK_178=1 FFT_CLK_SEL=1 ./make.sh
# Takes precedence over FFT_CLK_200; do not set both.  Default 0 = unchanged 250 MHz.
export FFT_CLK_178=${FFT_CLK_178:-0}

# The two FFT-clock retunes are mutually exclusive (both drive the same CLKOUT4 /
# pll_ser_clk). Setting both would silently use 178 (RTL `ifdef precedence) — fail
# loudly instead so the intended clock is never ambiguous.
if [[ "$FFT_CLK_178" != "0" && "$FFT_CLK_200" != "0" ]]; then
    echo "ERROR: FFT_CLK_178 and FFT_CLK_200 are mutually exclusive — set only one." >&2
    exit 1
fi

# DETERMINISTIC selects a place_design directive (see red_pitaya_vivado.tcl;
# Vivado has no place_design -seed, so directives are the placement-variation lever):
#   0 (default) = fast 8-thread P&R, Default directive, NOT run-to-run reproducible
#   >=1         = single-threaded P&R, value mapped to a directive → bit-identical bitstream
# e.g. DETERMINISTIC=1 ./make.sh   (or sweep DETERMINISTIC=1..N for the best WNS).
export DETERMINISTIC=${DETERMINISTIC:-0}

# ---- Resolved build settings banner ---------------------------------------
# Print the EFFECTIVE configuration after profile + command-line overrides
# resolve, so every build self-documents the actual values handed to Vivado.
# Unlike the per-profile echo above (static text, wrong when a profile var is
# overridden), this is derived from the live variables. For any var a non-profile
# build leaves unset, mirror the downstream default (red_pitaya_vivado.tcl).
_ssr=${FFT_SSR:-4}
if [[ "$FFT_CLK_SEL" == "0" ]]; then
    _fftclk="125MHz (SEL=0, pinned to adc_clk)"
elif [[ "$FFT_CLK_178" != "0" ]]; then
    _fftclk="178.57MHz (SEL=1, CLK_178)"
elif [[ "$FFT_CLK_200" != "0" ]]; then
    _fftclk="200MHz (SEL=1, CLK_200)"
else
    _fftclk="250MHz (SEL=1)"
fi
if [[ $_ssr -ge 8 ]]; then _single_def=1; else _single_def=0; fi
echo "==> Build config: IMPL=${FFT_IMPL} SSR=${_ssr} NFFT=${FFT_NFFT:-12} SCALED=${FFT_SCALED:-2} WIDTH=${FFT_WIDTH:-auto} SINGLE=${FFT_SINGLE:-$_single_def}"
echo "                  FFT_CLK=${_fftclk} | DET=${DETERMINISTIC} PHYS_OPT=${PHYS_OPT:-AggressiveExplore} DSP_FB_PIPELINE=${DSP_FB_PIPELINE:-0}"
unset _ssr _fftclk _single_def

mkdir -p "$ROOT/.hls"

# Vivado HLS csim uses a bundled GCC 6.2.0 that doesn't know Ubuntu 24.04's
# multiarch layout. Expose both libraries and headers explicitly so GCC finds them.
export LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib32:${LIBRARY_PATH:-}
export CPATH=/usr/include/x86_64-linux-gnu:${CPATH:-}

# Fingerprint of HLS-relevant build parameters.
# Stored alongside each stamp so that changing FFT_SSR (or any other param)
# invalidates the cached output even when source files haven't changed.
hls_fingerprint() {
    echo "FPGA_PART=${FPGA_PART:-} FFT_IMPL=${FFT_IMPL:-} FFT_SSR=${FFT_SSR:-} FFT_NFFT=${FFT_NFFT:-} FFT_WIDTH=${FFT_WIDTH:-} PEAK_FRAC=${PEAK_FRAC:-} FFT_SCALED=${FFT_SCALED:-} FFT_INTERNAL_W=${FFT_INTERNAL_W:-} FFT_CLK_PERIOD=${FFT_CLK_PERIOD:-} FFT_USE_APPROX=${FFT_USE_APPROX:-} FFT_UNSCALED=${FFT_UNSCALED:-} FFT_CORDIC_ITER=${FFT_CORDIC_ITER:-} FFT_MULT_LUT=${FFT_MULT_LUT:-} FFT_RUNTIME_NFFT=${FFT_RUNTIME_NFFT:-} HIST_BLOCK_SIZE=${HIST_BLOCK_SIZE:-}"
}

fmt_elapsed() {
    local t=$1
    if [[ $t -ge 60 ]]; then printf '%dm %02ds' $((t/60)) $((t%60))
    else printf '%ds' $t; fi
}

needs_rebuild() {
    local stamp="$1"; shift
    local params_file="${stamp%.stamp}.params"

    # Missing stamp → always rebuild.
    [[ ! -f "$stamp" ]] && return 0

    # Params changed since last build → rebuild.
    local current
    current=$(hls_fingerprint)
    if [[ ! -f "$params_file" ]] || [[ "$(cat "$params_file")" != "$current" ]]; then
        return 0
    fi

    # Any source file newer than stamp → rebuild.
    for f in "$@"; do
        [[ "$f" -nt "$stamp" ]] && return 0
    done

    return 1
}

run_hls() {
    local name="$1"
    local tcl="$2"
    local stamp="$ROOT/.hls/.${name}.stamp"
    local params_file="$ROOT/.hls/.${name}.params"
    shift 2   # remaining args are source files to watch

    if needs_rebuild "$stamp" "$@"; then
        echo "==> HLS: $name rebuilding..."
        local t0=$SECONDS
        (cd "$ROOT" && $VIVADO_HLS -f "$tcl")
        hls_fingerprint > "$params_file"
        touch "$stamp"
        echo "==> HLS: $name done in $(fmt_elapsed $((SECONDS - t0)))."
    else
        echo "==> HLS: $name up to date, skipping."
    fi
}

check_hls() {
    local impl=${FFT_IMPL:-3}

    if [[ "$impl" == "2" ]]; then
        run_hls fft_ssr hls/fft_ssr.tcl \
            hls/fft_ssr.cpp hls/fft_ssr.tcl
    elif [[ "$impl" == "3" ]]; then
        # pre generates the twiddle LUT header; post compiles against it.
        run_hls fft_ip_ssr_pre hls/fft_ip_ssr_pre.tcl \
            hls/fft_ip_ssr.cpp hls/fft_ip_ssr_pre.tcl \
            hls/gen_twiddle_lut.cpp hls/gen_twiddle_lut.py
        run_hls fft_ip_ssr_post hls/fft_ip_ssr_post.tcl \
            hls/fft_ip_ssr.cpp hls/fft_ip_ssr_post.tcl
    elif [[ "$impl" == "4" ]]; then
        # Native-SSR xfft (Vivado 2025.2 CONFIG.super_sample_rates): thin pre + mag.
        run_hls fft_native_pre hls/fft_ssr_native_pre.tcl \
            hls/fft_ssr_native.cpp hls/fft_ssr_native_pre.tcl
        run_hls fft_native_mag hls/fft_ssr_native_mag.tcl \
            hls/fft_ssr_native.cpp hls/fft_ssr_native_mag.tcl
    elif [[ "$impl" == "5" ]]; then
        # Direct hls::fft instantiation (hls_fft.h): one HLS IP, no BD, no separate post.
        # Uses the shared twiddle ROM (generated by the .tcl via gen_twiddle_lut).
        run_hls fft_hls_direct hls/fft_hls_direct.tcl \
            hls/fft_hls_direct.cpp hls/fft_hls_direct.tcl \
            hls/gen_twiddle_lut.cpp hls/gen_twiddle_lut.py
    fi
    # FFT_IMPL==1 (plain LogiCORE) needs no HLS FFT build.

    run_hls peak_detector hls/peak_detector.tcl \
        hls/peak_detector.cpp hls/peak_detector.h \
        hls/peak_detector_tb.cpp hls/peak_detector.tcl
}

BUILD_START=$SECONDS

if [[ "${1:-}" == "hls" ]]; then
    check_hls
    echo "==> HLS build complete in $(fmt_elapsed $((SECONDS - BUILD_START)))."
    exit 0
fi

# Isolated build dir: symlink shared read-only sources/cache; keep out/.Xil/sdk local.
if [[ "$WORKROOT" != "$ROOT" ]]; then
    mkdir -p "$WORKROOT"
    for d in rtl ip sdc elements hls .hls Vitis_Libraries; do
        [[ -e "$ROOT/$d" ]] && ln -sfn "$ROOT/$d" "$WORKROOT/$d"
    done
fi

# Archive the previous build output before wiping (keep at most 10 snapshots).
if [[ -d "$WORKROOT/out" ]]; then
    mkdir -p "$WORKROOT/out.d"
    mv "$WORKROOT/out" "$WORKROOT/out.d/$(date +%Y%m%d-%H%M%S)"
    ls -1dt "$WORKROOT/out.d"/[0-9]*-[0-9]* 2>/dev/null | tail -n +11 | xargs rm -rf
fi
rm -rf "$WORKROOT/.Xil" "$WORKROOT/.srcs" "$WORKROOT/.gen" "$WORKROOT/sdk"

# HLS is shared via $ROOT/.hls. Build it only for in-place builds; isolated parallel
# workers assume it is already built (pre-warm with `./make.sh hls`) so concurrent
# workers never race on the shared HLS cache.
if [[ "$WORKROOT" == "$ROOT" ]]; then
    check_hls
fi

script="${1:-red_pitaya_vivado.tcl}"
[[ $# -gt 0 ]] && shift
# Resolve to absolute so it can be sourced from an isolated WORKROOT cwd (its
# internal relative paths rtl/ ip/ sdc/ out/ then resolve against the symlinks).
[[ "$script" != /* ]] && script="$ROOT/$script"

# ---- Build provenance manifest ---------------------------------------------
# Written into out/ (which red_pitaya_vivado.tcl also writes to) so every build
# — and every out.d/ archive — is self-describing and reproducible from the
# recorded git commit + params + seed.
if [[ "$DETERMINISTIC" =~ ^[1-9][0-9]*$ ]]; then
    det_dirs=(Explore ExtraNetDelay_high AltSpreadLogic_high WLDrivenBlockPlacement ExtraPostPlacementOpt EarlyBlockPlacement AltSpreadLogic_medium Default ExtraTimingOpt ExtraNetDelay_low AltSpreadLogic_low)
    det_dir="${det_dirs[$(( (DETERMINISTIC-1) % ${#det_dirs[@]} ))]}"
    seed_str="$DETERMINISTIC (deterministic: maxThreads 1, place_design -directive $det_dir)"
else
    seed_str="default(1) — NOT reproducible (multithreaded, maxThreads 8)"
fi
git_dirty=$(git -C "$ROOT" status --porcelain --untracked-files=no 2>/dev/null)
mkdir -p "$WORKROOT/out"
manifest="$WORKROOT/out/BUILD_INFO.txt"
{
    echo "build_date    = $(date -Is)"
    echo "host          = $(hostname)"
    echo "git_branch    = $(git -C "$ROOT" rev-parse --abbrev-ref HEAD 2>/dev/null)"
    echo "git_commit    = $(git -C "$ROOT" rev-parse HEAD 2>/dev/null)"
    echo "git_dirty     = $([[ -n "$git_dirty" ]] && echo yes || echo no)"
    [[ -n "$git_dirty" ]] && { echo "git_dirty_tracked:"; echo "$git_dirty" | sed 's/^/    /'; }
    echo "xilinx_version= $XILINX_VERSION"
    echo "placer_seed   = $seed_str"
    echo "# Build params explicitly set via env (empty => default; resolved values"
    echo "# and derived module parameters are appended below by red_pitaya_vivado.tcl):"
    for v in FPGA_PART ADC_SZ CLK_MULT CLK_ADC_DIV FFT_IMPL FFT_SSR FFT_NFFT \
             FFT_WIDTH PEAK_FRAC FFT_SCALED FFT_INTERNAL_W FFT_CLK_PERIOD FFT_CLK_SEL FFT_CLK_200 FFT_CLK_178 \
             FFT_MULT_LUT FFT_USE_APPROX FFT_UNSCALED FFT_CORDIC_ITER HIST_BLOCK_SIZE PHYS_OPT OPT_DIRECTIVE FFT_SINGLE; do
        printf '%-14s= %s\n' "$v" "${!v:-}"
    done
} > "$manifest"

vivado_start=$SECONDS
(cd "$WORKROOT" && $VIVADO -nolog -nojournal -mode tcl -source "$script" -tclargs "$@")
echo "==> Vivado done in $(fmt_elapsed $((SECONDS - vivado_start)))."

# Append the post-route WNS so the manifest captures the build's actual result.
if [[ -f "$WORKROOT/out/post_route_timing_summary.rpt" ]]; then
    {
        echo ""
        echo "# post-route intra-clock WNS (ns):"
        grep -E "^  (pll_adc_clk|pll_ser_clk|pll_dac_clk_1x) " \
            "$WORKROOT/out/post_route_timing_summary.rpt" | awk '{printf "    %-16s%s\n", $1, $2}'
    } >> "$manifest"
fi

echo "==> Build complete in $(fmt_elapsed $((SECONDS - BUILD_START)))."
