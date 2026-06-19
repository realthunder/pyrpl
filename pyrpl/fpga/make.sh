#!/bin/bash
set -euo pipefail

# Script lives in the fpga/ directory; all paths are relative to it.
ROOT=$(cd "$(dirname "$0")" && pwd)

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

# FFT_IMPL: FFT implementation selector.
#   1 = plain LogiCORE IP (no HLS FFT build needed)
#   2 = Vitis HLS SSR FFT, Decimation-In-Time (DIT) — fft_ssr
#   3 = HLS DIF SSR FFT using LogiCORE sub-FFTs — fft_ip_ssr (default)
#export FFT_IMPL=3

# HIST_BLOCK_SIZE: DMA packet size in detection words (excluding the 1-word header).
#   Total UDP payload = (HIST_BLOCK_SIZE + 1) * 8 bytes.
#   Default 183 → 184 * 8 = 1472 B = one standard Ethernet MTU (no fragmentation).
#export HIST_BLOCK_SIZE=183

mkdir -p "$ROOT/.hls"

# Vivado HLS csim uses a bundled GCC 6.2.0 that doesn't know Ubuntu 24.04's
# multiarch layout. Expose both libraries and headers explicitly so GCC finds them.
export LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib32:${LIBRARY_PATH:-}
export CPATH=/usr/include/x86_64-linux-gnu:${CPATH:-}

# Fingerprint of HLS-relevant build parameters.
# Stored alongside each stamp so that changing FFT_SSR (or any other param)
# invalidates the cached output even when source files haven't changed.
hls_fingerprint() {
    echo "FPGA_PART=${FPGA_PART:-} FFT_IMPL=${FFT_IMPL:-} FFT_SSR=${FFT_SSR:-} FFT_NFFT=${FFT_NFFT:-} FFT_WIDTH=${FFT_WIDTH:-} FFT_SCALED=${FFT_SCALED:-} FFT_CLK_PERIOD=${FFT_CLK_PERIOD:-} FFT_USE_APPROX=${FFT_USE_APPROX:-} HIST_BLOCK_SIZE=${HIST_BLOCK_SIZE:-}"
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
        (cd "$ROOT" && $VIVADO_HLS -f "$tcl")
        hls_fingerprint > "$params_file"
        touch "$stamp"
        echo "==> HLS: $name done."
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

if [[ "${1:-}" == "hls" ]]; then
    check_hls
    exit 0
fi

# Archive the previous build output before wiping (keep at most 10 snapshots).
if [[ -d "$ROOT/out" ]]; then
    mkdir -p "$ROOT/out.d"
    mv "$ROOT/out" "$ROOT/out.d/$(date +%Y%m%d-%H%M%S)"
    ls -1dt "$ROOT/out.d"/[0-9]*-[0-9]* 2>/dev/null | tail -n +11 | xargs rm -rf
fi
rm -rf "$ROOT/.Xil" "$ROOT/.srcs" "$ROOT/.gen" "$ROOT/sdk"

check_hls

script="${1:-red_pitaya_vivado.tcl}"
[[ $# -gt 0 ]] && shift

(cd "$ROOT" && $VIVADO -nolog -nojournal -mode tcl -source "$script" -tclargs "$@")

echo "compilation finished"
