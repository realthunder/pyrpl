#!/bin/bash
# Standalone functional verification for the FFT HLS implementations (csim).
# Kept separate from make.sh on purpose — this only runs C-simulation, no build.
#
# Usage:
#   verify/run.sh <impl> [FFT_SSR]
#     impl 2 : fft_ssr        (Vitis xf::dsp SSR FFT)        — full spectrum
#     impl 3 : fft_ip_ssr_pre (LogiCORE-sub-FFT front end)   — ADC input scaling only
#                              (the FFT itself is an external xfft IP, not in HLS)
#     impl 4 : fft_native_pre + fft_native_mag (native-SSR xfft glue) — input scaling
#                              + magnitude (the xfft IP ordering is checked on the DIF
#                              path by impl 5, which shares fft_proc.sv's DIF branch)
#     impl 5 : fft_hls_direct (direct hls::fft)              — full spectrum
#   e.g.  verify/run.sh 5 4      # IMPL=5, SSR=4
#         verify/run.sh 4 4      # IMPL=4, SSR=4 (pre + mag glue)
#         verify/run.sh 2        # IMPL=2, SSR=2 (default)
#
# Honours XILINX_VERSION (default 2025.2).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)

impl="${1:-5}"
export FFT_SSR="${2:-${FFT_SSR:-}}"

XILINX_VERSION="${XILINX_VERSION:-2025.2}"
export XILINX_VITIS=/opt/Xilinx/${XILINX_VERSION}/Vitis
export XILINX_VIVADO=/opt/Xilinx/${XILINX_VERSION}/Vivado
# shellcheck disable=SC1091
set +u; source "${XILINX_VITIS}/settings64.sh"; set -u
# csim's bundled GCC needs Ubuntu's multiarch paths (same as make.sh)
export LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib32:${LIBRARY_PATH:-}
export CPATH=/usr/include/x86_64-linux-gnu:${CPATH:-}
HLS="${XILINX_VITIS}/bin/loader -exec vitis_hls"

case "$impl" in
    2) tcls=("$HERE/fft_ssr.tcl") ;;
    3) tcls=("$HERE/fft_ip_ssr_pre.tcl") ;;
    4) tcls=("$HERE/fft_ssr_native_pre.tcl" "$HERE/fft_ssr_native_mag.tcl") ;;
    5) tcls=("$HERE/fft_hls_direct.tcl") ;;
    *) echo "unknown impl '$impl' (use 2, 3, 4, or 5)" >&2; exit 2 ;;
esac

echo "==> FFT verify: IMPL=$impl  FFT_SSR=${FFT_SSR:-default}"
cd "$ROOT"
for tcl in "${tcls[@]}"; do
    echo "==> csim: $(basename "$tcl")"
    $HLS -f "$tcl"
done
