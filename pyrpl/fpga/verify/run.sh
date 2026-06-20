#!/bin/bash
# Standalone functional verification for the FFT HLS implementations (csim).
# Kept separate from make.sh on purpose — this only runs C-simulation, no build.
#
# Usage:
#   verify/run.sh <impl> [FFT_SSR]
#     impl 2 : fft_ssr        (Vitis xf::dsp SSR FFT)        — full spectrum
#     impl 3 : fft_ip_ssr_pre (LogiCORE-sub-FFT front end)   — ADC input scaling only
#                              (the FFT itself is an external xfft IP, not in HLS)
#     impl 5 : fft_hls_direct (direct hls::fft)              — full spectrum
#   e.g.  verify/run.sh 5 4      # IMPL=5, SSR=4
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
    2) tcl="$HERE/fft_ssr.tcl" ;;
    3) tcl="$HERE/fft_ip_ssr_pre.tcl" ;;
    5) tcl="$HERE/fft_hls_direct.tcl" ;;
    *) echo "unknown impl '$impl' (use 2, 3, or 5)" >&2; exit 2 ;;
esac

echo "==> FFT verify: IMPL=$impl  FFT_SSR=${FFT_SSR:-default}"
cd "$ROOT"
$HLS -f "$tcl"
