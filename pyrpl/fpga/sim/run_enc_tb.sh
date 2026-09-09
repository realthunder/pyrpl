#!/bin/bash
# Self-checking sim of the Scanner360 encoder block. Needs no IP, so it runs
# straight on xvlog/xelab/xsim instead of a Vivado project (~10 s).
set -euo pipefail
root=$(cd "$(dirname "$0")/.." && pwd)
xdir=${XILINX_VIVADO:-/tools/Xilinx/2025.2/Vivado}/bin
out=${1:-$root/sim/.enc_tb}
mkdir -p "$out"
cd "$out"
"$xdir/xvlog" -sv "$root/rtl/red_pitaya_enc.sv" "$root/sim/tb_enc.sv"
"$xdir/xelab" -debug typical -timescale 1ns/1ps work.tb_enc -s tb_enc_snap
"$xdir/xsim" tb_enc_snap -runall
