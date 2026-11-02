#!/bin/bash
# zcu216_peak_wide (RTL) against its Python model, bit for bit.
# Usage: sim/zcu216/run_peak_wide.sh [SUB] [FRAMES] [SEED]   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
sub="${1:-11}"
frames="${2:-120}"
seed="${3:-1}"
py="${PYTHON:-$fpga/../../.venv/bin/python}"
work="$here/work/pw$sub"
mkdir -p "$work"
source /tools/Xilinx/2025.2/Vivado/settings64.sh

"$py" "$here/peak_wide_gen.py" "$work/frames.hex" "$work/exp.hex" $((sub + 2)) $frames $seed

cd "$work"
xvlog -sv -d PW_SUB=$sub -d "PW_IN=\"$work/frames.hex\"" -d "PW_OUT=\"$work/rtl.hex\"" \
      "$fpga/rtl/zcu216_peak_wide.sv" "$here/tb_peak_wide.sv" > xvlog.log 2>&1 \
    || { grep -E "ERROR|WARNING" xvlog.log; exit 1; }
xelab -debug off --timescale 1ns/1ps tb_peak_wide -s tb_pw > xelab.log 2>&1 || { grep -E "ERROR" xelab.log; exit 1; }
xsim tb_pw -R > xsim.log 2>&1 || { tail -20 xsim.log; exit 1; }
grep -E "^PW|Error|Fatal" xsim.log || true

"$py" -I - "$work/exp.hex" "$work/rtl.hex" <<'PY'
import sys
ref = open(sys.argv[1]).read().split()
rtl = open(sys.argv[2]).read().split()
bad = [i for i in range(min(len(ref), len(rtl))) if ref[i] != rtl[i]]
det = sum(int(w, 16) >> 32 & 1 for w in ref)
def fld(w):
    v = int(w, 16)
    return f'mag {v & 0xffffffff} det {v >> 32 & 1} k {(v >> 33) / 256:.4f}'
print(f'frames ref {len(ref)} rtl {len(rtl)}, detections {det}, mismatches {len(bad)}')
for i in bad[:8]:
    print(f'  frame {i}\n    ref {fld(ref[i])}\n    rtl {fld(rtl[i])}')
sys.exit(0 if not bad and len(ref) == len(rtl) else 1)
PY
