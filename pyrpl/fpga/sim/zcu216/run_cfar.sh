#!/bin/bash
# zcu216_cfar (RTL) against hls/peak_detector_cfar.cpp (C++), bit for bit:
# cfar_gen.py makes FRAMES frames of 4 * 2^SUB bins with random settings,
# cfar_ref (g++ with the Vitis HLS headers) and xsim each produce one word per
# frame, and every word must match.
# Usage: sim/zcu216/run_cfar.sh [SUB] [FRAMES] [SEED]   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
sub="${1:-10}"
frames="${2:-200}"
seed="${3:-1}"
work="$here/work/cfar$sub"
mkdir -p "$work"
source /tools/Xilinx/2025.2/Vivado/settings64.sh
fsz=$((sub + 2))

python3 -I "$here/cfar_gen.py" "$work/frames.hex" $fsz $frames $seed

g++ -O2 -std=c++14 -w -I /tools/Xilinx/2025.2/Vitis/include -I "$fpga/hls" \
    -DFFT_NATURAL_ORDER -DPEAK_CFAR -DFSSR=4 -DDSZ=32 -DFSZ=$fsz -DSSZ=$((fsz + 1)) \
    -DFRAC_BITS=8 -DPEAK_RAMP=1 -DCFAR_GUARD_MAX=16 -DCFAR_TRAIN_MAX=64 \
    -o "$work/cfar_ref" "$here/cfar_ref.cpp"
"$work/cfar_ref" "$work/frames.hex" | grep -x "[0-9a-f]\{16\}" > "$work/cfar_ref.hex"

cd "$work"
xvlog -sv -d CFAR_SUB=$sub -d "CFAR_IN=\"$work/frames.hex\"" -d "CFAR_OUT=\"$work/cfar_rtl.hex\"" \
      "$fpga/rtl/zcu216_fft_dit8.sv" "$fpga/rtl/zcu216_cfar.sv" "$here/tb_cfar.sv" > xvlog.log 2>&1 \
    || { grep -E "ERROR|WARNING" xvlog.log; exit 1; }
xelab -debug off --timescale 1ns/1ps tb_cfar -s tb_cfar > xelab.log 2>&1 || { grep -E "ERROR" xelab.log; exit 1; }
xsim tb_cfar -R > xsim.log 2>&1 || { tail -20 xsim.log; exit 1; }
grep -E "^CFAR|Error|Fatal" xsim.log || true

python3 -I - "$work/cfar_ref.hex" "$work/cfar_rtl.hex" "$work/frames.hex" $fsz <<'EOF'
import sys
ref = open(sys.argv[1]).read().split()
rtl = open(sys.argv[2]).read().split()
fsz = int(sys.argv[4])
lines = open(sys.argv[3]).read().split()
per = 10 + (1 << fsz)
cfgs = [lines[i * per:i * per + 10] for i in range(len(ref))]
bad = [i for i in range(min(len(ref), len(rtl))) if ref[i] != rtl[i]]
det = sum(int(w, 16) >> 32 & 1 for w in ref)
def fld(w):
    v = int(w, 16)
    return f'mag {v & 0xffffffff} det {v >> 32 & 1} k {(v >> 33) / 256:.4f}'
print(f'frames ref {len(ref)} rtl {len(rtl)}, detections {det}, mismatches {len(bad)}')
for i in bad[:10]:
    print(f'  frame {i} cfg {[int(c, 16) for c in cfgs[i]]}\n    ref {fld(ref[i])}\n    rtl {fld(rtl[i])}')
sys.exit(0 if not bad and len(ref) == len(rtl) else 1)
EOF
