#!/bin/bash
# Stand-alone xsim testbenches of the ZCU216 RTL (no Vivado project):
#   tb_snap  zcu216_snap: frame snapshots into a DDR model over two AXI4 ports
# Usage: sim/zcu216/run_unit.sh tb_snap   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
tb="${1:-tb_snap}"
case $tb in
  tb_snap) rtl="zcu216_snap.sv" ;;
  *) echo "unknown testbench $tb"; exit 1 ;;
esac
work="$here/work/$tb"
mkdir -p "$work"
source /tools/Xilinx/2025.2/Vivado/settings64.sh
cd "$work"
xvlog -sv $(for f in $rtl; do echo "$fpga/rtl/$f"; done) "$here/$tb.sv" > xvlog.log 2>&1 \
    || { grep -E "ERROR|WARNING" xvlog.log; exit 1; }
xelab -debug off --timescale 1ns/1ps $tb -s $tb > xelab.log 2>&1 || { grep -E "ERROR" xelab.log; exit 1; }
xsim $tb -R > xsim.log 2>&1 || { tail -20 xsim.log; exit 1; }
grep -E "^SNAP|Error|Fatal" xsim.log || true
