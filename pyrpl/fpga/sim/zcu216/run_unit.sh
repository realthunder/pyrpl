#!/bin/bash
# Stand-alone xsim testbenches of the ZCU216 RTL (no Vivado project):
#   tb_snap    zcu216_snap: frame snapshots into a DDR model over two AXI4 ports
#   tb_points  zcu216_points: the point stream into a DDR ring, against a model
#   tb_scope   zcu216_scope through its own writer: triggered traces, free run
# Usage: sim/zcu216/run_unit.sh tb_snap   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
tb="${1:-tb_snap}"
case $tb in
  tb_snap) rtl="zcu216_snap.sv" ;;
  tb_points) rtl="zcu216_points.sv" ;;
  tb_scope) rtl="zcu216_scope.sv zcu216_scope_tavg.sv zcu216_snap.sv" ;;   # the writer is zcu216_snap_wr
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
grep -E "^(SNAP|POINTS|SCOPE|ERR)|^  |Error|Fatal" xsim.log || true
