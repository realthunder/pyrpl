#!/bin/bash
# red_pitaya_asg + zcu216_sig_cdc (the ZCU216 ASG into the scope's clock): see tb_asg.sv
# Usage: sim/zcu216/run_asg.sh   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
work="$here/work/asg"
mkdir -p "$work"
source /tools/Xilinx/2025.2/Vivado/settings64.sh
cd "$work"
xvlog "$fpga/rtl/red_pitaya_asg.v" "$fpga/rtl/red_pitaya_asg_ch.v" \
      "$fpga/rtl/red_pitaya_adv_trigger.v" "$fpga/rtl/red_pitaya_prng.v" > xvlog.log 2>&1 \
    || { grep -E "ERROR" xvlog.log; exit 1; }
xvlog -sv "$fpga/rtl/zcu216_sig_cdc.sv" "$here/tb_asg.sv" >> xvlog.log 2>&1 \
    || { grep -E "ERROR" xvlog.log; exit 1; }
xelab -debug off --timescale 1ns/1ps tb_asg -s tb_asg > xelab.log 2>&1 || { grep -E "ERROR" xelab.log; exit 1; }
xsim tb_asg -R > xsim.log 2>&1 || { tail -20 xsim.log; exit 1; }
grep -E "^B-D|^ASG|^BAD|PASS|FAIL|Error|Fatal" xsim.log
