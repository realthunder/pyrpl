#!/bin/bash
# Stand-alone xsim testbenches of the ZCU216 RTL (no Vivado project):
#   tb_snap    zcu216_snap: frame snapshots into a DDR model over two AXI4 ports
#   tb_points  zcu216_points: the point stream into a DDR ring, against a model
#   tb_scope   zcu216_scope through its own writer: triggered traces, free run
#   tb_axi_slave  axi_slave (the PS register bus) under back-to-back writes
#   tb_fft_frame  zcu216_fft_frame: chirp framing against a model of the dit8 input
#   tb_fft_avg    zcu216_fft_avg: per-bin power average, bit-exact against a model
#   tb_bus_cdc    zcu216_bus_cdc: the system bus across two clocks, with a timeout
#   tb_dsp        zcu216_dsp: the signal hub and its modules
#   tb_adc_dec    zcu216_adc_dec: the RF-ADC channels at 250 MS/s
#   tb_rx         zcu216_rx: any band of an RF-ADC channel at 250 MS/s
#   tb_fmc_dac    zcu216_fmc_dac: 250 MS/s to the card's 125 MS/s
#   tb_bus_pipe   zcu216_bus_pipe: slots behind zcu216_bus_cdc
# Usage: sim/zcu216/run_unit.sh tb_snap   (from fpga/)
set -e
here=$(cd "$(dirname "$0")" && pwd)
fpga=$(cd "$here/../.." && pwd)
tb="${1:-tb_snap}"
case $tb in
  tb_snap) rtl="zcu216_snap.sv" ;;
  tb_points) rtl="zcu216_points.sv" ;;
  tb_scope) rtl="zcu216_scope.sv zcu216_snap.sv" ;;   # the writer is zcu216_snap_wr
  tb_axi_slave) rtl="axi_slave.v" ;;
  tb_fft_frame) rtl="zcu216_fft_frame.sv" ;;
  tb_fft_avg) rtl="zcu216_fft_avg.sv" ;;
  tb_bus_cdc) rtl="zcu216_bus_cdc.sv" ;;
  tb_dsp) rtl="zcu216_dsp.sv red_pitaya_pid_block.v red_pitaya_iq_block.v red_pitaya_iq_fgen_block.v
               red_pitaya_iq_demodulator_block.v red_pitaya_iq_modulator_block.v red_pitaya_pfd_block.v
               red_pitaya_filter_block.v red_pitaya_lpf_block.v red_pitaya_iq_lpf_block.v
               red_pitaya_iq_hpf_block.v red_pitaya_iir_block.v red_pitaya_product_sat.v
               red_pitaya_saturate.v" ;;
  tb_adc_dec) rtl="zcu216_adc_dec.sv zcu216_ring.sv" ;;
  tb_rx) rtl="zcu216_rx.sv zcu216_ring.sv" ;;
  tb_fmc_dac) rtl="zcu216_fmc_dac.sv" ;;
  tb_bus_pipe) rtl="zcu216_bus_cdc.sv zcu216_bus_pipe.sv" ;;
  *) echo "unknown testbench $tb"; exit 1 ;;
esac
work="$here/work/$tb"
mkdir -p "$work"
source /tools/Xilinx/2025.2/Vivado/settings64.sh
cd "$work"
# the Red Pitaya's .v files are Verilog, and some use a name before declaring it
xvlog -sv -d MODULE_FB_PIPELINE $(for f in $rtl; do case $f in *.sv) echo "$fpga/rtl/$f" ;; esac; done) "$here/$tb.sv" > xvlog.log 2>&1 \
    || { grep -E "ERROR|WARNING" xvlog.log; exit 1; }
v=$(for f in $rtl; do case $f in *.v) echo "$fpga/rtl/$f" ;; esac; done)
[ -z "$v" ] || xvlog --relax -d MODULE_FB_PIPELINE $v >> xvlog.log 2>&1 || { grep -E "ERROR" xvlog.log; exit 1; }
xelab -debug off --timescale 1ns/1ps $tb -s $tb > xelab.log 2>&1 || { grep -E "ERROR" xelab.log; exit 1; }
[ "$tb" != tb_rx ] || python3 -c "
import math
for i in range(4096): print('%04x' % (round(32767 * math.sin(2 * math.pi * i / 4096)) & 0xffff))" > sin.mem
xsim $tb -R > xsim.log 2>&1 || { tail -20 xsim.log; exit 1; }
grep -E "^(SNAP|POINTS|SCOPE|AXI|FRAME|AVG|CDC|DSP|DEC|FMC|PIPE|RX|ERR)|^  |Error|Fatal" xsim.log || true
[ "$tb" != tb_rx ] || "$fpga/../../.venv/bin/python" "$here/tb_rx.py" rx.txt
