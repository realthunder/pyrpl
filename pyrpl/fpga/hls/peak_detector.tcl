set path_out .hls

# PEAK_ALGO selects the detector implementation (same top function / IP name):
#   global (default) - single global mean/stdev threshold (peak_detector.cpp)
#   cfar             - moving-window CA-CFAR / local z-score (peak_detector_cfar.cpp).
#                      Adds runtime guard_cells/train_cells ports; NATURAL order only
#                      (requires FFT_IMPL=4). See peak_detector_cfar.cpp header.
set peak_algo [expr {[info exists env(PEAK_ALGO)] ? $env(PEAK_ALGO) : "global"}]

if {$peak_algo == "cfar"} {
    set src_file "../hls/peak_detector_cfar.cpp"
    set tb_file  "../hls/peak_detector_cfar_tb.cpp"
} else {
    set src_file "../hls/peak_detector.cpp"
    set tb_file  "../hls/peak_detector_tb.cpp"
}
set proj_name peak_detector
set top_func  peak_detector

set part           [expr {[info exists env(FPGA_PART)]      ? $env(FPGA_PART)      : "xc7z020clg400-1"}]
set fft_ssr        [expr {[info exists env(FFT_SSR)]        ? $env(FFT_SSR)        : 4}]
set fft_nfft       [expr {[info exists env(FFT_NFFT)]       ? $env(FFT_NFFT)       : 12}]
set fft_width      [expr {[info exists env(FFT_WIDTH)]      ? $env(FFT_WIDTH)      : 20}]
set fft_clk_period [expr {[info exists env(FFT_CLK_PERIOD)] ? $env(FFT_CLK_PERIOD) : 4.0}]
set fft_impl       [expr {[info exists env(FFT_IMPL)]       ? $env(FFT_IMPL)       : 5}]
# PEAK_FRAC: sub-bin interpolation fractional bits F (Q(FSZ).F output index).
# 0 disables interpolation entirely (plain integer bin, no extra logic).
set peak_frac      [expr {[info exists env(PEAK_FRAC)]      ? $env(PEAK_FRAC)      : 8}]

set cflags "-DFSSR=$fft_ssr -DDSZ=$fft_width -DFSZ=$fft_nfft -DFRAC_BITS=$peak_frac"

# CA-CFAR build: enable the extended (guard/train) signature and let the guard/
# train maxima be overridden from the environment (bound buffers/trip counts).
if {$peak_algo == "cfar"} {
    append cflags " -DPEAK_CFAR=1"
    if {[info exists env(CFAR_GUARD_MAX)]} { append cflags " -DCFAR_GUARD_MAX=$env(CFAR_GUARD_MAX)" }
    if {[info exists env(CFAR_TRAIN_MAX)]} { append cflags " -DCFAR_TRAIN_MAX=$env(CFAR_TRAIN_MAX)" }
}

# FFT_IMPL==4 is the native-SSR xfft, whose output is NATURAL order (PG109: SSR>1
# fixed-point is natural-only). Other engines default to DIF/bit_reversed_order, where
# the peak detector bit-reverses the streaming position to recover the bin.
# When interpolation is on (peak_frac != 0) the FFT is instead built in natural order
# (fft_hls_direct sets ordering_opt=natural_order) because sub-bin interp reads the two
# neighbouring bins, which are only adjacent in natural order. So natural order is
# required whenever IMPL==4 OR frac != 0 — keep this in lockstep with the ordering_opt
# gate in fft_hls_direct.tcl and the readback-address gate in fft_proc.sv.
if {$fft_impl == 4 || $peak_frac != 0} {
    append cflags " -DFFT_NATURAL_ORDER=1"
}

# -------- START HLS --------
file mkdir $path_out
cd $path_out
open_project -reset $proj_name

add_files          $src_file -cflags $cflags
add_files -tb      $tb_file  -cflags $cflags

set_top $top_func

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

# NOTE: config_compile -pipeline_style frp tried here — no effect. The STREAM
# loop carries accumulator/peak recurrences, so it is non-flushable and frp
# silently falls back to stp (identical RTL/timing). Left on default (stp).

# -------- RUN FLOW --------
# -ldflags "-B/usr/bin": use system ld (binutils 2.42) instead of Vivado's
# bundled binutils-2.26, which can't resolve libm GROUP paths on Ubuntu 24.04.
csim_design -ldflags "-B/usr/bin"
csynth_design

# Export RTL as Verilog IP
export_design -format ip_catalog -rtl verilog

# -------- DONE --------
exit
