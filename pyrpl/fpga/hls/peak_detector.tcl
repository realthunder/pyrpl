set path_out .hls

set src_file "../hls/peak_detector.cpp"
set tb_file  "../hls/peak_detector_tb.cpp"
set proj_name peak_detector
set top_func  peak_detector

set part           [expr {[info exists env(FPGA_PART)]      ? $env(FPGA_PART)      : "xc7z020clg400-1"}]
set fft_ssr        [expr {[info exists env(FFT_SSR)]        ? $env(FFT_SSR)        : 2}]
set fft_nfft       [expr {[info exists env(FFT_NFFT)]       ? $env(FFT_NFFT)       : 12}]
set fft_width      [expr {[info exists env(FFT_WIDTH)]      ? $env(FFT_WIDTH)      : 28}]
set fft_clk_period [expr {[info exists env(FFT_CLK_PERIOD)] ? $env(FFT_CLK_PERIOD) : 4.0}]

set cflags "-DFSSR=$fft_ssr -DDSZ=$fft_width -DFSZ=$fft_nfft"

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

# -------- RUN FLOW --------
csim_design
csynth_design

# Export RTL as Verilog IP
export_design -format ip_catalog -rtl verilog

# -------- DONE --------
exit
