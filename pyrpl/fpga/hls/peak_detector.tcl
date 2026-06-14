set path_out .hls

set src_file "../hls/peak_detector.cpp"
set tb_file  "../hls/peak_detector_tb.cpp"
set proj_name peak_detector
set top_func  peak_detector

global part
global fft_clk_period
global fft_ssr
global fft_width
global fft_nfft

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
