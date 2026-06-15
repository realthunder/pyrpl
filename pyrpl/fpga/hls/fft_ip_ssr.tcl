set path_out .hls

set src_file "../hls/fft_ip_ssr.cpp"

set proj_name fft_ip_ssr
set top_func  fft_ip_ssr

set part           [expr {[info exists env(FPGA_PART)]      ? $env(FPGA_PART)      : "xc7z020clg400-1"}]
set fft_ssr        [expr {[info exists env(FFT_SSR)]        ? $env(FFT_SSR)        : 2}]
set fft_nfft       [expr {[info exists env(FFT_NFFT)]       ? $env(FFT_NFFT)       : 12}]
set fft_clk_period [expr {[info exists env(FFT_CLK_PERIOD)] ? $env(FFT_CLK_PERIOD) : 4.0}]

file mkdir $path_out
cd $path_out

open_project -reset $proj_name
add_files $src_file -cflags "-DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft"
set_top $top_func

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design
export_design -format ip_catalog -rtl verilog

exit
