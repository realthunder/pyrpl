set path_out .hls

set src_file "../hls/fft_ssr.cpp"
set vitis_include "../Vitis_Libraries/dsp/L1/include/hw/vitis_fft/fixed"

set proj_name fft_ssr
set top_func fft_ssr

global part
global fft_ssr
global fft_clk_period
global fft_nfft

# -------- START HLS --------
file mkdir $path_out
cd $path_out
open_project -reset $proj_name

# Add source
add_files $src_file -cflags "-I$vitis_include -DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft"

# Set top
set_top $top_func

# Create solution
open_solution -reset solution1

set_part $part
create_clock -period $fft_clk_period

# -------- RUN FLOW --------
csynth_design

# Export RTL (Verilog IP)
export_design -format ip_catalog -rtl verilog

# -------- DONE --------
exit
