# HLS build for fft_native_mag (FFT_IMPL==4: native-SSR xfft magnitude back-end).
# Sourced standalone via: vitis_hls -f hls/fft_ssr_native_mag.tcl
# or from make.sh check_hls after globals are set.

proc getparam {name default} {
    upvar #0 $name g
    if {[info exists g] && $g ne ""} { return $g }
    # Env vars are UPPERCASE (FFT_SSR); the tcl var name is lowercase (fft_ssr).
    set envname [string toupper $name]
    if {[info exists ::env($envname)]} { return $::env($envname) }
    return $default
}

set part           [getparam part           xc7z020clg400-1]
set fft_ssr        [getparam fft_ssr        2]
set fft_nfft       [getparam fft_nfft       12]
set fft_clk_period [getparam fft_clk_period 4.0]
# FFT_USE_APPROX=1 (default): fast alpha-max-beta-min magnitude (matches IMPL=3 default).
set fft_use_approx [getparam fft_use_approx 1]
set approx_flag    [expr {$fft_use_approx ? "-DUSE_APPROXIMATION" : ""}]
set fft_scaled     [getparam fft_scaled     2]
set scaled_flag    [expr {$fft_scaled == 1 ? "-DFFT_SCALED=1" : ""}]
set fft_width      [getparam fft_width      [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]

set path_out .hls
file mkdir $path_out
cd $path_out

open_project -reset fft_native_mag
add_files ../hls/fft_ssr_native.cpp \
    -cflags "-I../hls \
             -DFFT_SSR=$fft_ssr \
             -DFFT_NFFT=$fft_nfft \
             -DASZ=14 \
             -DINTERNAL_W=16 \
             -DDSZ=$fft_width \
             $approx_flag \
             $scaled_flag"

set_top fft_native_mag

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design
export_design -format ip_catalog -rtl verilog

exit
