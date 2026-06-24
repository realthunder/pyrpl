# HLS build for fft_native_pre (FFT_IMPL==4: native-SSR xfft front-end).
# Sourced standalone via: vitis_hls -f hls/fft_ssr_native_pre.tcl
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
# FFT_SCALED: 0=unscaled, 1=scaled, 2=unscaled+saturate. Only mode 1 sets FFT_SCALED in HLS.
set fft_scaled     [getparam fft_scaled     2]
set scaled_flag    [expr {$fft_scaled == 1 ? "-DFFT_SCALED=1" : ""}]
# fft_width = DSZ (magnitude output bits): 0->28, 1->16, 2->20
set fft_width      [getparam fft_width      [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]

# Native SSR xfft config: 8-bit word, forward transform (bit0=1), no scaling schedule
# (unscaled) / scaled mode uses a fixed schedule that the IP applies internally.
set cfg_w    8
set cfg_word 1

set path_out .hls
file mkdir $path_out
cd $path_out

open_project -reset fft_native_pre
add_files ../hls/fft_ssr_native.cpp \
    -cflags "-I../hls \
             -DFFT_SSR=$fft_ssr \
             -DFFT_NFFT=$fft_nfft \
             -DASZ=14 \
             -DINTERNAL_W=16 \
             -DDSZ=$fft_width \
             -DCFG_W=$cfg_w \
             -DCFG_WORD=$cfg_word \
             $scaled_flag"

set_top fft_native_pre

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design
export_design -format ip_catalog -rtl verilog

exit
