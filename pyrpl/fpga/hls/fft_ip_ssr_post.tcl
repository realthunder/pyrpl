# HLS build script for fft_ip_ssr_post (FFT_IMPL==3).
# Sourced standalone via: vivado_hls -f hls/fft_ip_ssr_post.tcl
# or from red_pitaya_vivado.tcl after globals are set.

proc getparam {name default} {
    upvar #0 $name g
    if {[info exists g] && $g ne ""} { return $g }
    if {[info exists ::env($name)]}   { return $::env($name) }
    return $default
}

set part           [getparam part           xc7z020clg400-1]
set fft_ssr        [getparam fft_ssr        2]
set fft_nfft       [getparam fft_nfft       12]
set fft_clk_period [getparam fft_clk_period 4.0]
# FFT_USE_APPROX=1: use fast alpha-max-beta-min (matches old behaviour).
# Default (0): CORDIC vectoring mode, same convention as fft_ssr.cpp.
set fft_use_approx [getparam fft_use_approx 1]
set approx_flag    [expr {$fft_use_approx ? "-DUSE_APPROXIMATION" : ""}]
# FFT_SCALED=0 (default): unscaled mode — xfft accumulates without ÷2 → ~140 dB dynamic range.
# FFT_SCALED=1: scaled mode — ÷2 per stage, output stays in INT_W range → ~72 dB.
set fft_scaled     [getparam fft_scaled     0]
set scaled_flag    [expr {$fft_scaled ? "-DFFT_SCALED=1" : ""}]

set ssr_bits [expr {int(log($fft_ssr) / log(2) + 0.5)}]
set sub_nfft [expr {$fft_nfft - $ssr_bits}]
set fft_size [expr {1 << $fft_nfft}]

proc scale_sched_val {n} {
    set ibits [expr {($n + 1) / 2}]
    if {$n % 2 == 1} { set s 1 } else { set s 2 }
    for {set i 1} {$i < $ibits - 1} {incr i} {
        set s [expr {($s << 2) | 2}]
    }
    set s [expr {($s << 2) | 3}]
    return $s
}
set sch      [scale_sched_val $sub_nfft]
set cfg_word [expr {($sch << 1) | 1}]
set sch_w    [expr {(($sub_nfft + 1) / 2) * 2}]
set cfg_w    [expr {(($sch_w + 1 + 7) / 8) * 8}]

# Ensure twiddle header exists (pre TCL generates it; post just needs the
# header to be present for compilation even though post doesn't use LUT).
set hls_dir [file normalize [file dirname [info script]]]
set lut_hdr [file join $hls_dir fft_ip_ssr_twiddle.hpp]
if {![file exists $lut_hdr]} {
    error "fft_ip_ssr_twiddle.hpp missing — run fft_ip_ssr_pre.tcl first"
}

set path_out .hls
file mkdir $path_out
cd $path_out

open_project -reset fft_ip_ssr_post

add_files ../hls/fft_ip_ssr.cpp \
    -cflags "-I../hls \
             -I/usr/include \
             -I/usr/include/x86_64-linux-gnu \
             -DFFT_SSR=$fft_ssr \
             -DFFT_NFFT=$fft_nfft \
             -DASZ=14 \
             -DINT_W=16 \
             -DTWID_W=18 \
             -DSUB_NFFT=$sub_nfft \
             -DCFG_W=$cfg_w \
             -DCFG_WORD=$cfg_word \
             $approx_flag \
             $scaled_flag"

set_top fft_ip_ssr_post

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design

export_design -format ip_catalog -rtl verilog

exit
