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
set hls_dir [file dirname [info script]]
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
             -DDSZ=28 \
             -DSUB_NFFT=$sub_nfft \
             -DCFG_W=$cfg_w \
             -DCFG_WORD=$cfg_word"

set_top fft_ip_ssr_post

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design

# Same Vivado 2020.1 core_revision overflow workaround as pre.tcl
if {[catch {export_design -format ip_catalog -rtl verilog} err]} {
    puts "WARNING: export_design failed ($err) — patching core_revision ..."
    set ippack [glob -nocomplain [pwd]/fft_ip_ssr_post/solution1/impl/ip/run_ippack.tcl]
    if {$ippack ne ""} {
        set fd [open $ippack r]; set src [read $fd]; close $fd
        regsub {set Revision\s+"[0-9]+"} $src {set Revision "1"} src
        set fd [open $ippack w]; puts -nonewline $fd $src; close $fd
        exec /tools/Xilinx/Vivado/2020.1/bin/vivado \
            -notrace -mode batch -source $ippack
        puts "INFO: IP packaging completed after revision patch."
    } else {
        error "run_ippack.tcl not found after export_design failure"
    }
}

exit
