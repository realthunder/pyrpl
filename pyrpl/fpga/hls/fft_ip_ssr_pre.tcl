# HLS build script for fft_ip_ssr_pre (FFT_IMPL==3).
# Sourced standalone via: vivado_hls -f hls/fft_ip_ssr_pre.tcl
# or from red_pitaya_vivado.tcl after globals are set.

# ---- Parameters (globals override env, env overrides defaults) ----------
proc getparam {name default} {
    upvar #0 $name g
    if {[info exists g] && $g ne ""} { return $g }
    if {[info exists ::env($name)]}   { return $::env($name) }
    return $default
}

set part          [getparam part          xc7z020clg400-1]
set fft_ssr       [getparam fft_ssr       2]
set fft_nfft      [getparam fft_nfft      12]
set fft_clk_period [getparam fft_clk_period 4.0]

# Derived
set ssr_bits  [expr {int(log($fft_ssr) / log(2) + 0.5)}]
set sub_nfft  [expr {$fft_nfft - $ssr_bits}]
set sub_size  [expr {1 << $sub_nfft}]
set fft_size  [expr {1 << $fft_nfft}]

# xfft config word: scale_sched for sub-FFT, FWD_INV in bit 0
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

# ---- Generate twiddle LUT if not present --------------------------------
set hls_dir  [file dirname [info script]]
set lut_hdr  [file join $hls_dir fft_ip_ssr_twiddle.hpp]
set gen_src  [file join $hls_dir gen_twiddle_lut.cpp]
set gen_exe  [file join $hls_dir gen_twiddle_lut_exe]

if {![file exists $lut_hdr] || [file mtime $gen_src] > [file mtime $lut_hdr]} {
    puts "Generating twiddle LUT: $lut_hdr ..."
    set orig [pwd]
    cd $hls_dir

    # Try g++ first, then python3 as fallback
    set compiled 0
    if {![catch {exec g++ -O2 -o $gen_exe $gen_src}]} {
        if {![catch {exec $gen_exe $fft_size $fft_ssr 18} msg]} {
            puts $msg
            set compiled 1
        }
    }
    if {!$compiled} {
        puts "g++ unavailable or failed — trying python3 ..."
        set py_script [file join $hls_dir gen_twiddle_lut.py]
        set rc [catch {exec python3 $py_script $fft_size $fft_ssr 18} msg]
        if {$rc} { error "gen_twiddle_lut (python3) failed: $msg" }
        puts $msg
    }

    cd $orig
}

# ---- HLS project --------------------------------------------------------
set path_out .hls
file mkdir $path_out
cd $path_out

open_project -reset fft_ip_ssr_pre

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

set_top fft_ip_ssr_pre

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design

# export_design may fail with "bad lexical cast" on core_revision when the
# current timestamp exceeds INT_MAX (Vivado 2020.1 bug). Catch and re-run
# IP packaging after patching the revision to a safe value.
if {[catch {export_design -format ip_catalog -rtl verilog} err]} {
    puts "WARNING: export_design failed ($err) — patching core_revision ..."
    set ippack [glob -nocomplain [pwd]/fft_ip_ssr_pre/solution1/impl/ip/run_ippack.tcl]
    if {$ippack ne ""} {
        set fd [open $ippack r]; set src [read $fd]; close $fd
        # Replace any oversized revision number with a safe constant
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
