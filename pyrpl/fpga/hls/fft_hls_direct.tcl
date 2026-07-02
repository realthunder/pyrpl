# HLS build for fft_hls_direct (FFT_IMPL==5).
# Instantiates the LogiCORE FFT directly inside HLS via hls::fft<> (hls_fft.h),
# fusing the SSR butterfly/twiddle pre, the sub-FFTs, and the magnitude post into
# ONE HLS IP — no block design and no separate post IP. This is the approach that
# failed on Vivado 2020.1 (hls::fft internal buffer corruption under dataflow);
# retried on 2025.2.
#
# Sourced standalone via: vitis_hls -f hls/fft_hls_direct.tcl
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
set fft_ssr        [getparam fft_ssr        4]
set fft_nfft       [getparam fft_nfft       12]
set fft_clk_period [getparam fft_clk_period 4.0]
set fft_scaled     [getparam fft_scaled     2]
# fft_width = DSZ (magnitude output bits): 0->28, 1->16, 2->20
set fft_width      [getparam fft_width      [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]
# fft_internal_w = INTERNAL_W: internal FFT datapath precision (wider = more small-signal
# dynamic range, at DSP/BRAM cost). Default 16.
set fft_internal_w [getparam fft_internal_w 16]
set fft_size       [expr {1 << $fft_nfft}]
# FFT_MULT_LUT=1: build the FFT complex multipliers/butterflies in LUTs, not DSP.
set fft_mult_lut   [getparam fft_mult_lut    0]
set mult_lut_flag  [expr {$fft_mult_lut ? "-DFFT_MULT_LUT" : ""}]
# FFT_RUNTIME_NFFT=1: run-time configurable transform length (PS can change the
# FFT size on the fly via fft_nfft). Adds ~2700 LUTs; off by default so the
# shipping fixed-length build keeps its smaller footprint and adc/dac timing.
set fft_runtime    [getparam fft_runtime_nfft 0]
set runtime_flag   [expr {$fft_runtime ? "-DFFT_RUNTIME_NFFT" : ""}]
# FFT_USE_APPROX=1 (default): fast alpha-max-beta-min magnitude. =0: CORDIC vectoring.
set fft_use_approx [getparam fft_use_approx 1]
set approx_flag    [expr {$fft_use_approx ? "-DUSE_APPROXIMATION" : ""}]
# FFT_UNSCALED=1: run the hls::fft sub-cores unscaled (no per-stage /2; output grows
# by SUB_NFFT bits -> more small-signal dynamic range). Default 0 = scaled.
set fft_unscaled   [getparam fft_unscaled 0]
set unscaled_flag  [expr {$fft_unscaled ? "-DFFT_UNSCALED" : ""}]
# FFT_CORDIC_ITER: CORDIC magnitude iterations (default 10). Fewer = smaller/less
# accurate. Only used when FFT_USE_APPROX=0.
set fft_cordic_iter [getparam fft_cordic_iter 10]

# PEAK_FRAC: sub-bin interpolation fractional bits (matches the peak_detector build).
# When != 0 the peak detector interpolates using the two neighbouring bins, which are
# only adjacent in NATURAL bin order — so build the hls::fft in natural_order instead of
# the default bit_reversed_order. Keep this default (8) in lockstep with peak_detector.tcl
# and red_pitaya_vivado.tcl so all three agree on the ordering. See fft_hls_direct.cpp
# (ordering_opt) and fft_proc.sv (readback-address gate).
set peak_frac      [getparam peak_frac 8]
set natural_flag   [expr {$peak_frac != 0 ? "-DFFT_NATURAL_ORDER" : ""}]

# ---- Generate twiddle LUT if not present (shared with FFT_IMPL=3) ----------
set hls_dir [file normalize [file dirname [info script]]]
set lut_hdr [file join $hls_dir fft_ip_ssr_twiddle.hpp]
set gen_src [file join $hls_dir gen_twiddle_lut.cpp]
set gen_exe [file join $hls_dir gen_twiddle_lut_exe]
# Regenerate when missing, when the generator changed, OR when the header's
# params (FFT_SIZE/FFT_SSR in its comment) don't match this build — the ROM
# layout differs by SSR (e.g. SSR=4 adds twid_re_2/twid_re_3 rows).
# SSR=1 uses no twiddles (no DIF pre-stage), so it needs no ROM.
set need_gen 0
if {$fft_ssr < 2} {
    set need_gen 0
} elseif {![file exists $lut_hdr] || [file mtime $gen_src] > [file mtime $lut_hdr]} {
    set need_gen 1
} else {
    set fh [open $lut_hdr r]; set hdr [read $fh 256]; close $fh
    if {![string match "*FFT_SIZE=$fft_size *FFT_SSR=$fft_ssr *" $hdr]} { set need_gen 1 }
}
if {$need_gen} {
    puts "Generating twiddle LUT: $lut_hdr ..."
    set orig [pwd]
    cd $hls_dir
    set compiled 0
    if {![catch {exec g++ -O2 -o $gen_exe $gen_src}]} {
        if {![catch {exec $gen_exe $fft_size $fft_ssr 18} msg]} { puts $msg; set compiled 1 }
    }
    if {!$compiled} {
        set py_script [file join $hls_dir gen_twiddle_lut.py]
        set rc [catch {exec python3 $py_script $fft_size $fft_ssr 18} msg]
        if {$rc} { error "gen_twiddle_lut failed: $msg" }
        puts $msg
    }
    cd $orig
}

set path_out .hls
file mkdir $path_out
cd $path_out

open_project -reset fft_hls_direct
add_files ../hls/fft_hls_direct.cpp \
    -cflags "-I../hls \
             -DFFT_SSR=$fft_ssr \
             -DFFT_NFFT=$fft_nfft \
             -DASZ=14 \
             -DDSZ=$fft_width \
             -DINTERNAL_W=$fft_internal_w \
             $mult_lut_flag \
             $runtime_flag \
             $approx_flag \
             $unscaled_flag \
             $natural_flag \
             -DCORDIC_ITER=$fft_cordic_iter"

set_top fft_hls_direct

open_solution -reset solution1
set_part $part
create_clock -period $fft_clk_period

csynth_design
export_design -format ip_catalog -rtl verilog

exit
