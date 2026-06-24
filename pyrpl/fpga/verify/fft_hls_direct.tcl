# csim of fft_hls_direct (FFT_IMPL=5) — full spectrum. Run via verify/run.sh 5 [SSR].
proc getp {name def} {
    set e [string toupper $name]
    if {[info exists ::env($e)] && $::env($e) ne ""} { return $::env($e) }
    return $def
}
set part       [getp part       xc7z020clg400-1]
set fft_ssr    [getp fft_ssr    4]
set fft_nfft   [getp fft_nfft   12]
set fft_scaled [getp fft_scaled 2]
set fft_width  [getp fft_width  [expr {$fft_scaled == 1 ? 16 : ($fft_scaled == 2 ? 20 : 28)}]]
set fft_size   [expr {1 << $fft_nfft}]

# twiddle ROM must match FFT_SSR (gen if missing/mismatched) — same as the build.
# SSR=1 uses no twiddles (no DIF pre-stage), so it needs no ROM.
set hls_dir [file normalize [file join [file dirname [info script]] .. hls]]
set lut [file join $hls_dir fft_ip_ssr_twiddle.hpp]
set need [expr {$fft_ssr >= 2}]
if {$need && [file exists $lut]} {
    set fh [open $lut r]; set h [read $fh 256]; close $fh
    if {[string match "*FFT_SIZE=$fft_size *FFT_SSR=$fft_ssr *" $h]} { set need 0 }
}
if {$need} {
    set o [pwd]; cd $hls_dir
    if {[catch {exec g++ -O2 -o gen_twiddle_lut_exe gen_twiddle_lut.cpp}] || \
        [catch {exec ./gen_twiddle_lut_exe $fft_size $fft_ssr 18} m]} {
        exec python3 gen_twiddle_lut.py $fft_size $fft_ssr 18
    }
    cd $o
}

# Default ON here so csim exercises the run-time configurable length (the max
# length it sweeps also covers the fixed-length path). Set FFT_RUNTIME_NFFT=0 to
# verify the fixed-length build instead.
set fft_runtime    [getp fft_runtime_nfft 1]
set fft_internal_w [getp fft_internal_w   16]
set fft_use_approx [getp fft_use_approx   1]
set fft_unscaled    [getp fft_unscaled     0]
set fft_cordic_iter [getp fft_cordic_iter  10]
set defs "-DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft -DASZ=14 -DDSZ=$fft_width -DINTERNAL_W=$fft_internal_w -DCORDIC_ITER=$fft_cordic_iter"
if {$fft_runtime}      { append defs " -DFFT_RUNTIME_NFFT" }
if {$fft_use_approx}   { append defs " -DUSE_APPROXIMATION" }
if {$fft_unscaled}     { append defs " -DFFT_UNSCALED" }
file mkdir .hls; cd .hls
open_project -reset fft_hls_direct_verify
add_files     ../hls/fft_hls_direct.cpp    -cflags "-I../hls $defs"
add_files -tb ../verify/fft_hls_direct_tb.cpp -cflags "-I../verify $defs"
set_top fft_hls_direct
open_solution -reset solution1
set_part $part
create_clock -period 8.0
csim_design
exit
