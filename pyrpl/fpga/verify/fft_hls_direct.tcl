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

# twiddle ROM must match FFT_SSR (gen if missing/mismatched) — same as the build
set hls_dir [file normalize [file join [file dirname [info script]] .. hls]]
set lut [file join $hls_dir fft_ip_ssr_twiddle.hpp]
set need 1
if {[file exists $lut]} {
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

set defs "-DFFT_SSR=$fft_ssr -DFFT_NFFT=$fft_nfft -DASZ=14 -DDSZ=$fft_width"
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
