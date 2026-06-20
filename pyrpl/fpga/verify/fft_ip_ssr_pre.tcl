# Input-scaling csim of fft_ip_ssr_pre (FFT_IMPL=3), compiled SSR=1 (passthrough pre).
# The FFT itself is an external xfft LogiCORE and cannot be csim'd here; this checks
# only the ADC->complex conversion. Run via verify/run.sh 3.
proc getp {name def} {
    set e [string toupper $name]
    if {[info exists ::env($e)] && $::env($e) ne ""} { return $::env($e) }
    return $def
}
set part     [getp part     xc7z020clg400-1]
set fft_nfft [getp fft_nfft 12]
set sub_nfft $fft_nfft
set cfg_w    [expr {((((($sub_nfft + 1) / 2) * 2) + 1 + 7) / 8) * 8}]

# fft_ip_ssr.cpp #includes the twiddle header; the SSR=1 code doesn't use its arrays,
# but the header must exist to compile. Generate an SSR=2 table if absent.
set hls_dir [file normalize [file join [file dirname [info script]] .. hls]]
set lut [file join $hls_dir fft_ip_ssr_twiddle.hpp]
if {![file exists $lut]} {
    set o [pwd]; cd $hls_dir
    if {[catch {exec g++ -O2 -o gen_twiddle_lut_exe gen_twiddle_lut.cpp}] || \
        [catch {exec ./gen_twiddle_lut_exe [expr {1 << $fft_nfft}] 2 18} m]} {
        exec python3 gen_twiddle_lut.py [expr {1 << $fft_nfft}] 2 18
    }
    cd $o
}

set defs "-DFFT_SSR=1 -DFFT_NFFT=$fft_nfft -DSUB_NFFT=$sub_nfft -DCFG_W=$cfg_w -DCFG_WORD=1 -DASZ=14 -DINT_W=16"
file mkdir .hls; cd .hls
open_project -reset fft_ip_ssr_pre_verify
add_files     ../hls/fft_ip_ssr.cpp           -cflags "-I../hls $defs"
add_files -tb ../verify/fft_ip_ssr_pre_tb.cpp -cflags "-I../verify $defs"
set_top fft_ip_ssr_pre
open_solution -reset solution1
set_part $part
create_clock -period 8.0
csim_design
exit
