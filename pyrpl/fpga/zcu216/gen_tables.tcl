# ROM tables for the ZCU216 radix-8 FFT (rtl/zcu216_fft_dit8.sv) and the test
# tone generator (rtl/zcu216_tone_gen.sv), as $readmemh files.
#
#   zcu216_gen_tables <dir> <nfft> [tw]
#
# writes into <dir>:
#   zcu216_twc.mem  coarse twiddles Wc[h] = exp(-j*2*pi*h*2^L / 2^nfft), 2^H entries
#   zcu216_twf.mem  fine twiddles   Wf[l] = exp(-j*2*pi*l / 2^nfft),     2^L entries
#                   (H = ceil(nfft/2), L = nfft - H; W[a] = Wc[a >> L] * Wf[a & (2^L-1)])
#                   one {im, re} word per line, each tw bits, Q1.(tw-1)
#   zcu216_sin.mem  4096-entry sine, 16-bit signed, amplitude 32767
#
# Plain Tcl, so the Vivado build and the xsim testbench share it.

proc zcu216_gen_tables {dir nfft {tw 18}} {
    set pi    [expr {acos(-1.0)}]
    set h     [expr {($nfft + 1) / 2}]
    set l     [expr {$nfft - $h}]
    set scale [expr {(1 << ($tw - 1)) - 1}]
    set mask  [expr {(1 << $tw) - 1}]
    set digits [expr {(2 * $tw + 3) / 4}]
    file mkdir $dir

    foreach {name n step} [list twc [expr {1 << $h}] [expr {1 << $l}] \
                                twf [expr {1 << $l}] 1] {
        set f [open [file join $dir zcu216_$name.mem] w]
        for {set i 0} {$i < $n} {incr i} {
            set th [expr {2.0 * $pi * $i * $step / double(1 << $nfft)}]
            set re [expr {round($scale * cos($th))}]
            set im [expr {round(-$scale * sin($th))}]
            puts $f [format %0*llx $digits [expr {(($im & $mask) << $tw) | ($re & $mask)}]]
        }
        close $f
    }

    set f [open [file join $dir zcu216_sin.mem] w]
    for {set i 0} {$i < 4096} {incr i} {
        puts $f [format %04x [expr {round(32767 * sin(2.0 * $pi * $i / 4096.0)) & 0xffff}]]
    }
    close $f
}
