#!/usr/bin/env python3
# gen_twiddle_lut.py — Python3 equivalent of gen_twiddle_lut.cpp
# Usage: python3 gen_twiddle_lut.py <FFT_SIZE> <FFT_SSR> <TWID_W>
# Writes fft_ip_ssr_twiddle.hpp in the current directory.

import sys
import math

def main():
    if len(sys.argv) != 4:
        print("Usage: gen_twiddle_lut.py FFT_SIZE FFT_SSR TWID_W", file=sys.stderr)
        sys.exit(1)

    fft_size = int(sys.argv[1])
    fft_ssr  = int(sys.argv[2])
    twid_w   = int(sys.argv[3])

    if fft_ssr not in (2, 4, 8):
        print("ERROR: FFT_SSR must be 2, 4, or 8", file=sys.stderr)
        sys.exit(1)

    sub_size   = fft_size // fft_ssr
    ssr_minus1 = fft_ssr - 1
    PI         = math.pi

    with open("fft_ip_ssr_twiddle.hpp", "w") as f:
        f.write("// AUTO-GENERATED — do not edit.\n")
        f.write(f"// FFT_SIZE={fft_size}  FFT_SSR={fft_ssr}  TWID_W={twid_w}\n")
        f.write(f"// W_{{FFT_SIZE}}^{{r*k}} for r=1..{ssr_minus1}, k=0..{sub_size-1}\n")
        f.write("#ifndef FFT_IP_SSR_TWIDDLE_HPP\n")
        f.write("#define FFT_IP_SSR_TWIDDLE_HPP\n\n")
        f.write('#include "ap_fixed.h"\n\n')
        f.write(f"typedef ap_fixed<{twid_w}, 2> twid_lut_t;\n\n")

        for r in range(1, fft_ssr):
            f.write(f"static const twid_lut_t twid_re_{r}[{sub_size}] = {{\n")
            for k in range(sub_size):
                angle = -2.0 * PI * r * k / fft_size
                val   = math.cos(angle)
                comma = "," if k < sub_size - 1 else ""
                f.write(f"    {val:.18f}{comma}")
                if (k + 1) % 4 == 0 or k == sub_size - 1:
                    f.write("\n")
            f.write("};\n\n")

            f.write(f"static const twid_lut_t twid_im_{r}[{sub_size}] = {{\n")
            for k in range(sub_size):
                angle = -2.0 * PI * r * k / fft_size
                val   = math.sin(angle)
                comma = "," if k < sub_size - 1 else ""
                f.write(f"    {val:.18f}{comma}")
                if (k + 1) % 4 == 0 or k == sub_size - 1:
                    f.write("\n")
            f.write("};\n\n")

        f.write("#endif // FFT_IP_SSR_TWIDDLE_HPP\n")

    print(f"Generated fft_ip_ssr_twiddle.hpp ({sub_size} entries per row, {ssr_minus1} rows)")

if __name__ == "__main__":
    main()
