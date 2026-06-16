// Generates fft_ip_ssr_twiddle.hpp — twiddle ROM for fft_ip_ssr_pre.
// Usage: gen_twiddle_lut <FFT_SIZE> <FFT_SSR> <TWID_W>
// Outputs fft_ip_ssr_twiddle.hpp in the current directory.
//
// For SSR=2 DIF: needs W_{FFT_SIZE}^k for k=0..FFT_SIZE/2-1.
// For SSR=4 DIF: needs W_{FFT_SIZE}^{r*k} for r=1..3, k=0..FFT_SIZE/4-1.

#include <cstdio>
#include <cstdlib>
#include <cmath>

static const double PI = 3.141592653589793238462643383;

int main(int argc, char** argv) {
    if (argc != 4) {
        fprintf(stderr, "Usage: gen_twiddle_lut FFT_SIZE FFT_SSR TWID_W\n");
        return 1;
    }

    int fft_size = atoi(argv[1]);
    int fft_ssr  = atoi(argv[2]);
    int twid_w   = atoi(argv[3]);

    if (fft_ssr != 2 && fft_ssr != 4) {
        fprintf(stderr, "ERROR: FFT_SSR must be 2 or 4\n");
        return 1;
    }

    int sub_size = fft_size / fft_ssr;  // twiddle table depth per row
    int ssr_minus1 = fft_ssr - 1;       // number of non-trivial rows

    FILE *f = fopen("fft_ip_ssr_twiddle.hpp", "w");
    if (!f) { perror("fopen"); return 1; }

    fprintf(f, "// AUTO-GENERATED — do not edit.\n");
    fprintf(f, "// FFT_SIZE=%d  FFT_SSR=%d  TWID_W=%d\n", fft_size, fft_ssr, twid_w);
    fprintf(f, "// W_{FFT_SIZE}^{r*k} for r=1..%d, k=0..%d\n", ssr_minus1, sub_size-1);
    fprintf(f, "#ifndef FFT_IP_SSR_TWIDDLE_HPP\n");
    fprintf(f, "#define FFT_IP_SSR_TWIDDLE_HPP\n\n");
    fprintf(f, "#include \"ap_fixed.h\"\n\n");
    fprintf(f, "typedef ap_fixed<%d, 2> twid_lut_t;\n\n", twid_w);

    // For each non-trivial twiddle row r=1..SSR-1
    for (int r = 1; r < fft_ssr; r++) {
        fprintf(f, "static const twid_lut_t twid_re_%d[%d] = {\n", r, sub_size);
        for (int k = 0; k < sub_size; k++) {
            double angle = -2.0 * PI * r * k / fft_size;
            double val   = cos(angle);
            fprintf(f, "    %.18f", val);
            if (k < sub_size - 1) fprintf(f, ",");
            if ((k+1) % 4 == 0 || k == sub_size-1) fprintf(f, "\n");
        }
        fprintf(f, "};\n\n");

        fprintf(f, "static const twid_lut_t twid_im_%d[%d] = {\n", r, sub_size);
        for (int k = 0; k < sub_size; k++) {
            double angle = -2.0 * PI * r * k / fft_size;
            double val   = sin(angle);
            fprintf(f, "    %.18f", val);
            if (k < sub_size - 1) fprintf(f, ",");
            if ((k+1) % 4 == 0 || k == sub_size-1) fprintf(f, "\n");
        }
        fprintf(f, "};\n\n");
    }

    fprintf(f, "#endif // FFT_IP_SSR_TWIDDLE_HPP\n");
    fclose(f);
    fprintf(stdout, "Generated fft_ip_ssr_twiddle.hpp (%d entries per row, %d rows)\n",
            sub_size, ssr_minus1);
    return 0;
}
