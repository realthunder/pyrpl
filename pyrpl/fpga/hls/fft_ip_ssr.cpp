// fft_ip_ssr — SSR FFT using LogiCORE sub-FFTs, Decimation-In-Frequency (DIF) architecture.
// AXI-Stream I/O, magnitude output, bit-reversed bin order.
//
// DIF vs Decimation-In-Time (DIT) decomposition for SSR=2:
//   DIF (this module): the first butterfly stage splits the N-point problem by
//     output bin parity — sub-FFT 0 computes even-indexed bins X[0,2,4,...],
//     sub-FFT 1 computes odd-indexed bins X[1,3,5,...].
//   DIT (Vitis xf::dsp::fft, FFT_IMPL==2): the first stage splits by input
//     sample index — sub-FFT 0 gets even-indexed samples, sub-FFT 1 gets odd.
//     This groups output bins by lower/upper spectrum half instead of even/odd.
//
// Output ordering (drives the BRAM channel/address split in fft_proc.sv):
//   SSR=1: single LogiCORE in bit_reversed_order → beat b = X[bit_rev(b, NFFT)]
//   SSR=2: both sub-FFTs run in bit_reversed_order →
//           beat b = {X[2·bit_rev(b,SUB_NFFT)], X[2·bit_rev(b,SUB_NFFT)+1]}
//   SSR=4: four sub-FFTs in bit_reversed_order →
//           beat b = {X[4·bit_rev(b,SUB_NFFT)], X[4·bit_rev(b,SUB_NFFT)+1],
//                     X[4·bit_rev(b,SUB_NFFT)+2], X[4·bit_rev(b,SUB_NFFT)+3]}
//   DIF lanes hold bins by residue mod FSSR: channel = k[SSR_BITS-1:0].
//   FFT_IMPL==2 (DIT) uses channel = k[MSB] instead. See fft_proc.sv.
//
// Compile flags: -DFFT_SSR=<1|2|4> -DFFT_NFFT=<log2(N)>

#include <hls_stream.h>
#include <ap_axi_sdata.h>
#include <ap_fixed.h>
#include <ap_int.h>
#include <complex>
#include <hls_fft.h>
#include <hls_math.h>

// ============================================================
// Compile-time configuration
// ============================================================

#if FFT_SSR == 1
  #define SSR_LOG2 0
#elif FFT_SSR == 2
  #define SSR_LOG2 1
#elif FFT_SSR == 4
  #define SSR_LOG2 2
#else
  #error "FFT_SSR must be 1, 2, or 4"
#endif

#define FFT_SIZE (1 << FFT_NFFT)
#define SUB_NFFT (FFT_NFFT - SSR_LOG2)   // log2(N/SSR)
#define SUB_SIZE (1 << SUB_NFFT)          // N/SSR samples per serial sub-FFT

static const int ASZ = 14;  // ADC word width (hardware-fixed)
static const int DSZ = 28;  // output magnitude width

// AXI-Stream packing:
//   input:  FSSR ADC samples, each ASZ bits, byte-rounded total
//   output: FSSR magnitudes, each DSZ bits
#define IN_WIDTH  (((ASZ * FFT_SSR + 7) / 8) * 8)
#define OUT_WIDTH (DSZ * FFT_SSR)

typedef ap_axiu<IN_WIDTH,  0, 0, 0> axis_in_t;
typedef ap_axiu<OUT_WIDTH, 0, 0, 0> axis_out_t;

// ============================================================
// Fixed-point types
// ============================================================

static const int INT_W  = 16;  // internal FFT precision
static const int TWID_W = 18;  // twiddle factor precision

typedef std::complex<ap_fixed<INT_W, 1>>   cfixed_t;
typedef std::complex<ap_fixed<TWID_W, 2>> cfixed_twid_t;
typedef ap_fixed<INT_W + TWID_W + 1, 4>   fixed_mul_t;  // scalar multiply accumulator

// ============================================================
// LogiCORE FFT configuration
// ============================================================

// Avnet per-stage scaling schedule: alternating "10"/"01" + "11" final
// Prevents overflow while maximising precision for a given sub-FFT depth.
static constexpr int scale_sched(int n) {
    int ibits = (n + 1) / 2;
    int bits  = (n % 2 == 0) ? 2 : 1;               // "10" or "01"
    for (int i = 1; i < ibits - 1; i++) bits = (bits << 2) | 2; // append "10"
    bits = (bits << 2) | 3;                           // append "11"
    return bits;
}

static const int SCALE_SCHED = scale_sched(SUB_NFFT);

struct fft_params_t : hls::ip_fft::params_t {
    // bit_reversed_order costs no internal reorder buffer in LogiCORE.
    // The output permutation is handled by BRAM addressing in fft_proc.sv.
    static const unsigned ordering_opt       = hls::ip_fft::bit_reversed_order;
    static const unsigned max_nfft           = SUB_NFFT;
    static const unsigned input_width        = INT_W;
    static const unsigned output_width       = INT_W;
    static const unsigned status_width       = 8;
    static const unsigned config_width       = ((2*((SUB_NFFT+1)/2)+1+7)/8)*8;
    static const unsigned phase_factor_width = TWID_W;
    static const unsigned stages_block_ram   = (SUB_NFFT < 10) ? 0 : SUB_NFFT - 9;
};

typedef hls::ip_fft::config_t<fft_params_t> fft_config_t;
typedef hls::ip_fft::status_t<fft_params_t> fft_status_t;

// ============================================================
// SSR >= 2: parallel-sample pair struct
// ============================================================

#if FFT_SSR >= 2
struct par_data {
    cfixed_t data0;
    cfixed_t data1;
};

static cfixed_t cmul(cfixed_t d, cfixed_twid_t w) {
#pragma HLS INLINE
    fixed_mul_t re = d.real()*w.real() - d.imag()*w.imag();
    fixed_mul_t im = d.real()*w.imag() + d.imag()*w.real();
    cfixed_t r;
    r.real(re);
    r.imag(im);
    return r;
}
#endif

#if FFT_SSR == 4
struct par_data4 {
    cfixed_t data[4];
};
#endif

// ============================================================
// Shared helpers
// ============================================================

// DIF twiddle factor W_N^k = exp(-j 2π k/N)
// hls::cos / hls::sin synthesise to a pipelined CORDIC — II=1 in loops.
static inline cfixed_twid_t get_twiddle(int k) {
#pragma HLS INLINE off
    const ap_fixed<24, 1> scale = 2.0f * M_PI / FFT_SIZE;
    ap_fixed<24, 3> phase = (ap_fixed<16, 16>)k * scale;
    cfixed_twid_t w;
    w.real( hls::cos(phase));
    w.imag(-hls::sin(phase));
    return w;
}

// Alpha-max-beta-min magnitude: |z| ≈ max(|re|,|im|) + 3/8·min(|re|,|im|)
// Maps [0,~1.5) input to a DSZ-bit unsigned integer with 2 integer bits.
static inline ap_uint<DSZ> magnitude(cfixed_t v) {
#pragma HLS INLINE
    typedef ap_fixed<INT_W+1, 2> wider_t;
    wider_t re = v.real() < 0 ? (wider_t)(-v.real()) : (wider_t)v.real();
    wider_t im = v.imag() < 0 ? (wider_t)(-v.imag()) : (wider_t)v.imag();
    wider_t mx = (re > im) ? re : im;
    wider_t mn = (re > im) ? im : re;
    wider_t mag = mx + (mn >> 2) + (mn >> 3);
    ap_ufixed<DSZ, 2> out = mag;
    return out.range(DSZ-1, 0);
}

// hls::fft (2020.1) takes C arrays, not streams, so each FFT needs an array feed
// and drain. These run as processes in the TOP-level DATAFLOW region alongside
// hls::fft — matching the canonical Xilinx FFT example's two-level dataflow
// (fft_top → hls::fft). Wrapping them in an intermediate fft_serial() function
// adds a third dataflow level that corrupts hls::fft's internal xn_cp/xk_cp
// buffers (they degrade to ROM/BRAM and the stream-based FFT wrapper is wired to
// memory ports), so the feed/fft/drain are inlined into the top function below.
static void fft_feed(hls::stream<cfixed_t> &in,
                     cfixed_t xn[SUB_SIZE],
                     fft_config_t *cfg) {
    cfg->setDir(1);
    cfg->setSch(SCALE_SCHED);
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        xn[i] = in.read();
    }
}

static void fft_drain(cfixed_t xk[SUB_SIZE],
                      hls::stream<cfixed_t> &out,
                      fft_status_t *sts) {
    (void)*sts;  // overflow status unused
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        out.write(xk[i]);
    }
}

// ============================================================
// SSR = 1  (single serial FFT, no butterfly stage)
// ============================================================
#if FFT_SSR == 1

static void input_ssr1(hls::stream<axis_in_t> &s_axis,
                       hls::stream<cfixed_t>  &out) {
    for (int i = 0; i < FFT_SIZE; i++) {
#pragma HLS PIPELINE II=1
        ap_int<ASZ> raw = s_axis.read().data.range(ASZ-1, 0);
        cfixed_t v;
        v.real(ap_fixed<INT_W,1>(raw));
        v.imag(0);
        out.write(v);
    }
}

static void output_ssr1(hls::stream<cfixed_t>   &in,
                        hls::stream<axis_out_t> &m_axis) {
    for (int i = 0; i < FFT_SIZE; i++) {
#pragma HLS PIPELINE II=1
        axis_out_t pkt;
        pkt.data.range(DSZ-1, 0) = magnitude(in.read());
        pkt.last = (i == FFT_SIZE - 1);
        m_axis.write(pkt);
    }
}

#endif // FFT_SSR == 1

// ============================================================
// SSR = 2  (DIF: stage-1 butterfly + two serial sub-FFTs)
// ============================================================
#if FFT_SSR == 2

// Unpack two ASZ-bit ADC samples per beat → par_data stream
static void input_ssr2(hls::stream<axis_in_t> &s_axis,
                       hls::stream<par_data>  &out) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        auto d = s_axis.read().data;
        par_data t;
        t.data0.real(ap_fixed<INT_W,1>((ap_int<ASZ>)d.range(  ASZ-1,     0)));
        t.data0.imag(0);
        t.data1.real(ap_fixed<INT_W,1>((ap_int<ASZ>)d.range(2*ASZ-1,   ASZ)));
        t.data1.imag(0);
        out.write(t);
    }
}

// Reorder interleaved {x[2k],x[2k+1]} → split {x[k], x[k+N/2]}
// Buffers one full frame (2×SUB_SIZE samples) before outputting.
static void reorder(hls::stream<par_data> &din,
                    hls::stream<par_data> &dout) {
    cfixed_t buff[2][SUB_SIZE];
#pragma HLS ARRAY_PARTITION variable=buff complete dim=1
#pragma HLS ARRAY_PARTITION variable=buff cyclic factor=2 dim=2

    // First N/2 samples → buff[0]
    for (int k = 0; k < SUB_SIZE/2; k++) {
#pragma HLS PIPELINE II=1
        par_data t = din.read();
        buff[0][2*k]   = t.data0;
        buff[0][2*k+1] = t.data1;
    }
    // Second N/2 samples → buff[1]
    for (int k = 0; k < SUB_SIZE/2; k++) {
#pragma HLS PIPELINE II=1
        par_data t = din.read();
        buff[1][2*k]   = t.data0;
        buff[1][2*k+1] = t.data1;
    }
    // Output beat k = { x[k], x[k+N/2] }
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        par_data t;
        t.data0 = buff[0][k];
        t.data1 = buff[1][k];
        dout.write(t);
    }
}

// Stage-1 DIF butterfly: y0=(x0+x1)/2, y1=(x0-x1)/2
static void radix2p(hls::stream<par_data> &din,
                    hls::stream<par_data> &dout) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        par_data t = din.read();
        typedef ap_fixed<INT_W+1, 2> wide_t;
        wide_t a = t.data0.real() + t.data1.real();
        wide_t b = t.data0.imag() + t.data1.imag();
        wide_t c = t.data0.real() - t.data1.real();
        wide_t d = t.data0.imag() - t.data1.imag();
        par_data r;
        r.data0.real(a>>1); r.data0.imag(b>>1);
        r.data1.real(c>>1); r.data1.imag(d>>1);
        dout.write(r);
    }
}

// Twiddle: multiply data1 by W_N^k; pass data0 unchanged
static void twiddle(hls::stream<par_data> &din,
                    hls::stream<cfixed_t> &dout0,
                    hls::stream<cfixed_t> &dout1) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        par_data t = din.read();
        dout0.write(t.data0);
        dout1.write(cmul(t.data1, get_twiddle(i)));
    }
}

// Joiner + magnitude: stream sub-FFT outputs → m_axis
// With bit_reversed_order sub-FFTs:
//   beat b = { |X[2·bit_rev(b,SUB_NFFT)]|, |X[2·bit_rev(b,SUB_NFFT)+1]| }
static void output_ssr2(hls::stream<cfixed_t>   &fft0_out,
                        hls::stream<cfixed_t>   &fft1_out,
                        hls::stream<axis_out_t> &m_axis) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        axis_out_t pkt;
        pkt.data.range(  DSZ-1,   0) = magnitude(fft0_out.read());
        pkt.data.range(2*DSZ-1, DSZ) = magnitude(fft1_out.read());
        pkt.last = (i == SUB_SIZE - 1);
        m_axis.write(pkt);
    }
}

#endif // FFT_SSR == 2

// ============================================================
// SSR = 4  (DIF: two radix-2 stages + four serial sub-FFTs)
// ============================================================
#if FFT_SSR == 4

// Unpack four ASZ-bit ADC samples per beat → par_data4 stream
static void input_ssr4(hls::stream<axis_in_t>  &s_axis,
                       hls::stream<par_data4>  &out) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        auto d = s_axis.read().data;
        par_data4 t;
        for (int q = 0; q < 4; q++) {
#pragma HLS UNROLL
            t.data[q].real(ap_fixed<INT_W,1>((ap_int<ASZ>)d.range((q+1)*ASZ-1, q*ASZ)));
            t.data[q].imag(0);
        }
        out.write(t);
    }
}

// Reorder interleaved {x[4k]..x[4k+3]} → split {x[k], x[k+N/4], x[k+N/2], x[k+3N/4]}
// Input arrives as SUB_SIZE beats of 4 consecutive samples each.
// Reorganised as SUB_SIZE beats where beat k has the four samples needed for the
// radix-4 DIF butterfly: one from each N/4-sample quarter of the frame.
//
// Storage is explicitly banked as buff[quarter][bank][depth] with both leading
// dims fully partitioned. This avoids the cyclic-partition modulo addressing
// (which put a divide on the write critical path) — each of the 16 sub-arrays is
// indexed only by the depth counter on writes.
static void reorder_ssr4(hls::stream<par_data4> &din,
                         hls::stream<par_data4> &dout) {
    static const int DEPTH = SUB_SIZE / 4;
    cfixed_t buff[4][4][DEPTH];
#pragma HLS ARRAY_PARTITION variable=buff complete dim=1
#pragma HLS ARRAY_PARTITION variable=buff complete dim=2

    // Write: beat i belongs to quarter q (DEPTH beats per quarter); its four
    // samples land in the four banks at the same depth d. DEPTH is a power of
    // two, so q and d are bit-slices of i (top 2 bits / low bits) — pure wiring,
    // no induction add/select on the address path.
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        par_data4 t = din.read();
        ap_uint<2> q = i >> (SUB_NFFT - 2);
        int        d = i & (DEPTH - 1);
        for (int b = 0; b < 4; b++) {
#pragma HLS UNROLL
            buff[q][b][d] = t.data[b];
        }
    }
    // Output beat k = {x[k], x[k+N/4], x[k+N/2], x[k+3N/4]}.
    // Position k within a quarter sits at bank k%4, depth k/4 (both power-of-two
    // slices of k — cheap). The bank select is a small 4:1 mux per quarter.
    for (int k = 0; k < SUB_SIZE; k++) {
#pragma HLS PIPELINE II=1
        ap_uint<2>          bank  = k & 0x3;
        int                 depth = k >> 2;
        par_data4 t;
        for (int qq = 0; qq < 4; qq++) {
#pragma HLS UNROLL
            t.data[qq] = buff[qq][bank][depth];
        }
        dout.write(t);
    }
}

// Stage-1 DIF butterfly on pairs (lane0,lane2) and (lane1,lane3).
// Twiddle for lane2: W_N^k.  Twiddle for lane3: W_N^{k+N/4} = W_N^k * (-j).
// Outputs split into two par_data streams: upper={out0,out1}, lower={out2,out3}.
static void stage1_ssr4(hls::stream<par_data4> &din,
                        hls::stream<par_data>  &upper,
                        hls::stream<par_data>  &lower) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        par_data4 t = din.read();
        typedef ap_fixed<INT_W+1, 2> wide_t;

        wide_t a0r = t.data[0].real() + t.data[2].real();
        wide_t a0i = t.data[0].imag() + t.data[2].imag();
        wide_t a1r = t.data[1].real() + t.data[3].real();
        wide_t a1i = t.data[1].imag() + t.data[3].imag();
        wide_t q0r = t.data[0].real() - t.data[2].real();
        wide_t q0i = t.data[0].imag() - t.data[2].imag();
        wide_t q1r = t.data[1].real() - t.data[3].real();
        wide_t q1i = t.data[1].imag() - t.data[3].imag();

        cfixed_t out0, out1, q0, q1;
        out0.real(a0r >> 1); out0.imag(a0i >> 1);
        out1.real(a1r >> 1); out1.imag(a1i >> 1);
        q0.real(q0r >> 1);   q0.imag(q0i >> 1);
        q1.real(q1r >> 1);   q1.imag(q1i >> 1);

        cfixed_twid_t w = get_twiddle(i);
        cfixed_t out2   = cmul(q0, w);
        // W_N^{k+N/4} = W_N^k * (-j): multiply by (-j) rotates (a+jb) → (b-ja)
        cfixed_t q1w    = cmul(q1, w);
        cfixed_t out3;
        out3.real( q1w.imag());
        out3.imag(-q1w.real());

        par_data pu, pl;
        pu.data0 = out0; pu.data1 = out1;
        pl.data0 = out2; pl.data1 = out3;
        upper.write(pu);
        lower.write(pl);
    }
}

// Stage-2 DIF butterfly on the upper half {out0, out1}.
// out0 → sub-FFT for bins ≡ 0 mod 4; out1 → bins ≡ 2 mod 4.
// Twiddle: W_{N/2}^k = W_N^{2k}.
static void stage2_upper(hls::stream<par_data>  &din,
                         hls::stream<cfixed_t>  &fft0_in,
                         hls::stream<cfixed_t>  &fft2_in) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        par_data t = din.read();
        typedef ap_fixed<INT_W+1, 2> wide_t;
        wide_t f0r = t.data0.real() + t.data1.real();
        wide_t f0i = t.data0.imag() + t.data1.imag();
        wide_t  dr = t.data0.real() - t.data1.real();
        wide_t  di = t.data0.imag() - t.data1.imag();
        cfixed_t f0, d;
        f0.real(f0r >> 1); f0.imag(f0i >> 1);
        d.real(dr >> 1);   d.imag(di >> 1);
        fft0_in.write(f0);
        fft2_in.write(cmul(d, get_twiddle(2*i)));
    }
}

// Stage-2 DIF butterfly on the lower half {out2, out3}.
// out2 → sub-FFT for bins ≡ 1 mod 4; out3 → bins ≡ 3 mod 4.
// Twiddle: W_{N/2}^k = W_N^{2k}.
static void stage2_lower(hls::stream<par_data>  &din,
                         hls::stream<cfixed_t>  &fft1_in,
                         hls::stream<cfixed_t>  &fft3_in) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        par_data t = din.read();
        typedef ap_fixed<INT_W+1, 2> wide_t;
        wide_t f1r = t.data0.real() + t.data1.real();
        wide_t f1i = t.data0.imag() + t.data1.imag();
        wide_t  dr = t.data0.real() - t.data1.real();
        wide_t  di = t.data0.imag() - t.data1.imag();
        cfixed_t f1, d;
        f1.real(f1r >> 1); f1.imag(f1i >> 1);
        d.real(dr >> 1);   d.imag(di >> 1);
        fft1_in.write(f1);
        fft3_in.write(cmul(d, get_twiddle(2*i)));
    }
}

// Joiner + magnitude: four sub-FFT outputs → m_axis.
// With bit_reversed_order sub-FFTs:
//   beat b = {|X[4·bit_rev(b,SUB_NFFT)]|, |X[4·bit_rev(b,SUB_NFFT)+1]|,
//              |X[4·bit_rev(b,SUB_NFFT)+2]|, |X[4·bit_rev(b,SUB_NFFT)+3]|}
static void output_ssr4(hls::stream<cfixed_t>   &fft0_out,
                        hls::stream<cfixed_t>   &fft1_out,
                        hls::stream<cfixed_t>   &fft2_out,
                        hls::stream<cfixed_t>   &fft3_out,
                        hls::stream<axis_out_t> &m_axis) {
    for (int i = 0; i < SUB_SIZE; i++) {
#pragma HLS PIPELINE II=1
        axis_out_t pkt;
        pkt.data.range(  DSZ-1,     0) = magnitude(fft0_out.read());
        pkt.data.range(2*DSZ-1,   DSZ) = magnitude(fft1_out.read());
        pkt.data.range(3*DSZ-1, 2*DSZ) = magnitude(fft2_out.read());
        pkt.data.range(4*DSZ-1, 3*DSZ) = magnitude(fft3_out.read());
        pkt.last = (i == SUB_SIZE - 1);
        m_axis.write(pkt);
    }
}

#endif // FFT_SSR == 4

// ============================================================
// Top-level function
// ============================================================

void fft_ip_ssr(hls::stream<axis_in_t>  &s_axis,
                hls::stream<axis_out_t> &m_axis,
                bool                    &event_frame_started)
{
#pragma HLS INTERFACE axis port=s_axis
#pragma HLS INTERFACE axis port=m_axis
#pragma HLS INTERFACE ap_none port=event_frame_started
#pragma HLS INTERFACE ap_ctrl_none port=return

    event_frame_started = true;

#pragma HLS DATAFLOW

#if FFT_SSR == 1

    hls::stream<cfixed_t> pre_fft("pre_fft");
    hls::stream<cfixed_t> post_fft("post_fft");
#pragma HLS STREAM variable=pre_fft  depth=16
#pragma HLS STREAM variable=post_fft depth=16
    cfixed_t     xn[SUB_SIZE];
    cfixed_t     xk[SUB_SIZE];
    fft_config_t cfg;
    fft_status_t sts;

    input_ssr1 (s_axis, pre_fft);
    fft_feed   (pre_fft, xn, &cfg);
    hls::fft<fft_params_t>(xn, xk, &sts, &cfg);
    fft_drain  (xk, post_fft, &sts);
    output_ssr1(post_fft, m_axis);

#elif FFT_SSR == 2

    hls::stream<par_data>  raw_par    ("raw_par");
    hls::stream<par_data>  reorder_out("reorder_out");
    hls::stream<par_data>  radix_out  ("radix_out");
    hls::stream<cfixed_t>  tw_out0    ("tw_out0");
    hls::stream<cfixed_t>  tw_out1    ("tw_out1");
    hls::stream<cfixed_t>  fft0_out   ("fft0_out");
    hls::stream<cfixed_t>  fft1_out   ("fft1_out");
#pragma HLS STREAM variable=raw_par      depth=16
#pragma HLS STREAM variable=reorder_out  depth=16
#pragma HLS STREAM variable=radix_out    depth=16
#pragma HLS STREAM variable=tw_out0      depth=16
#pragma HLS STREAM variable=tw_out1      depth=16
#pragma HLS STREAM variable=fft0_out     depth=16
#pragma HLS STREAM variable=fft1_out     depth=16
    cfixed_t     xn0[SUB_SIZE], xk0[SUB_SIZE];
    cfixed_t     xn1[SUB_SIZE], xk1[SUB_SIZE];
    fft_config_t cfg0, cfg1;
    fft_status_t sts0, sts1;

    input_ssr2 (s_axis, raw_par);
    reorder    (raw_par, reorder_out);
    radix2p    (reorder_out, radix_out);
    twiddle    (radix_out, tw_out0, tw_out1);
    fft_feed   (tw_out0, xn0, &cfg0);
    hls::fft<fft_params_t>(xn0, xk0, &sts0, &cfg0);
    fft_drain  (xk0, fft0_out, &sts0);
    fft_feed   (tw_out1, xn1, &cfg1);
    hls::fft<fft_params_t>(xn1, xk1, &sts1, &cfg1);
    fft_drain  (xk1, fft1_out, &sts1);
    output_ssr2(fft0_out, fft1_out, m_axis);

#elif FFT_SSR == 4

    hls::stream<par_data4>  raw_par4    ("raw_par4");
    hls::stream<par_data4>  reorder_out ("reorder_out");
    hls::stream<par_data>   upper       ("upper");
    hls::stream<par_data>   lower       ("lower");
    hls::stream<cfixed_t>   fft0_in     ("fft0_in");
    hls::stream<cfixed_t>   fft1_in     ("fft1_in");
    hls::stream<cfixed_t>   fft2_in     ("fft2_in");
    hls::stream<cfixed_t>   fft3_in     ("fft3_in");
    hls::stream<cfixed_t>   fft0_out    ("fft0_out");
    hls::stream<cfixed_t>   fft1_out    ("fft1_out");
    hls::stream<cfixed_t>   fft2_out    ("fft2_out");
    hls::stream<cfixed_t>   fft3_out    ("fft3_out");
#pragma HLS STREAM variable=raw_par4    depth=16
#pragma HLS STREAM variable=reorder_out depth=16
#pragma HLS STREAM variable=upper       depth=16
#pragma HLS STREAM variable=lower       depth=16
#pragma HLS STREAM variable=fft0_in     depth=16
#pragma HLS STREAM variable=fft1_in     depth=16
#pragma HLS STREAM variable=fft2_in     depth=16
#pragma HLS STREAM variable=fft3_in     depth=16
#pragma HLS STREAM variable=fft0_out    depth=16
#pragma HLS STREAM variable=fft1_out    depth=16
#pragma HLS STREAM variable=fft2_out    depth=16
#pragma HLS STREAM variable=fft3_out    depth=16
    cfixed_t     xn0[SUB_SIZE], xk0[SUB_SIZE];
    cfixed_t     xn1[SUB_SIZE], xk1[SUB_SIZE];
    cfixed_t     xn2[SUB_SIZE], xk2[SUB_SIZE];
    cfixed_t     xn3[SUB_SIZE], xk3[SUB_SIZE];
    fft_config_t cfg0, cfg1, cfg2, cfg3;
    fft_status_t sts0, sts1, sts2, sts3;

    input_ssr4  (s_axis, raw_par4);
    reorder_ssr4(raw_par4, reorder_out);
    stage1_ssr4 (reorder_out, upper, lower);
    stage2_upper(upper, fft0_in, fft2_in);
    stage2_lower(lower, fft1_in, fft3_in);
    fft_feed (fft0_in, xn0, &cfg0);
    hls::fft<fft_params_t>(xn0, xk0, &sts0, &cfg0);
    fft_drain(xk0, fft0_out, &sts0);
    fft_feed (fft1_in, xn1, &cfg1);
    hls::fft<fft_params_t>(xn1, xk1, &sts1, &cfg1);
    fft_drain(xk1, fft1_out, &sts1);
    fft_feed (fft2_in, xn2, &cfg2);
    hls::fft<fft_params_t>(xn2, xk2, &sts2, &cfg2);
    fft_drain(xk2, fft2_out, &sts2);
    fft_feed (fft3_in, xn3, &cfg3);
    hls::fft<fft_params_t>(xn3, xk3, &sts3, &cfg3);
    fft_drain(xk3, fft3_out, &sts3);
    output_ssr4 (fft0_out, fft1_out, fft2_out, fft3_out, m_axis);

#endif
}
