"""Compare tb_dit8 output with numpy FFTs of the accepted input samples
(in.txt: frame sample):
  out.txt  frame bin re im   complex bins, bit-reversed order (zcu216_fft_dit8)
  nat.txt  frame bin mag     K|X|/2 after zcu216_fft_mag + zcu216_fft_reorder,
                             which must come in natural order 0, 1, 2, ...
Usage: cmp.py <dir> <N> [scale]   (scale = 2^cut of the sub-FFT outputs)."""
import numpy as np, sys
d = sys.argv[1]; N = int(sys.argv[2]); sc = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
x = np.loadtxt(d + '/in.txt'); o = np.loadtxt(d + '/out.txt')
for f in range(int(o[:, 0].max()) + 1):
    xs = x[x[:, 0] == f][:, 1]
    r = o[o[:, 0] == f]
    if len(xs) != N or len(r) != N // 2:
        print(f"frame {f}: {len(xs)} input samples, {len(r)} output bins (incomplete)"); continue
    X = np.zeros(N // 2, complex); X[r[:, 1].astype(int)] = r[:, 2] + 1j * r[:, 3]
    ref = np.fft.fft(xs)[:N // 2] / sc
    err = np.abs(X - ref)
    print(f"frame {f}: bins {len(set(r[:,1].astype(int)))}/{N//2}  max|X| {np.abs(ref).max():10.4g}"
          f"  max|err| {err.max():8.3g}  rms err {np.sqrt((err**2).mean()):8.3g}"
          f"  rel {err.max()/max(np.abs(ref).max(),1):.2e}")

K = 1.6467602581210656
nat = np.loadtxt(d + '/nat.txt')
for f in range(int(nat[:, 0].max()) + 1):
    xs = x[x[:, 0] == f][:, 1]
    r = nat[nat[:, 0] == f]
    if len(xs) != N or len(r) != N // 2:
        print(f"nat frame {f}: {len(xs)} input samples, {len(r)} bins (incomplete)"); continue
    order_ok = np.array_equal(r[:, 1].astype(int), np.arange(N // 2))
    ref = K * np.abs(np.fft.fft(xs)[:N // 2]) / sc / 2
    err = np.abs(r[:, 2] - ref)
    print(f"nat frame {f}: natural order {'OK' if order_ok else 'WRONG'}  max mag {ref.max():10.4g}"
          f"  max|err| {err.max():8.3g}  rms err {np.sqrt((err**2).mean()):8.3g}")
