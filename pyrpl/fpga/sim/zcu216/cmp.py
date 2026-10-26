"""Compare tb_dit8 output (out.txt: frame bin re im) with numpy FFTs of the
accepted input samples (in.txt: frame sample). Usage: cmp.py <dir> <N> [scale]
(scale = 2^cut when the sub-FFT outputs are cut before the twiddles)."""
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
