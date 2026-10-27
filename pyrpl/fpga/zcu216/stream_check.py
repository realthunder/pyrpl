#!/usr/bin/env python3
"""All three ZCU216 streams at once, unsynchronised, as they will be used:

  - the oscilloscope through pyrpl (board='zcu216'): triggered traces at the
    smallest decimation, then a free run (the rolling-mode data path) read
    while it writes; every trace is checked with a sine fit at the loopback
    tone, since a lost word is a phase jump;
  - frame snapshots (snap.py), captured and fetched back to back;
  - the point stream (points.py), counting gaps and drops.

Run rf_loopback.py first (DAC tone on bin --bin into ADC channel 0).

  python3 zcu216/stream_check.py [--host 192.168.1.216] [--seconds 20]
"""
import argparse, os, subprocess, sys, time
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..', '..', '..')))


def sine_fit(y, fs):
    """residual rms, frequency and amplitude of the best sine fit (the FFT
    peak, refined over +-1 bin)"""
    n = y.size
    k = int(np.argmax(np.abs(np.fft.rfft(y * np.hanning(n)))[1:])) + 1
    t = np.arange(n) / fs
    best = None
    for f in k * fs / n + np.linspace(-fs / n, fs / n, 41):
        A = np.c_[np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t), np.ones(n)]
        c, *_ = np.linalg.lstsq(A, y, rcond=None)
        r = (y - A @ c).std()
        if best is None or r < best[0]:
            best = (r, f, np.hypot(c[0], c[1]))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.1.216')
    ap.add_argument('--seconds', type=float, default=20)
    ap.add_argument('--input', default='adc0', help='scope channel 1 (the loopback ADC)')
    a = ap.parse_args()
    T = a.seconds

    procs = {
        'points': subprocess.Popen(
            [sys.executable, 'points.py', '--host', a.host, '--seconds', str(T)],
            cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True),
        'snapshots': subprocess.Popen(
            ['bash', '-c', f'end=$((SECONDS + {int(T)})); n=0; '
                           f'while [ $SECONDS -lt $end ]; do '
                           f'{sys.executable} snap.py --host {a.host} --frames 30 | grep -E "beats|WARN|Error"; '
                           f'n=$((n+1)); done; echo "captures: $n"'],
            cwd=HERE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)}

    from pyrpl import Pyrpl
    p = Pyrpl(config='', hostname=a.host, board='zcu216', gui=False)
    s = p.rp.scope
    dmin = list(type(s).decimation.options(s))[0]
    t_end = time.time() + T

    # triggered traces at the smallest decimation, for the first 2/3 of the time
    s.setup(input1=a.input, input2='adc1', trigger_source='ch1_positive_edge',
            threshold=0.0, hysteresis=0.01, trigger_delay=0, average=False,
            rolling_mode=False, duration=s.data_length * s._base_sampling_time * dmin)
    fs = 1 / s.sampling_time
    n = bad = 0
    res = None
    while time.time() < t_end - T / 3:
        tr = s.single()
        st = s._ring_status
        res = sine_fit(tr[0], fs)
        n += 1
        if st or res[0] > 0.05 * res[2]:
            bad += 1
            print(f'  trace {n}: ring status {st}, {res[1] / 1e6:.3f} MHz, '
                  f'amplitude {res[2]:.4f}, residual {res[0]:.4f}')
    print(f'scope, triggered: {n} traces of {s.data_length} samples at decimation '
          f'{s.decimation} ({fs / 1e6:.0f} MS/s), {bad} bad', end='')
    if res:
        print(f'; last: {res[1] / 1e6:.3f} MHz, amplitude {res[2]:.4f}, residual '
              f'{res[0]:.5f} ({20 * np.log10(res[2] / max(res[0], 1e-9)):.1f} dB)')
    else:
        print()

    # a free run at twice the smallest decimation, read while it writes
    s.setup(rolling_mode=True, trigger_source='off',
            duration=s.data_length * s._base_sampling_time * 2 * dmin)
    s._start_acquisition_rolling_mode()
    nr = rb = 0
    while time.time() < t_end:
        time.sleep(0.05)
        _, d = s._get_rolling_curve()
        r = sine_fit(d[0], 1 / s.sampling_time)
        nr += 1
        if r[0] > 0.05 * r[2]:
            rb += 1
    st = s._ring_status
    s._reset_writestate_machine = True
    print(f'scope, free run: {nr} reads at decimation {s.decimation}, {rb} bad, '
          f'ring status {st}')

    for name, pr in procs.items():
        out = (pr.communicate()[0] or '').strip().splitlines()
        print(f'{name}:')
        for line in out[-4:]:
            print('  ' + line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
