#!/usr/bin/env python3
"""checks the outputs of tb_rx.sv: python3 tb_rx.py rx.txt"""
import sys
import numpy as np

d = np.loadtxt(sys.argv[1], dtype=int)
BIN = 250e6 / 4096
A = 5000.
errs = 0


def spec(x):
    """amplitudes by bin of N real or complex samples"""
    return np.fft.fft(x) / len(x)


def check(name, ok, msg):
    global errs
    print('RX %-24s %s %s' % (name, msg, '' if ok else 'ERR'))
    errs += 0 if ok else 1


def real_case(i, name, b, a=A):
    x = d[d[:, 0] == i, 1].astype(float)
    S = np.abs(spec(x))[:2049] * 2
    S[0] /= 2
    k = int(np.argmax(S))
    rest = np.delete(S, k).max()
    got = S[k]
    check(name, k == b and abs(got - a) < 0.02 * a,
          'bin %d (want %d), amp %.1f (want %.0f), next %.1f dB' % (
              k, b, got, a, 20 * np.log10(max(rest, 1e-3) / got)))


def reject(i, name, want_db):
    x = d[d[:, 0] == i, 1].astype(float)
    S = np.abs(spec(x))[1:2049] * 2
    db = 20 * np.log10(max(S.max(), 1e-3) / A)
    check(name, db < want_db, 'worst %.1f dB (want < %.0f)' % (db, want_db))


def complex_case(i, name, b):
    r = d[d[:, 0] == i]
    z = r[:, 1] + 1j * r[:, 2]
    S = np.abs(spec(z))
    k = int(np.argmax(S))
    k = k - 4096 if k >= 2048 else k
    img = S[-k] / S[k]
    check(name, k == b and abs(S[k] - A) < 0.02 * A,
          'bin %d (want %d), amp %.1f, image %.1f dB' % (
              k, b, S[k], 20 * np.log10(max(img, 1e-6))))


real_case(0, 'low-pass 30 MHz', 492)
real_case(1, 'low-pass 99 MHz', 1622)
reject(2, 'low-pass 170 MHz alias', -50)
complex_case(3, 'I, Q +23 MHz', 377)
complex_case(4, 'I, Q -23 MHz', -377)
real_case(5, 'real 62.5 + 23 MHz', 1024 + 377)
real_case(6, 'real 62.5 - 23 MHz', 1024 - 377)
reject(7, 'real, 160 MHz above', -70)
real_case(8, 'real, ADC channel 2', 1024 + 377)
real_case(9, 'low-pass after a band', 492)
r = d[d[:, 0] == 9]
check('low-pass after: Q is 0', abs(r[:, 2]).max() == 0, 'max |Q| %d' % abs(r[:, 2]).max())
print('RX checks done, %d errors' % errs)
