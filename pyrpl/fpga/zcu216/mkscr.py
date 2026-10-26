"""Encode/decode a U-Boot legacy script image, as mkimage -A arm -T script -C none
would (no mkimage on the hosts). python3 mkscr.py <boot.scr> checks a round
trip; python3 mkscr.py --encode <boot.cmd> <boot.scr> writes one."""
import struct, sys, zlib

def decode(b):
    hdr = struct.unpack('>7I4B32s', b[:64])
    magic, hcrc, tm, size, load, ep, dcrc, os_, arch, typ, comp, name = hdr
    assert magic == 0x27051956 and typ == 6, hex(magic)
    data = b[64:64 + size]
    assert zlib.crc32(data) == dcrc, 'data crc'
    h0 = b[:4] + b'\0' * 4 + b[8:64]
    assert zlib.crc32(h0) == hcrc, 'header crc'
    ln = struct.unpack('>I', data[:4])[0]
    assert data[4:8] == b'\0' * 4
    return dict(time=tm, os=os_, arch=arch, comp=comp, name=name, load=load, ep=ep), data[8:8 + ln], data

def encode(script, time, os_=5, arch=2, name=b'', load=0, ep=0):
    data = struct.pack('>II', len(script), 0) + script
    h = struct.pack('>7I4B32s', 0x27051956, 0, time, len(data), load, ep,
                    zlib.crc32(data), os_, arch, 6, 0, name.ljust(32, b'\0')[:32])
    h = h[:4] + struct.pack('>I', zlib.crc32(h)) + h[8:]
    return h + data

if __name__ == '__main__':
    if sys.argv[1] == '--encode':
        script = open(sys.argv[2], 'rb').read()
        open(sys.argv[3], 'wb').write(encode(script, 0, name=b'Boot script'))
        sys.exit(0)
    orig = open(sys.argv[1], 'rb').read()
    meta, script, data = decode(orig)
    print('meta', meta, 'script', len(script), 'bytes, file', len(orig))
    re = encode(script, meta['time'], meta['os'], meta['arch'], meta['name'].rstrip(b'\0'), meta['load'], meta['ep'])
    print('round trip identical:', re == orig, len(re))
