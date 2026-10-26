"""Build system.dtb for the ZCU216 SD card: the device tree inside the stock
image.ub (PetaLinux 2020.1 evaluation image) plus reserved-memory.dtsi.

    python3 mkdtb.py <image.ub> <system.dtb>

boot.cmd boots the kernel and ramdisk from image.ub with this tree instead of
the one built into it, so image.ub itself stays the stock file. Needs dtc.
"""
import os, struct, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def fit_fdt(fit):
    """Return the data of the first flat_dt image in a FIT (itself an FDT)."""
    magic, _, off_struct, off_strings = struct.unpack('>4I', fit[:16])
    assert magic == 0xd00dfeed, 'not a FIT image'
    strs = fit[off_strings:]
    p, path, props = off_struct, [], {}
    while True:
        tok = struct.unpack('>I', fit[p:p + 4])[0]
        p += 4
        if tok == 1:                                  # FDT_BEGIN_NODE
            e = fit.index(b'\0', p)
            path.append(fit[p:e].decode())
            p = (e + 4) & ~3
        elif tok == 2:                                # FDT_END_NODE
            node = '/'.join(path)
            if props.get(node, {}).get('type') == b'flat_dt\0':
                return props[node]['data']
            path.pop()
        elif tok == 3:                                # FDT_PROP
            ln, no = struct.unpack('>II', fit[p:p + 8])
            p += 8
            name = strs[no:strs.index(b'\0', no)].decode()
            props.setdefault('/'.join(path), {})[name] = fit[p:p + ln]
            p = (p + ln + 3) & ~3
        elif tok == 9:                                # FDT_END
            raise SystemExit('no flat_dt image in the FIT')


def main(image_ub, out_dtb):
    dtb = fit_fdt(open(image_ub, 'rb').read())
    with tempfile.TemporaryDirectory() as tmp:
        stock = os.path.join(tmp, 'stock.dtb')
        open(stock, 'wb').write(dtb)
        dts = subprocess.run(['dtc', '-q', '-I', 'dtb', '-O', 'dts', stock],
                             check=True, capture_output=True, text=True).stdout
        if 'reserved-memory' in dts:
            raise SystemExit('stock tree already has reserved-memory; merge by hand')
        extra = open(os.path.join(HERE, 'reserved-memory.dtsi')).read()
        src = os.path.join(tmp, 'system.dts')
        open(src, 'w').write(dts + '\n' + extra)
        subprocess.run(['dtc', '-q', '-I', 'dts', '-O', 'dtb', '-o', out_dtb, src],
                       check=True)
    print('%s: %d bytes (stock tree %d bytes)' % (out_dtb, os.path.getsize(out_dtb), len(dtb)))


if __name__ == '__main__':
    main(*sys.argv[1:3])
