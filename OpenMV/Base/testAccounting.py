#!/usr/bin/env python3
"""testAccounting.py - invariants for syncServer.py's packet accounting.

    python3 Base/testAccounting.py

These four cases are the ones that silently corrupt a drop report rather than
crashing, and none of them can be reached in a short live run:

  1. The 16-bit seq wrap. At 200 fps it arrives every 5.5 minutes.
  2. The 32-bit t_us wrap. Every 71.6 minutes.
  3. A reordered packet. Its timestamp goes BACKWARDS, which an over-eager
     reboot detector reads as a restart - and a restart clears the gap
     bookkeeping, so the losses being measured vanish. This one was a real bug.
  4. A genuine reboot, which must be distinguished from case 1 even though
     both jump the sequence number backwards by tens of thousands.
"""

import os
import shutil
import struct
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import syncServer as S

PER = 5000          # 200 fps


def pkt(seq, tick, cam=1, init=0):
    return struct.pack('<BBBBHI' + 'fffB', S.PKT_MAGIC, cam, init, 1,
                       seq & 0xFFFF, tick & 0xFFFFFFFF, 1.0, 2.0, 0.05, 1)


def check(name, got, want):
    ok = got == want
    print('  %-46s %s' % (name, 'PASS' if ok else 'FAIL  got %r want %r'
                          % (got, want)))
    return ok


def main():
    tmp = tempfile.mkdtemp()
    try:
        st = S.CamStats(1, tmp)
        seq0 = 65000                       # crosses 65536 mid-stream
        tick0 = (1 << 32) - 400 * PER      # crosses 2^32 mid-stream
        ref0 = 10 ** 9
        n = 2000
        drop_a = set(range(300, 305))      # burst of 5
        drop_b = set(range(900, 912))      # burst of 12
        reorder_at = 1500
        held = None
        for i in range(n):
            if i in drop_a or i in drop_b:
                continue
            ref = ref0 + i * PER
            if i == reorder_at:
                held = pkt(seq0 + i, tick0 + i * PER)
                continue
            st.on_packet(pkt(seq0 + i, tick0 + i * PER), ref, 0)
            if i == reorder_at + 1 and held:
                st.on_packet(held, ref + 200, 0)
                held = None
        st.reap(ref0 + n * PER + 10 ** 6)

        ok = True
        print('continuous stream across both wraps:')
        ok &= check('packets received', st.received, n - 17)
        ok &= check('losses charged', st.lost, 17)
        ok &= check('loss runs grouped, not counted singly',
                    st.run_hist, {5: 1, 12: 1})
        ok &= check('reorder counted as reorder, not loss', st.reordered, 1)
        ok &= check('no duplicates invented', st.duplicated, 0)
        ok &= check('seq/tick wrap not mistaken for reboot', st.reboots, 0)

        print('genuine reboot (seq and tick both restart at 0):')
        ref = ref0 + n * PER + 3 * 10 ** 6
        for i in range(50):
            st.on_packet(pkt(i, i * PER), ref + i * PER, 0)
        st.reap(ref + 50 * PER + 10 ** 6)
        ok &= check('restart detected', st.reboots, 1)
        ok &= check('new segment opened', st.seg, 1)
        ok &= check('pre-reboot losses not double-charged', st.lost, 17)
        st.close()

        print('\n%s' % ('ALL PASS' if ok else 'FAILURES ABOVE'))
        return 0 if ok else 1
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
