#!/usr/bin/env python3
"""eye_report.py (v8) - how well did the camera eye see the worker in a run?

    python3 ~/ros2_ws/src/tools/eye_report.py <label>

Reads ~/runs/eye/<label>.csv (written by eye.py) and prints, for the thesis:
  - speed: frames per second, detector time on the Mac, round trip from the VM, how fast the
    simulation ran;
  - detection: of the frames where the worker was in the camera's view (inside the 90 deg,
    0.8-8 m) and the lidar saw her (so nothing hid her), the share where the camera found a
    person, and where camera and lidar agreed on one; by distance;
  - accuracy: fused position against where the script put her, and the camera's bearing;
  - mistakes: people "seen" more than 1 m from her, or while she was out of view;
  - what else YOLO saw in the warehouse.
Nothing is changed or written.
"""
import csv
import os
import sys
from collections import Counter

RUNS = os.path.expanduser('~/runs')


def fnum(v, default=float('nan')):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def pct(n, d):
    return f'{100.0 * n / d:5.1f} %' if d else '   -  '


def quant(xs, q):
    xs = sorted(x for x in xs if x == x)
    if not xs:
        return float('nan')
    k = (len(xs) - 1) * q
    i, f = int(k), k - int(k)
    return xs[i] if i + 1 >= len(xs) else xs[i] * (1 - f) + xs[i + 1] * f


def report(rows, out=print):
    n = len(rows)
    if not n:
        out('no frames in the log')
        return 1
    ok = [r for r in rows if r['ok'] == '1']
    t = [fnum(r['t']) for r in rows]
    wall = [fnum(r['wall']) for r in rows]
    dur_w, dur_s = wall[-1] - wall[0], t[-1] - t[0]
    out(f'--- speed  ({n} frames over {dur_w / 60:.1f} min of wall time)')
    out(f'  frames processed      {n / dur_w if dur_w > 0 else float("nan"):.1f} per second; detector answered '
        f'{pct(len(ok), n)} of them')
    dms = [fnum(r['det_ms']) for r in ok]
    tms = [fnum(r['trip_ms']) for r in ok]
    out(f'  YOLO on the Mac       median {quant(dms, 0.5):.0f} ms, 90 % under {quant(dms, 0.9):.0f} ms')
    out(f'  round trip from VM    median {quant(tms, 0.5):.0f} ms, 90 % under {quant(tms, 0.9):.0f} ms '
        f'(JPEG + network + YOLO)')
    out(f'  simulation speed      {dur_s / dur_w if dur_w > 0 else float("nan"):.2f} x real time while the eye ran')
    out('')
    vis = [r for r in ok if r['truth_in_view'] == '1' and r['truth_lidar'] == '1']
    found = [r for r in vis if int(r['persons'] or 0) > 0]
    fused = [r for r in vis if int(r['fused'] or 0) > 0]
    out('--- detection  (frames where she was in the camera view, 0.8-8 m, and the lidar saw her)')
    out(f'  frames                {len(vis)}')
    out(f'  camera found a person {pct(len(found), len(vis))}')
    out(f'  camera + lidar agreed {pct(len(fused), len(vis))}  (position from the lidar, identity from the camera)')
    out('  by distance           ' + '   '.join(
        f'{lo:.0f}-{hi:.0f} m: {pct(sum(1 for r in vis if lo <= fnum(r["truth_range"]) < hi and int(r["persons"] or 0) > 0), sum(1 for r in vis if lo <= fnum(r["truth_range"]) < hi)).strip()} '
        f'of {sum(1 for r in vis if lo <= fnum(r["truth_range"]) < hi)}'
        for lo, hi in ((0.8, 2.0), (2.0, 4.0), (4.0, 6.0), (6.0, 8.0))))
    confs = [fnum(r['p_conf']) for r in found]
    out(f'  confidence            median {quant(confs, 0.5):.2f}, lowest 10 % below {quant(confs, 0.1):.2f}')
    out('')
    errs = [fnum(r['err']) for r in fused if fnum(r['err']) == fnum(r['err'])]
    good = [e for e in errs if e <= 1.0]
    berr = [fnum(r['bearing_err']) for r in found]
    out('--- accuracy  (against where the worker script put her)')
    out(f'  fused position        median {quant(good, 0.5):.2f} m, 90 % within {quant(good, 0.9):.2f} m '
        f'({len(good)} frames)')
    out(f'  camera bearing        median {quant(berr, 0.5):.1f} deg, 90 % within {quant(berr, 0.9):.1f} deg')
    feet = [fnum(r['p_feet']) for r in fused if fnum(r['p_feet']) == fnum(r['p_feet'])]
    out(f'  feet check            box bottom / floor row at the lidar depth: median {quant(feet, 0.5):.2f} '
        f'(1.00 = consistent)')
    out('')
    far = [e for e in errs if e > 1.0]
    away = [r for r in ok if r['truth_in_view'] == '0' and int(r['persons'] or 0) > 0]
    cam_only = sum(int(r['cam_only'] or 0) for r in ok)
    out('--- mistakes')
    out(f'  fused person > 1 m from her            {len(far)} frames ({pct(len(far), len(errs)).strip()} of fused)')
    out(f'  person seen while she was out of view  {len(away)} frames ({pct(len(away), len(ok)).strip()} of all)')
    out(f'  person boxes the lidar could not place {cam_only} (camera only)')
    hidden = [r for r in ok if r['truth_in_view'] == '1' and r['truth_lidar'] != '1']
    out(f'  frames with her in view but hidden from the lidar: {len(hidden)}, camera found her in '
        f'{pct(sum(1 for r in hidden if int(r["persons"] or 0) > 0), len(hidden)).strip()}')
    other = Counter()
    for r in ok:
        for item in filter(None, (r.get('others') or '').split(';')):
            other[item.split(':')[0]] += 1
    out('')
    out('--- other objects YOLO reported (frames): ' + (', '.join(f'{k} {v}' for k, v in other.most_common(8)) or 'none'))
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print('usage: eye_report.py <label>')
        return 1
    path = os.path.join(RUNS, 'eye', f'{argv[0]}.csv')
    if not os.path.exists(path):
        print(f'no eye log {path} (was eye.py running with this label?)')
        return 1
    rows = list(csv.DictReader(open(path)))
    print(f'=== eye_report {argv[0]}: {len(rows)} frames from {path}')
    print()
    return report(rows)


if __name__ == '__main__':
    sys.exit(main())
