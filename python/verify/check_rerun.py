"""Gate G3 — verify a Rerun .rrd reflects the estimated trajectory.

Reads the sensor poses logged in the .rrd and compares them against the TUM
trajectory written by the same run. Pass means the visualization draws exactly
the estimate (final-pose error < tol, equal point count), so the accumulated
map is correct by construction.

Rerun stores translations as float32, so the .rrd differs from the float64 TUM
by the float32 quantization (~1e-5 m at tens of metres). The tolerance is 1 mm:
loose enough to ignore that, tight enough to catch any real divergence.

  python verify/check_rerun.py --rrd results/se3lio.rrd --tum results/traj.tum
"""

import argparse
import sys

import numpy as np

from se3_lio.viz import read_rrd_trajectory


def read_tum_positions(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            t, x, y, z = line.split()[:4]
            rows.append([float(t), float(x), float(y), float(z)])
    arr = np.array(rows)
    return arr[:, 0], arr[:, 1:4]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rrd", required=True)
    ap.add_argument("--tum", required=True)
    ap.add_argument("--tol", type=float, default=1e-3)
    args = ap.parse_args()

    _, rrd_pos = read_rrd_trajectory(args.rrd)
    _, tum_pos = read_tum_positions(args.tum)

    if len(rrd_pos) != len(tum_pos):
        print(f"FAIL: point count differs — rrd {len(rrd_pos)} vs tum {len(tum_pos)}")
        sys.exit(1)

    err = np.linalg.norm(rrd_pos - tum_pos, axis=1)
    final = float(np.linalg.norm(rrd_pos[-1] - tum_pos[-1]))
    print(f"frames={len(rrd_pos)}  max_err={err.max():.3e}  final_err={final:.3e}  tol={args.tol:.1e}")
    if err.max() > args.tol:
        print("FAIL: trajectory mismatch above tolerance")
        sys.exit(1)
    print("PASS: .rrd trajectory matches TUM")


if __name__ == "__main__":
    main()
