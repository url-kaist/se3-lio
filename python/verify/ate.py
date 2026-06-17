"""Compare two TUM trajectories that share the same fixed frame and origin.

No Sim(3) alignment: both come from the same SE3-LIO core starting at identity
in the `map` frame, so poses are directly comparable. Associates by closest
timestamp, then reports translation/rotation error stats.
"""

import argparse

import numpy as np

import tum


def associate(ta, tb, max_dt):
    pairs = []
    j = 0
    for i, t in enumerate(ta):
        while j + 1 < len(tb) and abs(tb[j + 1] - t) <= abs(tb[j] - t):
            j += 1
        if abs(tb[j] - t) <= max_dt:
            pairs.append((i, j))
    return pairs


def quat_angle_deg(qa, qb):
    # angle of relative rotation between two unit quaternions (xyzw)
    dot = np.abs(np.sum(qa * qb, axis=1))
    dot = np.clip(dot, -1.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref", help="reference TUM (node)")
    ap.add_argument("est", help="estimate TUM (python)")
    ap.add_argument("--max-dt", type=float, default=0.02)
    args = ap.parse_args()

    tr, pr, qr = tum.read(args.ref)
    te, pe, qe = tum.read(args.est)
    pairs = associate(tr, te, args.max_dt)
    print(f"ref poses: {len(tr)}   est poses: {len(te)}   associated: {len(pairs)}")
    if not pairs:
        print("NO ASSOCIATIONS — check timestamps/frames")
        return

    ir = [i for i, _ in pairs]
    ie = [j for _, j in pairs]
    dpos = np.linalg.norm(pr[ir] - pe[ie], axis=1)
    dang = quat_angle_deg(qr[ir], qe[ie])
    dt = np.abs(tr[ir] - te[ie])

    print(f"timestamp assoc error: mean={dt.mean()*1e3:.3f} ms  max={dt.max()*1e3:.3f} ms")
    print("translation error [m]:  "
          f"rmse={np.sqrt((dpos**2).mean()):.6f}  mean={dpos.mean():.6f}  "
          f"median={np.median(dpos):.6f}  max={dpos.max():.6f}")
    print("rotation error [deg]:    "
          f"rmse={np.sqrt((dang**2).mean()):.6f}  mean={dang.mean():.6f}  "
          f"median={np.median(dang):.6f}  max={dang.max():.6f}")
    print(f"final ref pos: {pr[ir[-1]]}")
    print(f"final est pos: {pe[ie[-1]]}")


if __name__ == "__main__":
    main()
