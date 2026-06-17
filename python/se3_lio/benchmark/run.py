"""Benchmark runner: `python -m se3_lio.benchmark.run <dataset>`.

Expands a grid sweep, runs SE(3)-LIO over the dataset frames for each combo,
evaluates rigid-aligned ATE against ground truth, and writes scores.csv +
best.yaml. The compiled binding is imported lazily inside the functions that
actually execute a sweep, so registry/search/evaluate stay host-importable.
"""

import argparse
import copy
import csv
import os
import tempfile
from multiprocessing import Pool
from pathlib import Path

import yaml

from se3_lio.benchmark import evaluate as ev
from se3_lio.benchmark.registry import load_registry, resolve
from se3_lio.benchmark.search import expand

# Per-worker cache, populated by the Pool initializer so the (large) frame
# arrays are read once and never pickled per task.
_WORKER = {}


def _apply_overrides(raw, overrides):
    """Apply dotted-key overrides onto a (deep-copied) params dict in place."""
    merged = copy.deepcopy(raw)
    node = merged.get("/**", merged)
    p = node.get("ros__parameters", node)
    for dotted, value in overrides.items():
        cur = p
        keys = dotted.split(".")
        for k in keys[:-1]:
            cur = cur.setdefault(k, {})
        cur[keys[-1]] = value
    return merged


def _config_from_overrides(base_params, overrides):
    """Build an SE3LIOConfig (+extrinsic etc.) with dotted overrides applied.

    Writes the merged params dict to a temp yaml and reuses load_node_params so
    the node's exact key mapping is preserved.
    """
    from se3_lio.config import load_node_params

    with open(base_params) as f:
        raw = yaml.safe_load(f)
    merged = _apply_overrides(raw, overrides)
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tf:
        yaml.safe_dump(merged, tf)
        tmp = tf.name
    try:
        return load_node_params(tmp)
    finally:
        os.unlink(tmp)


def _make_dataset(input_type, bag, imu_topic, lidar_topic, min_range, max_frames):
    from se3_lio.datasets import Ros1BagDataset, RosbagDataset

    args = (bag, imu_topic, lidar_topic, min_range, max_frames)
    if input_type == "ros1-ouster":
        return Ros1BagDataset(*args)
    return RosbagDataset(*args)


def _load_frames(entry):
    """Read the bag once and materialize the synchronized frame list."""
    p = resolve(entry)
    ds = _make_dataset(
        p["input_type"], p["bag"], p["imu_topic"], p["lidar_topic"],
        p["min_range"], p["max_frames"],
    )
    from se3_lio.datasets.rosbag import Frame

    frames = [
        Frame(f.points, f.point_times, f.imu, f.stamp) for f in ds
    ]
    return frames, p


def _worker_init(entry, gt_tum, out_dir):
    frames, p = _load_frames(entry)
    _WORKER["frames"] = frames
    _WORKER["extrinsic"] = p["extrinsic"]
    _WORKER["base_params"] = p["base_params"]
    _WORKER["gt_tum"] = gt_tum
    _WORKER["out_dir"] = out_dir


def _run_combo(args):
    """Run one override combo over the cached frames -> result row."""
    from se3_lio.pipeline import OdometryPipeline

    combo_id, overrides = args
    p = _config_from_overrides(_WORKER["base_params"], overrides)
    pipe = OdometryPipeline(_WORKER["frames"], p["config"], _WORKER["extrinsic"])
    pipe.run(progress=False)
    tum_path = Path(_WORKER["out_dir"]) / f"combo_{combo_id:04d}.tum"
    pipe.save_tum(tum_path)
    metrics = ev.evaluate(str(tum_path), _WORKER["gt_tum"])
    row = {"combo_id": combo_id}
    row.update(overrides)
    row.update(metrics)
    return row


_METRIC_KEYS = [
    "ate_rmse", "ate_mean", "ate_median", "ate_max",
    "rot_rmse_deg", "n_assoc", "n_est", "n_gt",
]


def _write_scores(rows, override_keys, path):
    fields = ["combo_id"] + override_keys + _METRIC_KEYS
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fields})


def _verify_only(entry, gt_tum, out_dir):
    from se3_lio.pipeline import OdometryPipeline

    frames, p = _load_frames(entry)
    print(f"synchronized {len(frames)} frames")
    pipe = OdometryPipeline(frames, p["config"], p["extrinsic"])
    pipe.run(progress=True)
    tum_path = Path(out_dir) / "verify.tum"
    pipe.save_tum(tum_path)
    metrics = ev.evaluate(str(tum_path), gt_tum)
    print(pipe.summary())
    print(f"trajectory -> {tum_path}")
    for k in _METRIC_KEYS:
        print(f"  {k}: {metrics[k]}")
    return metrics


def main(argv=None):
    ap = argparse.ArgumentParser(prog="se3_lio.benchmark.run")
    ap.add_argument("dataset", help="key into the registry yaml")
    ap.add_argument("--config", default="benchmark/datasets.yaml",
                    help="dataset registry yaml")
    ap.add_argument("--sweep", default=None,
                    help="sweep yaml (default benchmark/sweeps/<dataset>.yaml)")
    ap.add_argument("--verify-only", action="store_true",
                    help="run base params once and evaluate vs GT, then exit")
    ap.add_argument("--jobs", type=int, default=os.cpu_count(),
                    help="parallel workers")
    ap.add_argument("--out", default=None,
                    help="output dir (default benchmark/results/<dataset>/)")
    args = ap.parse_args(argv)

    registry = load_registry(args.config)
    if args.dataset not in registry:
        ap.error(f"dataset '{args.dataset}' not in {args.config}")
    entry = registry[args.dataset]
    gt_tum = entry["gt_tum"]

    out_dir = Path(args.out or f"benchmark/results/{args.dataset}")
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.verify_only:
        _verify_only(entry, gt_tum, out_dir)
        return

    sweep_path = args.sweep or f"benchmark/sweeps/{args.dataset}.yaml"
    with open(sweep_path) as f:
        spec = yaml.safe_load(f)
    combos = expand(spec)
    override_keys = sorted(spec.get("grid", {}).keys())
    tasks = list(enumerate(combos))
    print(f"{len(tasks)} combos, {args.jobs} workers")

    init_args = (entry, gt_tum, str(out_dir))
    if args.jobs <= 1:
        _worker_init(*init_args)
        rows = [_run_combo(t) for t in tasks]
    else:
        with Pool(args.jobs, initializer=_worker_init, initargs=init_args) as pool:
            rows = pool.map(_run_combo, tasks)

    rows.sort(key=lambda r: (r["ate_rmse"], r["combo_id"]))
    _write_scores(rows, override_keys, out_dir / "scores.csv")

    best = rows[0]
    best_overrides = {k: best[k] for k in override_keys}
    with open(out_dir / "best.yaml", "w") as f:
        yaml.safe_dump(
            {"base_params": entry["base_params"],
             "overrides": best_overrides,
             "ate_rmse": best["ate_rmse"]},
            f, sort_keys=False,
        )
    print(f"best ate_rmse={best['ate_rmse']:.4f} -> {best_overrides}")
    print(f"scores -> {out_dir / 'scores.csv'}")


if __name__ == "__main__":
    main()
