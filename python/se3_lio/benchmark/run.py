"""Benchmark runner: `python -m se3_lio.benchmark.run <target>`.

Expands a grid sweep and, for each combo, runs SE(3)-LIO over one or more bags,
evaluates rigid-aligned ATE against ground truth, and ranks combos. With a
single bag the score is that bag's ATE RMSE; with a suite (the sweep yaml lists
`datasets:`) the score is the mean of per-bag RMSE, so the winner generalizes
across bags. Writes scores.csv + best.yaml. The compiled binding is imported
lazily inside the functions that actually execute a sweep, so
registry/search/evaluate stay host-importable.
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

# Frames + paths for the bag currently being swept in this process. Populated by
# the worker initializer (or directly for --jobs 1).
_BAG = {}


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


def _run_combo(args):
    """Run one override combo over the bag's frames -> result row."""
    from se3_lio.pipeline import OdometryPipeline

    combo_id, overrides = args
    p = _config_from_overrides(_BAG["base_params"], overrides)
    pipe = OdometryPipeline(_BAG["frames"], p["config"], _BAG["extrinsic"])
    pipe.run(progress=False)
    tum_path = Path(_BAG["out_dir"]) / f"combo_{combo_id:04d}.tum"
    pipe.save_tum(tum_path)
    metrics = ev.evaluate(str(tum_path), _BAG["gt_tum"])
    return {"combo_id": combo_id, "ate_rmse": metrics["ate_rmse"]}


def _worker_init(entry, gt_tum, out_dir):
    """Read the bag's frames in THIS process, then warm the core up once.

    The core's first pipeline run in a process differs from later ones (its
    floating-point accumulation order depends on the heap layout, which a first
    run reshapes; it then stays stable). Each process reads its own frames and
    warms up, so every measured combo is compared on equal footing -> scores are
    reproducible and identical across serial/parallel. (Reading per worker, not
    sharing via fork, is deliberate: an inherited parent heap perturbs the
    regime; an independent read+warm-up keeps it deterministic.)
    """
    from se3_lio.pipeline import OdometryPipeline

    frames, p = _load_frames(entry)
    _BAG.update(frames=frames, extrinsic=p["extrinsic"], base_params=p["base_params"],
                gt_tum=gt_tum, out_dir=out_dir)
    OdometryPipeline(frames, p["config"], p["extrinsic"]).run(progress=False)


def _run_bag(entry, gt_tum, bag_out_dir, tasks, jobs):
    """Run every combo over one bag's frames -> {combo_id: ate_rmse}."""
    bag_out_dir.mkdir(parents=True, exist_ok=True)
    init_args = (entry, gt_tum, str(bag_out_dir))
    if jobs <= 1:
        _worker_init(*init_args)
        rows = [_run_combo(t) for t in tasks]
    else:
        with Pool(jobs, initializer=_worker_init, initargs=init_args) as pool:
            rows = pool.map(_run_combo, tasks)
    return {r["combo_id"]: r["ate_rmse"] for r in rows}


def _mean(values):
    return sum(values) / len(values)  # any inf propagates -> combo ranks last


_VERIFY_KEYS = [
    "ate_rmse", "ate_mean", "ate_median", "ate_max",
    "rot_rmse_deg", "n_assoc", "n_est", "n_gt",
]


def _verify_only(entry, gt_tum, out_dir):
    from se3_lio.pipeline import OdometryPipeline

    frames, p = _load_frames(entry)
    print(f"synchronized {len(frames)} frames")
    OdometryPipeline(frames, p["config"], p["extrinsic"]).run(progress=False)  # warm-up (see _worker_init)
    pipe = OdometryPipeline(frames, p["config"], p["extrinsic"])
    pipe.run(progress=True)
    tum_path = Path(out_dir) / "verify.tum"
    pipe.save_tum(tum_path)
    metrics = ev.evaluate(str(tum_path), gt_tum)
    print(pipe.summary())
    print(f"trajectory -> {tum_path}")
    for k in _VERIFY_KEYS:
        print(f"  {k}: {metrics[k]}")
    return metrics


def _write_scores(results, override_keys, ds_keys, path):
    fields = ["combo_id"] + override_keys + [f"rmse_{k}" for k in ds_keys] + ["mean_rmse"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in results:
            w.writerow({k: r.get(k, "") for k in fields})


def main(argv=None):
    ap = argparse.ArgumentParser(prog="se3_lio.benchmark.run")
    ap.add_argument("target", help="dataset key (single bag) or sweep/output name (suite)")
    ap.add_argument("--config", default="benchmark/datasets.yaml",
                    help="dataset registry yaml")
    ap.add_argument("--sweep", default=None,
                    help="sweep yaml (default benchmark/sweeps/<target>.yaml)")
    ap.add_argument("--verify-only", action="store_true",
                    help="run base params once and evaluate vs GT, then exit")
    ap.add_argument("--jobs", type=int, default=os.cpu_count(),
                    help="parallel workers")
    ap.add_argument("--out", default=None,
                    help="output dir (default benchmark/results/<target>/)")
    args = ap.parse_args(argv)

    registry = load_registry(args.config)
    out_dir = Path(args.out or f"benchmark/results/{args.target}")
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.verify_only:
        if args.target not in registry:
            ap.error(f"dataset '{args.target}' not in {args.config}")
        entry = registry[args.target]
        _verify_only(entry, entry["gt_tum"], out_dir)
        return

    sweep_path = args.sweep or f"benchmark/sweeps/{args.target}.yaml"
    with open(sweep_path) as f:
        spec = yaml.safe_load(f)
    combos = expand(spec)
    override_keys = sorted(spec.get("grid", {}).keys())
    tasks = list(enumerate(combos))

    # A suite lists `datasets:`; otherwise the target itself is the single bag.
    ds_keys = spec.get("datasets") or [args.target]
    for k in ds_keys:
        if k not in registry:
            ap.error(f"dataset '{k}' not in {args.config}")
    print(f"{len(combos)} combos x {len(ds_keys)} bag(s), {args.jobs} workers")

    # combo_id -> {ds_key -> ate_rmse}; bags run sequentially (frames cached
    # per bag), combos run in parallel within each bag.
    per_bag = {cid: {} for cid, _ in tasks}
    for ds_key in ds_keys:
        entry = registry[ds_key]
        rmse_by_combo = _run_bag(
            entry, entry["gt_tum"], out_dir / ds_key, tasks, args.jobs
        )
        for cid, rmse in rmse_by_combo.items():
            per_bag[cid][ds_key] = rmse
        print(f"  {ds_key}: done")

    results = []
    for cid, overrides in tasks:
        row = {"combo_id": cid}
        row.update(overrides)
        for k in ds_keys:
            row[f"rmse_{k}"] = per_bag[cid][k]
        row["mean_rmse"] = _mean([per_bag[cid][k] for k in ds_keys])
        results.append(row)
    results.sort(key=lambda r: (r["mean_rmse"], r["combo_id"]))
    _write_scores(results, override_keys, ds_keys, out_dir / "scores.csv")

    best = results[0]
    best_overrides = {k: best[k] for k in override_keys}
    with open(out_dir / "best.yaml", "w") as f:
        yaml.safe_dump(
            {"base_params": registry[ds_keys[0]]["base_params"],
             "datasets": ds_keys,
             "overrides": best_overrides,
             "mean_rmse": best["mean_rmse"],
             "per_bag_rmse": {k: best[f"rmse_{k}"] for k in ds_keys}},
            f, sort_keys=False,
        )
    print(f"best mean_rmse={best['mean_rmse']:.4f} -> {best_overrides}")
    print(f"scores -> {out_dir / 'scores.csv'}")


if __name__ == "__main__":
    main()
