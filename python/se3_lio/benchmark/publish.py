"""Publish the winning sweep combo as a complete, ready-to-use params.yaml.

Reads best.yaml (base_params + winning overrides, written by run.py), applies
the overrides onto the base node params, and writes a full yaml. Defaults to
the private results dir; pass --out to promote it into a public
pipelines/<ros>/config/ location.
"""

import argparse
from pathlib import Path

import yaml

from se3_lio.benchmark.run import _apply_overrides


def publish(best_path, out_path):
    with open(best_path) as f:
        best = yaml.safe_load(f)
    with open(best["base_params"]) as f:
        raw = yaml.safe_load(f)
    merged = _apply_overrides(raw, best.get("overrides", {}))
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        yaml.safe_dump(merged, f, sort_keys=False)
    return out_path


def main(argv=None):
    ap = argparse.ArgumentParser(prog="se3_lio.benchmark.publish")
    ap.add_argument("dataset", help="dataset key; locates results/<dataset>/best.yaml")
    ap.add_argument("--best", default=None,
                    help="best.yaml (default benchmark/results/<dataset>/best.yaml)")
    ap.add_argument("--out", default=None,
                    help="output yaml (default benchmark/results/<dataset>/<dataset>_tuned.yaml)")
    args = ap.parse_args(argv)
    best_path = args.best or f"benchmark/results/{args.dataset}/best.yaml"
    out_path = args.out or f"benchmark/results/{args.dataset}/{args.dataset}_tuned.yaml"
    publish(best_path, out_path)
    print(f"published {best_path} -> {out_path}")


if __name__ == "__main__":
    main()
