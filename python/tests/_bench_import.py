"""Load benchmark.evaluate/.search without triggering se3_lio/__init__.

se3_lio/__init__.py imports the compiled binding, which is unavailable on the
host. These modules are pure numpy/stdlib, so we load them straight from file.
"""

import importlib.util
from pathlib import Path

_BENCH = Path(__file__).resolve().parents[1] / "se3_lio" / "benchmark"


def _load(name):
    spec = importlib.util.spec_from_file_location(
        f"_bench_{name}", _BENCH / f"{name}.py"
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


evaluate = _load("evaluate")
search = _load("search")
