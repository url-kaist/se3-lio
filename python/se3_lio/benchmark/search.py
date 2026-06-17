"""Search strategies for parameter sweeps.

Only grid search is implemented. The single public entry point is `expand`,
which keeps a seam for adding random/optuna strategies later without changing
callers (they would dispatch on a `spec` key).
"""

from itertools import product


def expand(spec):
    """Expand a sweep spec into a list of override dicts.

    spec = {"grid": {dotted_key: [values], ...}}
    returns the cartesian product as [{dotted_key: value, ...}, ...].
    """
    grid = spec.get("grid", {})
    keys = list(grid)
    if not keys:
        return [{}]
    return [dict(zip(keys, combo)) for combo in product(*(grid[k] for k in keys))]
