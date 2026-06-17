"""Dataset registry: maps a dataset key to its bag, base params, GT, topics.

Imports no compiled binding. `resolve` defers loading the base params (which
pulls in the binding via load_node_params) to the caller in run.py.
"""

import yaml

_OVERRIDE_KEYS = (
    "imu_topic",
    "lidar_topic",
    "min_range",
    "max_frames",
    "input_type",
    "bag",
    "gt_tum",
)


def load_registry(path):
    """Load the dataset registry yaml -> {dataset_key: entry_dict}."""
    with open(path) as f:
        return yaml.safe_load(f) or {}


def resolve(entry):
    """Merge registry-level fields onto the loaded base node params.

    Loads `entry["base_params"]` via load_node_params (binding required) and
    overrides imu_topic/lidar_topic/min_range with any registry values, then
    carries bag/gt_tum/input_type/max_frames through. Returns a flat dict.
    """
    from se3_lio.config import load_node_params

    p = load_node_params(entry["base_params"])
    resolved = {
        "config": p["config"],
        "extrinsic": p["extrinsic"],
        "imu_topic": p["imu_topic"],
        "lidar_topic": p["lidar_topic"],
        "min_range": p["min_range"],
        "base_params": entry["base_params"],
        "input_type": "ros1-ouster",
        "max_frames": None,
        "bag": None,
        "gt_tum": None,
    }
    for k in _OVERRIDE_KEYS:
        if k in entry and entry[k] is not None:
            resolved[k] = entry[k]
    return resolved
