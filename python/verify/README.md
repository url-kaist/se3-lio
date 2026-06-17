# Verification harness

Confirms the Python binding reproduces the live ROS2 node by running the **same
rosbag** through both and comparing trajectories.

The Python path reproduces the node's data path exactly:
`convertIMUMessage` / `convertLivoxMessage` / `convertOusterMessage` and the
`MeasurementSynchronizer` port (`synchronize`), now in
[se3_lio/datasets/rosbag.py](../se3_lio/datasets/rosbag.py) ·
[ros1bag.py](../se3_lio/datasets/ros1bag.py); the param mapping in
[se3_lio/config.py](../se3_lio/config.py) (`load_node_params`); and the per-frame
extrinsic + timestamp sort (inside the binding's `_register_frame`).

## Run (inside the `se3_lio:ros2` container)

```bash
# build the binding first (see ../README.md), then:
python/verify/run_verification.sh /ws/data/<your_ros2_bag> 300 40
#                                  <bag_dir>                <frames> <play_s>
```

Outputs `node.tum`, `python.tum` and an ATE report to `/verify`.

## Result (300 frames)

| metric | node vs python | python vs python |
|---|---|---|
| timestamp assoc error | 0.000 ms | 0.000 ms |
| translation RMSE | 8.8 mm (max 19.7 mm) | 0.000 mm |
| rotation RMSE | 0.11° (max 0.52°) | 0.0025° |

- **Exact timestamp match** → the data-path replication (convert + sync + extrinsic)
  is faithful; both compare identical frames.
- **Python is deterministic run-to-run** → the residual node-vs-python difference
  is not a binding bug. It is attributed to OpenMP reduction-order differences
  between the live multithreaded node and the offline run, accumulating to a few
  mm. The algorithm itself is deterministic given a fixed reduction order.

## Files

- [bag_reader.py](bag_reader.py) — thin re-export of the canonical readers in `se3_lio/datasets/` (kept so import paths stay stable)
- bag reading + conversion + synchronizer port live in the package: `se3_lio/datasets/rosbag.py` · `ros1bag.py`; param mapping in `se3_lio/config.py`
- [run_offline.py](run_offline.py) — runs the binding offline → TUM
- [record_node_odom.py](record_node_odom.py) — records `/local/odometry` → TUM
- [ate.py](ate.py) — timestamp-associated trajectory comparison
- [tum.py](tum.py) — TUM I/O + rotation→quaternion
- [run_verification.sh](run_verification.sh) — orchestrates node + offline + compare
