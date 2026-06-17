"""Subscribe to the node's odometry and write it as a TUM trajectory.

Exits after no message has arrived for `--idle-timeout` seconds.
"""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class Recorder(Node):
    def __init__(self, topic, out, idle_timeout):
        super().__init__("odom_recorder")
        self.out = out
        self.idle_timeout = idle_timeout
        self.rows = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Odometry, topic, self.cb, qos)
        self.last_rx = None
        self.timer = self.create_timer(0.5, self.check_idle)
        self.get_logger().info(f"recording {topic} -> {out}")

    def cb(self, msg):
        self.last_rx = self.get_clock().now()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rows.append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def check_idle(self):
        if self.last_rx is None:
            return
        dt = (self.get_clock().now() - self.last_rx).nanoseconds * 1e-9
        if dt > self.idle_timeout:
            self.get_logger().info(f"idle {dt:.1f}s, stopping with {len(self.rows)} msgs")
            raise SystemExit

    def flush(self):
        with open(self.out, "w") as f:
            for r in self.rows:
                f.write(" ".join(f"{v:.9f}" for v in r) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/local/odometry")
    ap.add_argument("--out", required=True)
    ap.add_argument("--idle-timeout", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = Recorder(args.topic, args.out, args.idle_timeout)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.flush()
        node.get_logger().info(f"wrote {len(node.rows)} poses to {args.out}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
