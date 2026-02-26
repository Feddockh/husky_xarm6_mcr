#!/usr/bin/env python3
import time
import json
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_srvs.srv import Trigger
from sensor_msgs.msg import Image


def _sample_views_from_hemisphere(center: np.ndarray, n: int, rmin: float, rmax: float) -> List[Tuple[np.ndarray, np.ndarray]]:
    """
    Kept only to preserve the same metadata structure.
    We DO NOT move the camera anymore, but we still sample 'virtual' views
    so your dataset structure matches your old pipeline.
    """
    out = []
    for _ in range(n):
        v = np.random.normal(size=3)
        v = v / (np.linalg.norm(v) + 1e-9)
        if v[2] < 0.0:
            v[2] *= -1.0  # hemisphere (z>=0)

        radius = np.random.uniform(rmin, rmax)
        pos = center + radius * v
        out.append((pos, center.copy()))
    return out


def _newest_file(path: Path) -> Path | None:
    files = [p for p in path.glob("*") if p.is_file()]
    if not files:
        return None
    return max(files, key=lambda p: p.stat().st_mtime)


class GazeboViewpointCapture(Node):
    """
    Capture node that only TRIGGERS and RECORDS saved filenames.
    No Gazebo pose service, no entity motion.
    """

    def __init__(self):
        super().__init__("gazebo_viewpoint_capture_node")

        # Inputs / outputs
        self.declare_parameter("gt_yaml", "")
        self.declare_parameter("output_dir", "/home/hayden/tmp/saved_images")
        self.declare_parameter("left_save_dir", "/home/hayden/tmp/saved_images/firefly_left")

        # Trigger + ack
        self.declare_parameter("trigger_service", "/trigger/send_trigger")
        self.declare_parameter("ack_image_topic", "/firefly_left/image_rect")  # you can change to image_rect_scaled

        # Sampling metadata (virtual views only)
        self.declare_parameter("num_views_per_point", 10)
        self.declare_parameter("min_radius", 0.15)
        self.declare_parameter("max_radius", 0.60)
        self.declare_parameter("seed", 0)

        # Read params
        self.gt_yaml = self.get_parameter("gt_yaml").value
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.left_save_dir = Path(self.get_parameter("left_save_dir").value)

        self.trigger_service_name = self.get_parameter("trigger_service").value
        self.ack_image_topic = self.get_parameter("ack_image_topic").value

        self.num_views = int(self.get_parameter("num_views_per_point").value)
        self.rmin = float(self.get_parameter("min_radius").value)
        self.rmax = float(self.get_parameter("max_radius").value)

        seed = int(self.get_parameter("seed").value)
        if seed != 0:
            np.random.seed(seed)

        if not self.gt_yaml:
            raise RuntimeError("gt_yaml is empty. Provide your markers YAML path.")

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.left_save_dir.mkdir(parents=True, exist_ok=True)

        # Ack subscriber (detect new frame arrived)
        self._last_ack_stamp = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Image, self.ack_image_topic, self._ack_cb, qos)

        # Trigger service client (wait longer; Gazebo startup races are common)
        self.trigger_cli = self.create_client(Trigger, self.trigger_service_name)
        self._wait_for_service(self.trigger_cli, self.trigger_service_name, timeout_sec=60.0)

        self.get_logger().info(f"GT yaml: {self.gt_yaml}")
        self.get_logger().info(f"Ack image topic: {self.ack_image_topic}")
        self.get_logger().info(f"Left save dir (newest-file lookup): {self.left_save_dir}")
        self.get_logger().info(f"Trigger service: {self.trigger_service_name}")
        self.get_logger().info("Starting capture in 1s...")

        self._started = False
        self.create_timer(1.0, self._run_once)

    def _wait_for_service(self, client, srv_name: str, timeout_sec: float = 60.0):
        t0 = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - t0 > timeout_sec:
                raise RuntimeError(f"Service not available: {srv_name}")
            self.get_logger().info(f"Waiting for service {srv_name}...")

    def _ack_cb(self, msg: Image):
        self._last_ack_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def _wait_for_new_ack(self, prev_stamp, timeout_s=2.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._last_ack_stamp is not None and self._last_ack_stamp != prev_stamp:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def _trigger_once(self):
        fut = self.trigger_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().success:
            msg = fut.result().message if fut.result() else "no response"
            raise RuntimeError(f"trigger failed: {msg}")

    def _load_markers(self) -> Dict:
        with open(self.gt_yaml, "r") as f:
            data = yaml.safe_load(f)
        if "markers" not in data:
            raise RuntimeError("GT YAML missing 'markers' list")
        return data

    def _run_once(self):
        if self._started:
            return
        self._started = True

        gt = self._load_markers()
        frame_id = gt.get("frame_id", "world")
        markers = gt["markers"]

        meta = {
            "gt_yaml": self.gt_yaml,
            "frame_id": frame_id,
            "trigger_service": self.trigger_service_name,
            "ack_image_topic": self.ack_image_topic,
            "left_save_dir": str(self.left_save_dir),
            "num_views_per_point": self.num_views,
            "min_radius": self.rmin,
            "max_radius": self.rmax,
            "captures": []
        }

        last_file = _newest_file(self.left_save_dir)

        for m in markers:
            mid = int(m.get("id", -1))
            cls = int(m.get("class_id", -1))
            p = m["position"]
            center = np.array([p["x"], p["y"], p["z"]], dtype=float)

            # We still create virtual "views" for metadata indexing, but DO NOT move.
            views = _sample_views_from_hemisphere(center, self.num_views, self.rmin, self.rmax)
            self.get_logger().info(f"[marker {mid}] class={cls} center={center.tolist()} captures={len(views)}")

            for vi, (pos, tgt) in enumerate(views):
                prev_ack = self._last_ack_stamp

                # Trigger one synchronized set
                self._trigger_once()

                # Wait for new rectified frame (ack)
                ok = self._wait_for_new_ack(prev_ack, timeout_s=3.0)
                if not ok:
                    self.get_logger().warn("Timed out waiting for ack image; continuing anyway.")

                # Wait briefly until a new file appears
                t0 = time.time()
                newest = _newest_file(self.left_save_dir)
                while time.time() - t0 < 2.0 and (newest is None or newest == last_file):
                    time.sleep(0.05)
                    newest = _newest_file(self.left_save_dir)

                if newest is None:
                    self.get_logger().warn("No files found in left_save_dir after trigger.")
                    saved = None
                else:
                    saved = str(newest)
                    last_file = newest

                meta["captures"].append({
                    "marker_id": mid,
                    "class_id": cls,
                    "view_index": vi,

                    # Keep these for compatibility with your old metadata format
                    "virtual_camera_pos": pos.tolist(),
                    "virtual_target": tgt.tolist(),

                    "saved_left_image": saved,
                    "prev_ack_stamp": {"sec": prev_ack[0], "nsec": prev_ack[1]} if prev_ack else None,
                    "new_ack_stamp": {"sec": self._last_ack_stamp[0], "nsec": self._last_ack_stamp[1]} if self._last_ack_stamp else None,
                })

                if (vi + 1) % 5 == 0:
                    self.get_logger().info(f"  marker {mid}: captured {vi+1}/{len(views)}")

        out = self.output_dir / "capture_metadata.json"
        with open(out, "w") as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(f"Done. Metadata written: {out}")


def main():
    rclpy.init()
    node = GazeboViewpointCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()