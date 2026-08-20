#!/usr/bin/env python3
"""
Improved vehicle-detection node.

Key fixes vs. the original:
  1. Digital zoom no longer wastes a resize-back-up-to-full-size pass —
     the crop is fed straight to the model at inference resolution.
  2. Detection pixel coordinates are correctly remapped from the
     zoomed/cropped image back into the ORIGINAL camera frame before
     being published. (The old code published coordinates in the
     zoomed image's space, which is wrong for anything downstream that
     expects raw-frame pixels, e.g. gimbal aiming.)
  3. Explicit device selection (GPU + half precision if available,
     otherwise CPU) instead of relying on the library default.
  4. Inference restricted to the 4 vehicle classes via `classes=[...]`
     so YOLO doesn't run NMS/postprocessing over all 80 COCO classes.
  5. On-screen FPS + inference-time overlay so you can see exactly
     where time is going (this is the fastest way to diagnose lag).
  6. Display window is now optional (`show_window` param) — imshow/
     waitKey every frame is real overhead, especially over SSH/X11
     forwarding, and isn't needed once you're not actively debugging.
  7. Tunable via ROS2 parameters instead of hardcoded constants, so you
     can adjust without touching code:
       imgsz, conf_threshold, infer_every_n_frames, zoom_factor,
       show_window, model_path
"""

import time

import cv2
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from ultralytics import YOLO

from my_drone_controller.msg import TargetDetectionDrone

# COCO class indices for vehicles (avoids filtering by string every box)
VEHICLE_CLASS_IDS = {2: "car", 3: "motorcycle", 5: "bus", 7: "truck"}


class YoloTargetDetector(Node):
    def __init__(self):
        super().__init__("yolo_target_detector")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("model_path", "yolo11s.pt")
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("infer_every_n_frames", 3)
        self.declare_parameter("zoom_factor", 2.0)
        self.declare_parameter("show_window", True)

        model_path = self.get_parameter("model_path").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.infer_every_n_frames = int(self.get_parameter("infer_every_n_frames").value)
        self.zoom_factor = float(self.get_parameter("zoom_factor").value)
        self.show_window = bool(self.get_parameter("show_window").value)

        # ---------------- DEVICE ----------------
        if torch.cuda.is_available():
            self.device = "cuda:0"
            self.use_half = True
        else:
            self.device = "cpu"
            self.use_half = False
        self.get_logger().info(f"Using device: {self.device} (half precision: {self.use_half})")

        # ---------------- YOLO MODEL ----------------
        self.get_logger().info(f"Loading model: {model_path}")
        self.model = YOLO(model_path)
        self.model.to(self.device)
        self.get_logger().info("Model loaded.")

        # ---------------- STATE ----------------
        self.current_altitude = 0.0
        self.activation_threshold = 15.0
        self.window_visible = False

        self.current_lat = 0.0
        self.current_lon = 0.0

        self.frame_count = 0
        self.prev_time = time.time()

        # ---------------- CV ----------------
        self.bridge = CvBridge()

        # ---------------- QoS ----------------
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------------- SUBSCRIBERS ----------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/copter/mavros/local_position/pose",
            self.pose_callback,
            qos_profile=mavros_qos,
        )

        self.image_sub = self.create_subscription(
            Image, "/copter/camera/image_raw", self.image_callback, 10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/copter/mavros/global_position/global",
            self.gps_callback,
            qos_profile=mavros_qos,
        )

        # ---------------- PUBLISHER ----------------
        self.target_pub = self.create_publisher(
            TargetDetectionDrone, "/copter/detection/target", 10
        )

        self.get_logger().info("YOLO Node initialized (zoom + throttled inference enabled).")

    # ---------------- CALLBACKS ----------------
    def pose_callback(self, msg):
        self.current_altitude = msg.pose.position.z

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    # ---------------- ZOOM: crop only, no wasted upscale ----------------
    def crop_center(self, img, zoom_factor):
        """
        Crops the center region of the image. Returns the crop AND the
        offset (x1, y1) plus the crop's own (w, h), so callers can map
        detection coordinates back to the original frame.
        """
        h, w = img.shape[:2]
        new_w = int(w / zoom_factor)
        new_h = int(h / zoom_factor)

        x1 = (w - new_w) // 2
        y1 = (h - new_h) // 2
        x2 = x1 + new_w
        y2 = y1 + new_h

        cropped = img[y1:y2, x1:x2]
        return cropped, x1, y1

    # ---------------- MAIN IMAGE CALLBACK ----------------
    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.infer_every_n_frames != 0:
            return

        # --------- CONVERT IMAGE ----------
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        # --------- DIGITAL ZOOM (crop only — model resizes internally) ----------
        crop, offset_x, offset_y = self.crop_center(cv_image, self.zoom_factor)
        crop_h, crop_w = crop.shape[:2]

        # --------- YOLO INFERENCE ----------
        start = time.time()
        results = self.model.predict(
            crop,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            classes=list(VEHICLE_CLASS_IDS.keys()),
            device=self.device,
            half=self.use_half,
            verbose=False,
        )
        inference_ms = (time.time() - start) * 1000

        target_found = False
        annotated = crop.copy()

        if results and results[0].boxes is not None:
            for box in results[0].boxes:
                cls_id = int(box.cls[0])
                cls_name = VEHICLE_CLASS_IDS.get(cls_id)
                if cls_name is None:
                    continue

                xyxy = box.xyxy[0].tolist()
                conf = float(box.conf[0])

                # center in CROP coordinates
                crop_cx = (xyxy[0] + xyxy[2]) / 2.0
                crop_cy = (xyxy[1] + xyxy[3]) / 2.0

                # remap to ORIGINAL frame coordinates (this is the fix)
                pixel_x = offset_x + crop_cx
                pixel_y = offset_y + crop_cy

                # draw on the crop for the debug window (crop-space coords)
                cv2.rectangle(
                    annotated,
                    (int(xyxy[0]), int(xyxy[1])),
                    (int(xyxy[2]), int(xyxy[3])),
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    annotated,
                    f"{cls_name} {conf:.2f}",
                    (int(xyxy[0]), int(xyxy[1]) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

                self.get_logger().info(
                    f"VEHICLE DETECTED: {cls_name} ({conf:.2f}) at "
                    f"({pixel_x:.1f}, {pixel_y:.1f})",
                    throttle_duration_sec=1.0,
                )

                detectionDrone_msg = TargetDetectionDrone()
                detectionDrone_msg.target_found = True
                detectionDrone_msg.pixel_x = float(pixel_x)  # now in ORIGINAL frame space
                detectionDrone_msg.pixel_y = float(pixel_y)
                detectionDrone_msg.class_name = cls_name
                detectionDrone_msg.confidence = conf

                # Stamp with the SOURCE image's timestamp so the geolocator can
                # interpolate aircraft pose/GPS to the exact moment this frame
                # was captured, instead of falling back to "whatever's latest".
                if hasattr(detectionDrone_msg, "header"):
                    detectionDrone_msg.header.stamp = msg.header.stamp
                    detectionDrone_msg.header.frame_id = "camera_optical"

                # Tell the geolocator how much digital zoom was applied to
                # this frame so it scales fx/fy correctly (see geolocator's
                # pixel_to_gps docstring — skipping this biases the ground
                # position for any off-center detection).
                if hasattr(detectionDrone_msg, "zoom_factor"):
                    detectionDrone_msg.zoom_factor = float(self.zoom_factor)

                self.target_pub.publish(detectionDrone_msg)
                target_found = True

                # Publish one detection per frame (highest-priority / first
                # matching box). Remove this break if you want to publish
                # every detected vehicle box in a frame, not just one.
                break

        # --------- DIAGNOSTICS ----------
        fps = max(time.time() - self.prev_time, 1e-6)
        self.prev_time = time.time()

        if self.show_window:
            display_frame = cv2.resize(
                annotated, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LANCZOS4
            )
            cv2.putText(
                display_frame,
                f"FPS: {fps:.1f}  Inference: {inference_ms:.0f} ms",
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 0, 255),
                2,
            )
            cv2.imshow("YOLO Active Air Stream", display_frame)
            self.window_visible = True
            cv2.waitKey(1)


# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = YoloTargetDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()