#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2

from my_plane_controller.msg import TargetDetection
from ultralytics import YOLO
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class YoloTargetDetector(Node):
    def __init__(self):
        super().__init__('yolo_target_detector')

        # ---------------- YOLO MODEL ----------------
        self.get_logger().info("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt')  # consider yolov8s.pt for better accuracy

        self.vehicle_classes = ['car', 'truck', 'bus', 'motorcycle']

        # ---------------- STATE ----------------
        self.current_altitude = 0.0
        self.activation_threshold = 15.0
        self.window_visible = False
        self.target_dispatched = False

        self.current_lat = 0.0
        self.current_lon = 0.0

        # ---------------- FRAME CONTROL ----------------
        self.frame_count = 0
        self.infer_every_n_frames = 3   # process 1 of every N frames

        # ---------------- CV ----------------
        self.bridge = CvBridge()

        # ---------------- QoS ----------------
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ---------------- SUBSCRIBERS ----------------
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=mavros_qos
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            qos_profile=mavros_qos
        )

        # ---------------- PUBLISHER ----------------
        self.target_pub = self.create_publisher(
            TargetDetection,
            '/detection/target',
            10
        )

        self.get_logger().info("YOLO Node initialized (zoom + throttled inference enabled).")

    # ---------------- CALLBACKS ----------------
    def pose_callback(self, msg):
        self.current_altitude = msg.pose.position.z

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    # ---------------- IMAGE ZOOM FUNCTION ----------------
    def zoom_image(self, img, zoom_factor=2.5):
        h, w = img.shape[:2]

        new_w = int(w / zoom_factor)
        new_h = int(h / zoom_factor)

        x1 = (w - new_w) // 2
        y1 = (h - new_h) // 2
        x2 = x1 + new_w
        y2 = y1 + new_h

        cropped = img[y1:y2, x1:x2]
        zoomed = cv2.resize(cropped, (w, h), interpolation=cv2.INTER_LINEAR)

        return zoomed

    # ---------------- MAIN IMAGE CALLBACK ----------------
    def image_callback(self, msg):

        # --------- FRAME SKIP ----------
        self.frame_count += 1
        if self.frame_count % self.infer_every_n_frames != 0:
            return

        # # --------- ALTITUDE GATE ----------
        # if self.current_altitude < self.activation_threshold:
        #     if self.window_visible:
        #         cv2.destroyAllWindows()
        #         self.window_visible = False
        #     return

        # --------- CONVERT IMAGE ----------
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        # --------- DIGITAL ZOOM ----------
        cv_image = self.zoom_image(cv_image, zoom_factor=2.5)

        # --------- YOLO INFERENCE ----------
        results = self.model(
            cv_image,
            imgsz=960,        # higher resolution improves small object detection
            stream=True,
            verbose=False
        )

        target_found = False

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]

                if cls_name in self.vehicle_classes:

                    xyxy = box.xyxy[0].tolist()
                    conf = float(box.conf[0])

                    pixel_x = (xyxy[0] + xyxy[2]) / 2.0
                    pixel_y = (xyxy[1] + xyxy[3]) / 2.0

                    # draw box
                    cv2.rectangle(
                        cv_image,
                        (int(xyxy[0]), int(xyxy[1])),
                        (int(xyxy[2]), int(xyxy[3])),
                        (0, 255, 0),
                        2
                    )

                    cv2.putText(
                        cv_image,
                        f"{cls_name} {conf:.2f}",
                        (int(xyxy[0]), int(xyxy[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )

                    # -------- FIRST TARGET ONLY ----------
                    if not self.target_dispatched:
                        self.get_logger().warn(
                            f"🎯 FIRST VEHICLE DETECTED: {cls_name}"
                        )

                        detection_msg = TargetDetection()
                        detection_msg.target_found = True
                        detection_msg.pixel_x = float(pixel_x)
                        detection_msg.pixel_y = float(pixel_y)
                        # detection_msg.gps_lat = self.current_lat
                        # detection_msg.gps_lon = self.current_lon
                        detection_msg.class_name = cls_name
                        detection_msg.confidence = conf

                        self.target_pub.publish(detection_msg)

                        self.target_dispatched = True
                        target_found = True

                    break

            if target_found:
                break

        # --------- DISPLAY ----------
        cv2.imshow("YOLO Active Air Stream", cv_image)
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


if __name__ == '__main__':
    main()