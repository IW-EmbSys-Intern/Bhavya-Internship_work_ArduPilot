#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix
import cv2

# Import MAVROS Pose message to monitor altitude
from geometry_msgs.msg import PoseStamped

# Import your custom compiled message
from my_plane_controller.msg import TargetDetection

# Import Ultralytics YOLO
from ultralytics import YOLO

# Import QoS handling for MAVROS compatibility
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class YoloTargetDetector(Node):
    def __init__(self):
        super().__init__('yolo_target_detector')
        
        # 1. Initialize YOLO
        self.get_logger().info("Loading YOLOv8 model...")
        self.model = YOLO('yolov8n.pt') 
        
        # Explicit vehicle classes we want to look for
        self.vehicle_classes = ['car', 'truck', 'bus', 'motorcycle']
        
        # 2. State & Tracking Flags
        self.current_altitude = 0.0
        self.activation_threshold = 15.0  # Threshold in meters (AGL)
        self.window_visible = False
        
        # ONE-TIME GATEKEEPER FLAG
        self.target_dispatched = False
        
        # 3. Tools
        self.bridge = CvBridge()
        
        # 4. Publishers & Subscribers
        # mavros_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     depth=10
        # )
        # mavros_qos = QoSProfile(
        #     reliability=ReliabilityPolicy.RELIABLE,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=10
        # )
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
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
        
        self.target_pub = self.create_publisher(
            TargetDetection, 
            '/detection/target', 
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            mavros_qos
        )

        self.current_lat = 0.0
        self.current_lon = 0.0

        self.get_logger().info("YOLO Node initialized. Idle standing-by below 15m.")

    def pose_callback(self, msg):
        self.current_altitude = msg.pose.position.z

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def image_callback(self, msg):
        # Gatekeeper: Check altitude threshold before running inference
        if self.current_altitude < self.activation_threshold:
            if self.window_visible:
                cv2.destroyAllWindows()
                self.window_visible = False
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return

        # Run YOLO inference continuously (allows you to keep watching the stream live)
        results = self.model(cv_image, stream=True, verbose=False)
        
        target_found = False
        pixel_x = 0.0
        pixel_y = 0.0
        
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                
                if cls_name in self.vehicle_classes:
                    xyxy = box.xyxy[0].tolist()
                    pixel_x = (xyxy[0] + xyxy[2]) / 2.0
                    pixel_y = (xyxy[1] + xyxy[3]) / 2.0
                    conf = float(box.conf[0])
                    
                    # Visual styling: Draw the bounding boxes on your live window stream
                    cv2.rectangle(cv_image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"{cls_name} {conf:.2f}", (int(xyxy[0]), int(xyxy[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # ONLY execute if this is the FIRST vehicle ever spotted
                    if not self.target_dispatched:
                        target_found = True
                        self.get_logger().warn(f"🎯 FIRST VEHICLE FOUND: Sent {cls_name} at ({pixel_x:.1f}, {pixel_y:.1f}) to plane node. Network pipeline is now LOCKED.")
                        
                        # Build and publish the coordinate payload immediately
                        detection_msg = TargetDetection()
                        detection_msg.target_found = True
                        detection_msg.pixel_x = float(pixel_x)
                        detection_msg.pixel_y = float(pixel_y)
                        # detection_msg.gps_lat = self.current_lat + 0.0002
                        # detection_msg.gps_lon = self.current_lon + 0.0002
                        detection_msg.gps_lat = self.current_lat
                        detection_msg.gps_lon = self.current_lon
                        self.target_pub.publish(detection_msg)
                        
                        # Flip flag to True so this block never triggers again
                        self.target_dispatched = True
                    else:
                        # Log to console that we see vehicles but are safely ignoring them for navigation purposes
                        self.get_logger().info(f"Ignored secondary target ({cls_name}) to prevent plane navigation confusion.", throttle_duration_sec=2.0)
                    
                    break
            if target_found:
                break
        
        # Render the uninterrupted live stream window
        cv2.imshow("YOLO Active Air Stream", cv_image)
        self.window_visible = True
        cv2.waitKey(1)

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