import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        # Declare ROS2 parameters
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf_thres", 0.25)
        self.declare_parameter("iou_thres", 0.45)

        # Get parameter values
        self.imgsz = self.get_parameter("imgsz").value
        self.conf_thres = self.get_parameter("conf_thres").value
        self.iou_thres = self.get_parameter("iou_thres").value

        # Load YOLO model
        self.model = YOLO("yolov8n.pt")

        # Subscribe to camera image topic
        self.sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.callback,
            10
        )

        # Publisher for annotated images
        self.pub = self.create_publisher(
            Image,
            "/camera/yolo_annotated",
            10
        )

        self.get_logger().info(
            f"YOLO Node Started\n"
            f"Listening on: /camera/image_raw\n"
            f"Publishing: /camera/yolo_annotated\n"
            f"Image Size: {self.imgsz}\n"
            f"Confidence Threshold: {self.conf_thres}\n"
            f"IoU Threshold: {self.iou_thres}"
        )

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            # Run YOLO inference
            results = self.model(
                frame,
                imgsz=self.imgsz,
                conf=self.conf_thres,
                iou=self.iou_thres,
                verbose=False
            )[0]

            # Draw detections
            annotated_frame = results.plot()

            # Convert back to ROS Image
            annotated_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding="bgr8"
            )

            # Preserve original timestamp and frame_id
            annotated_msg.header = msg.header

            # Publish annotated image
            self.pub.publish(annotated_msg)

            # Display image (optional)
            cv2.imshow("Skywalker YOLO Feed", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(
                f"Failed to process image frame: {str(e)}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()

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