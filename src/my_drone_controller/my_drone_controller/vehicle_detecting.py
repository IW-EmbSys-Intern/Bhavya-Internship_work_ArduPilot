#!/usr/bin/env python3

"""
ROS2 ArduPilot GUIDED mission controller using MAVROS.

Designed for:
- ROS2 Jazzy
- MAVROS 2.14
- ArduCopter SITL
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from mavros_msgs.msg import State, CommandCode
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, CommandInt

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


def distance_meters(lat1, lon1, lat2, lon2):
    """
    Haversine distance between GPS points.
    """
    r = 6371000.0

    p1 = math.radians(lat1)
    p2 = math.radians(lat2)

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)

    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(p1)
        * math.cos(p2)
        * math.sin(dlon / 2) ** 2
    )

    return 2 * r * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def normalize_angle(angle):
    return angle % 360.0


def angle_difference(current, target):
    return (target - current + 180) % 360 - 180


class GuidedMissionNode(Node):

    def __init__(self):
        super().__init__("guided_mission_node")


        # -------------------------
        # Parameters
        # -------------------------

        self.declare_parameter(
            "target_lat",
            -35.36212801
        )

        self.declare_parameter(
            "target_lon",
            149.16512180
        )

        self.declare_parameter(
            "target_alt",
            50.0
        )

        self.declare_parameter(
            "target_yaw",
            90.0
        )

        self.declare_parameter(
            "arrival_radius",
            3.0
        )


        self.target_lat = self.get_parameter(
            "target_lat"
        ).value

        self.target_lon = self.get_parameter(
            "target_lon"
        ).value

        self.target_alt = self.get_parameter(
            "target_alt"
        ).value

        self.target_yaw = self.get_parameter(
            "target_yaw"
        ).value

        self.arrival_radius = self.get_parameter(
            "arrival_radius"
        ).value



        # -------------------------
        # State variables
        # -------------------------

        self.state = State()

        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_rel_alt = None
        self.current_heading = None



        # -------------------------
        # MAVROS subscriptions
        # -------------------------

        qos = QoSPresetProfiles.SENSOR_DATA.value


        self.create_subscription(
            State,
            "/copter/mavros/state",
            self.state_cb,
            10
        )


        self.create_subscription(
            NavSatFix,
            "/copter/mavros/global_position/global",
            self.gps_cb,
            qos
        )


        self.create_subscription(
            Float64,
            "/copter/mavros/global_position/rel_alt",
            self.alt_cb,
            qos
        )


        self.create_subscription(
            Float64,
            "/copter/mavros/global_position/compass_hdg",
            self.heading_cb,
            qos
        )



        # -------------------------
        # MAVROS services
        # -------------------------

        self.arm_client = self.create_client(
            CommandBool,
            "/copter/mavros/cmd/arming"
        )


        self.mode_client = self.create_client(
            SetMode,
            "/copter/mavros/set_mode"
        )


        self.takeoff_client = self.create_client(
            CommandTOL,
            "/copter/mavros/cmd/takeoff"
        )


        # MAV_CMD_DO_REPOSITION
        self.reposition_client = self.create_client(
            CommandInt,
            "/copter/mavros/cmd/command_int"
        )



        self.get_logger().info(
            "Guided Mission Node initialized"
        )

            # -------------------------------------------------
    # Callbacks
    # -------------------------------------------------

    def state_cb(self, msg):
        self.state = msg


    def gps_cb(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude


    def alt_cb(self, msg):
        self.current_rel_alt = msg.data


    def heading_cb(self, msg):
        self.current_heading = msg.data



    # -------------------------------------------------
    # Helpers
    # -------------------------------------------------

    def wait_until(self, condition, message="Waiting...", timeout=None):

        start = time.time()

        while rclpy.ok():

            if condition():
                return True

            if timeout:
                if time.time() - start > timeout:
                    self.get_logger().error(
                        "Timeout: " + message
                    )
                    return False

            self.get_logger().info(
                message
            )

            rclpy.spin_once(
                self,
                timeout_sec=0.5
            )

        return False



    def call_service(self, client, request, name):

        if not client.wait_for_service(
            timeout_sec=10
        ):
            self.get_logger().error(
                f"{name} service unavailable"
            )
            return None


        future = client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        return future.result()



    # -------------------------------------------------
    # Connection
    # -------------------------------------------------

    def wait_connection(self):

        self.get_logger().info(
            "Waiting for FCU..."
        )

        self.wait_until(
            lambda: self.state.connected,
            "Waiting for MAVROS connection"
        )


        self.get_logger().info(
            "FCU connected"
        )


        self.wait_until(
            lambda:
            self.current_lat is not None,
            "Waiting for GPS"
        )


        self.get_logger().info(
            "GPS ready"
        )



    # -------------------------------------------------
    # GUIDED mode
    # -------------------------------------------------

    def set_guided(self):

        self.get_logger().info(
            "Changing to GUIDED"
        )


        req = SetMode.Request()
        req.custom_mode = "GUIDED"


        result = self.call_service(
            self.mode_client,
            req,
            "set_mode"
        )


        if result is None or not result.mode_sent:
            raise RuntimeError(
                "GUIDED mode failed"
            )


        self.wait_until(
            lambda:
            self.state.mode == "GUIDED",
            "Waiting for GUIDED mode"
        )


        self.get_logger().info(
            "GUIDED active"
        )



    # -------------------------------------------------
    # ARM
    # -------------------------------------------------

    def arm(self):

        self.get_logger().info(
            "Arming vehicle"
        )


        req = CommandBool.Request()
        req.value = True


        result = self.call_service(
            self.arm_client,
            req,
            "arming"
        )


        if result is None or not result.success:
            raise RuntimeError(
                "Arm failed"
            )


        self.wait_until(
            lambda:
            self.state.armed,
            "Waiting for arm"
        )


        self.get_logger().info(
            "Vehicle armed"
        )



    # -------------------------------------------------
    # TAKEOFF
    # -------------------------------------------------

    def takeoff(self):

        self.get_logger().info(
            f"Taking off to {self.target_alt} m"
        )


        req = CommandTOL.Request()

        req.altitude = float(
            self.target_alt
        )

        req.latitude = 0.0
        req.longitude = 0.0
        req.yaw = 0.0
        req.min_pitch = 0.0


        result = self.call_service(
            self.takeoff_client,
            req,
            "takeoff"
        )


        if result is None or not result.success:
            raise RuntimeError(
                "Takeoff failed"
            )


        self.wait_until(
            lambda:
            self.current_rel_alt is not None,
            "Waiting altitude"
        )


        self.wait_until(
            lambda:
            self.current_rel_alt >
            self.target_alt * 0.95,
            "Climbing"
        )


        self.get_logger().info(
            "Reached takeoff altitude"
        )



    # -------------------------------------------------
    # GUIDED GPS movement
    # -------------------------------------------------

    def goto_position(self):

        self.get_logger().info(
            "Sending GUIDED reposition command"
        )


        req = CommandInt.Request()


        # MAV_FRAME_GLOBAL_RELATIVE_ALT
        req.frame = 6


        # MAV_CMD_DO_REPOSITION
        req.command = 192


        req.current = 0
        req.autocontinue = 0


        # params:
        # param1 = ground speed
        # param2 = bitmask
        # param3 = loiter radius
        # param4 = yaw
        # param5/6/7 = lat lon alt

        req.param1 = 5.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = float("nan")


        req.x = int(
            self.target_lat * 1e7
        )

        req.y = int(
            self.target_lon * 1e7
        )

        req.z = float(
            self.target_alt
        )


        result = self.call_service(
            self.reposition_client,
            req,
            "reposition"
        )


        if result is None or not result.success:
            raise RuntimeError(
                "Reposition command failed"
            )


        self.get_logger().info(
            "Reposition command accepted"
        )



        def arrived():

            if self.current_lat is None:
                return False


            d = distance_meters(
                self.current_lat,
                self.current_lon,
                self.target_lat,
                self.target_lon
            )


            self.get_logger().info(
                f"Distance: {d:.1f} m"
            )


            return d < self.arrival_radius



        self.wait_until(
            arrived,
            "Flying to waypoint"
        )


        self.get_logger().info(
            "Waypoint reached"
        )

            # -------------------------------------------------
    # Yaw alignment
    # -------------------------------------------------

    def set_yaw(self):

        target = normalize_angle(
            self.target_yaw
        )

        self.get_logger().info(
            f"Rotating to heading {target:.1f} degrees"
        )


        # ArduPilot accepts MAV_CMD_CONDITION_YAW
        # through command_int

        req = CommandInt.Request()

        # MAV_FRAME_GLOBAL_RELATIVE_ALT
        req.frame = 6

        # MAV_CMD_CONDITION_YAW
        req.command = 115

        req.current = 0
        req.autocontinue = 0


        # param1 = target angle
        # param2 = yaw speed deg/sec
        # param3 = direction
        # param4 = relative/absolute
        req.param1 = float(target)
        req.param2 = 20.0
        req.param3 = 1.0
        req.param4 = 0.0


        # unused
        req.x = 0
        req.y = 0
        req.z = 0.0


        result = self.call_service(
            self.reposition_client,
            req,
            "condition_yaw"
        )


        if result is None or not result.success:
            raise RuntimeError(
                "Yaw command failed"
            )


        def yaw_ok():

            if self.current_heading is None:
                return False

            error = abs(
                angle_difference(
                    self.current_heading,
                    target
                )
            )

            self.get_logger().info(
                f"Heading {self.current_heading:.1f} "
                f"error {error:.1f}"
            )

            return error < 3.0


        self.wait_until(
            yaw_ok,
            "Waiting for yaw"
        )


        self.get_logger().info(
            "Yaw aligned"
        )



    # -------------------------------------------------
    # Hold position
    # -------------------------------------------------

    def hover(self):

        self.get_logger().info(
            "Holding position"
        )

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=1.0
            )



    # -------------------------------------------------
    # Mission sequence
    # -------------------------------------------------

    def run_mission(self):

        try:

            self.wait_connection()

            self.set_guided()

            self.arm()

            self.takeoff()

            self.goto_position()

            self.set_yaw()

            self.hover()


        except Exception as e:

            self.get_logger().error(
                f"Mission failed: {e}"
            )



# -------------------------------------------------
# Main
# -------------------------------------------------

def main(args=None):

    rclpy.init(args=args)

    node = GuidedMissionNode()


    try:

        node.run_mission()


    except KeyboardInterrupt:

        node.get_logger().info(
            "Stopped by user"
        )


    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == "__main__":

    main()