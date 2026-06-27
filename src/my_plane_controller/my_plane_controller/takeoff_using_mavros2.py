import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear
from mavros_msgs.msg import Waypoint, State
from mavros_msgs.srv import WaypointSetCurrent
from geographic_msgs.msg import GeoPoseStamped

import math


class PlaneMission(Node):
    def __init__(self):
        super().__init__('plane_mission')

        # MAVROS services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')

        # state
        self.timer = self.create_timer(2.0, self.run_once)
        self.step = 0
        self.done = False

        self.mission = []
        self.mode_future = None
        self.arm_future = None
        self.wp_future = None

        # FCU state
        self.current_state = None
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_cb,
            10
        )
        self.wp_set_current_client = self.create_client(
            WaypointSetCurrent,
            '/mavros/mission/set_current'
        )

        self.get_logger().info("Waiting for MAVROS services...")
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.wp_clear_client.wait_for_service()
        self.wp_push_client.wait_for_service()

        # fake home
        self.home_lat = -35.36325846
        self.home_lon = 149.16523276

    # -------------------------
    # utilities
    # -------------------------
    def state_cb(self, msg):
        self.current_state = msg

    def meters_to_latlon(self, dx, dy):
        lat = self.home_lat + (dy / 111111.0)
        lon = self.home_lon + (dx / (111111.0 * math.cos(math.radians(self.home_lat + 1e-6))))
        return lat, lon

    def create_wp(self, lat, lon, alt, command):
        wp = Waypoint()
        wp.frame = 3  # GLOBAL_RELATIVE_ALT
        wp.command = command
        wp.is_current = False
        wp.autocontinue = True

        wp.param1 = 0.0
        wp.param2 = 0.0
        wp.param3 = 0.0
        wp.param4 = 0.0

        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        return wp

    # -------------------------
    # mission builder
    # -------------------------
    def build_mission(self):
        alt = 100.0

        p0 = (0, 0)
        p1 = (100, 0)
        p2 = (300, 300)
        center = (50, 50)
        p3 = (150, 150)
        p4 = (100, 100)
        p5 = (50, 50)

        lat0, lon0 = self.meters_to_latlon(*p0)
        lat1, lon1 = self.meters_to_latlon(*p1)
        lat2, lon2 = self.meters_to_latlon(*p2)
        latc, lonc = self.meters_to_latlon(*center)
        lat3, lon3 = self.meters_to_latlon(*p3)
        lat4, lon4 = self.meters_to_latlon(*p4)
        lat5, lon5 = self.meters_to_latlon(*p5)

        self.mission = [
            self.create_wp(lat0, lon0, alt, 16),  # waypoint
            self.create_wp(lat1, lon1, alt, 22),
            self.create_wp(lat2, lon2, alt, 16),
            self.create_wp(latc, lonc, alt, 16),
            self.create_wp(lat3, lon3, alt, 16),
            self.create_wp(lat4, lon4, alt, 21),
            self.create_wp(lat5, lon5, alt, 16),
        ]

        self.mission[0].is_current = True

    # -------------------------
    # state machine
    # -------------------------
    def run_once(self):

        if self.done:
            return

        # STEP 0: clear mission
        if self.step == 0:
            self.get_logger().info("Clearing mission...")
            self.wp_clear_client.call_async(WaypointClear.Request())
            self.step = 1
            return

        # STEP 1: upload mission
        if self.step == 1:
            self.build_mission()

            self.get_logger().info("Uploading mission...")

            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = self.mission

            # self.wp_push_client.call_async(req)
            self.wp_future = self.wp_push_client.call_async(req)
            self.step = 1.5
            self.get_logger().info(f"Mission uploaded: {len(self.mission)} WPs")
            return

        if self.step == 1.5:
            if self.wp_future is None or not self.wp_future.done():
                return

            result = self.wp_future.result()

            # if not result or result.success_count == 0:
            # if not result or not result.success:
            if not result or result.wp_transfered == 0:
                self.get_logger().error("MISSION UPLOAD FAILED")
                self.step = 1
                return

            self.get_logger().info("MISSION UPLOADED SUCCESSFULLY")
            self.step = 2
            return

        # STEP 2: set AUTO mode (async)
        if self.step == 2:

            if self.current_state and not self.current_state.connected:
                self.get_logger().warn("Waiting for FCU connection...")
                return

            self.get_logger().info("Setting AUTO mode...")

            req = SetMode.Request()
            req.custom_mode = "AUTO"

            self.mode_future = self.mode_client.call_async(req)
            self.step = 2.5
            return

        # STEP 2.5: check AUTO result
        if self.step == 2.5:

            if self.mode_future is None or not self.mode_future.done():
                return

            result = self.mode_future.result()

            if not result or not result.mode_sent:
                self.get_logger().error("AUTO mode REJECTED")
                self.step = 2
                return

            self.get_logger().info("AUTO mode ACTIVE. Waiting before arming...")
            self.create_timer(2.0, lambda: None)
            self.step = 3
            return

        # STEP 3: arm (async)
        if self.step == 3:

            self.get_logger().info("Arming...")

            req = CommandBool.Request()
            req.value = True

            self.arm_future = self.arm_client.call_async(req)
            self.step = 3.5
            return

        # STEP 3.5: check arm result
        if self.step == 3.5:

            if self.arm_future is None or not self.arm_future.done():
                return

            result = self.arm_future.result()

            if not result or not result.success:
                self.get_logger().error("ARM FAILED")
                self.step = 3
                return

            self.get_logger().info("ARMED SUCCESSFULLY")
            req = WaypointSetCurrent.Request()
            req.wp_seq = 0
            self.wp_set_current_client.call_async(req)
            self.step = 4
            return

        # STEP 4: done
        if self.step == 4:
            self.get_logger().info("Mission started successfully.")
            self.done = True


def main():
    rclpy.init()
    node = PlaneMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()