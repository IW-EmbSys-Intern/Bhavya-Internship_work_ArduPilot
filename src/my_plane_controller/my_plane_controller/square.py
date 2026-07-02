import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, WaypointSetCurrent
from mavros_msgs.msg import Waypoint, State, WaypointList
from std_msgs.msg import Float64


class PlaneSquareMission(Node):
    def __init__(self):
        super().__init__('plane_square_mission')

        # MAVROS services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.wp_set_current_client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')

        # State management
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        self.step = 0
        self.takeoff_announced = False
        self.mission_completed = False

        self.current_alt = 0.0
        self.current_wp_idx = 0
        self.current_state = None

        self.waiting_for_future = False

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, qos_profile_sensor_data)
        self.alt_sub = self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.alt_cb, qos_profile_sensor_data)
        self.wp_sub = self.create_subscription(WaypointList, '/mavros/mission/waypoints', self.wp_cb, qos_profile_sensor_data)

        # Home location
        self.home_lat = -35.35610226
        self.home_lon = 149.16162239

        self.mission = []

        self.get_logger().info("Waiting for MAVROS services...")
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.wp_clear_client.wait_for_service()
        self.wp_push_client.wait_for_service()

    def state_cb(self, msg):
        self.current_state = msg

    def alt_cb(self, msg):
        self.current_alt = msg.data

    def wp_cb(self, msg):
        for i, wp in enumerate(msg.waypoints):
            if wp.is_current:
                self.current_wp_idx = i
                break

    def create_wp(self, lat, lon, alt, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0):
        wp = Waypoint()
        wp.frame = 3  # GLOBAL_RELATIVE_ALT
        wp.command = command
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = p1
        wp.param2 = p2
        wp.param3 = p3
        wp.param4 = p4
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        return wp

    def build_mission(self):
        flight_alt = 20.0

        p1_lat, p1_lon = -35.35610226, 149.16162239
        p2_lat, p2_lon = -35.35677599, 149.16567019
        p3_lat, p3_lon = -35.36203112, 149.16679952
        p4_lat, p4_lon = -35.36307545, 149.16452777

        self.mission = [
            # WP 0: Home Position Reference
            self.create_wp(self.home_lat, self.home_lon, 0.0, 16),

            # WP 1: TAKEOFF - The critical starting point
            self.create_wp(self.home_lat, self.home_lon, flight_alt, 22, p1=15.0),

            # WPs 2-5: The 4 square tracks kept flat at 20m
            self.create_wp(p1_lat, p1_lon, flight_alt, 16),  
            self.create_wp(p2_lat, p2_lon, flight_alt, 16),  
            self.create_wp(p3_lat, p3_lon, flight_alt, 16),  
            self.create_wp(p4_lat, p4_lon, flight_alt, 16),  

            # WP 6: DO_LAND_START 
            self.create_wp(self.home_lat, self.home_lon, flight_alt, 190),

            # WP 7: Return above home base
            self.create_wp(self.home_lat, self.home_lon, flight_alt, 16),

            # WP 8: LAND sequence
            self.create_wp(self.home_lat, self.home_lon, 0.0, 21)
        ]

    def monitor_flight(self):
        """Monitors real-time telemetry positions safely during execution"""
        if not self.takeoff_announced:
            if self.current_wp_idx == 1 and self.current_alt >= 19.5:
                self.get_logger().info("TAKEOFF COMPLETE")
                self.takeoff_announced = True

        if self.current_wp_idx == 8:
            self.get_logger().info("Arrived at Home coordinates. Starting Landing Sequence...")
            self.mission_completed = True

    def state_machine_loop(self):
        if self.step == 4:
            self.monitor_flight()
            return

        if self.mission_completed or self.waiting_for_future:
            return
        
        if self.current_state is None:
            return

        # STEP 0: Clear FCU Mission
        if self.step == 0:
            self.get_logger().info("Clearing old missions...")
            self.waiting_for_future = True
            future = self.wp_clear_client.call_async(WaypointClear.Request())
            future.add_done_callback(self.clear_callback)
            return

        # STEP 1: Build & Upload Mission
        if self.step == 1:
            self.build_mission()
            self.get_logger().info("Uploading square mission profile...")
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = self.mission
            self.waiting_for_future = True
            future = self.wp_push_client.call_async(req)
            future.add_done_callback(self.upload_callback)
            self.get_logger().info("Uploaded square mission")
            return

        # STEP 2: Switch to AUTO mode
        if self.step == 2:
            if self.current_state and not self.current_state.connected:
                return
            self.get_logger().info("Setting AUTO flight mode...")
            req = SetMode.Request()
            req.custom_mode = "AUTO"
            self.waiting_for_future = True
            future = self.mode_client.call_async(req)
            future.add_done_callback(self.mode_callback)
            self.get_logger().info(f"Mode: {self.current_state.mode}")
            return

        # STEP 3: Arm Vehicle
        if self.step == 3:
            self.get_logger().info("Requesting FCU Arming status...")
            req = CommandBool.Request()
            req.value = True
            self.waiting_for_future = True
            future = self.arm_client.call_async(req)
            future.add_done_callback(self.arm_callback)
            self.get_logger().info(f"Armed: {self.current_state.armed}")
            return

    def clear_callback(self, future):
        self.waiting_for_future = False
        self.step = 1

    def upload_callback(self, future):
        self.waiting_for_future = False
        result = future.result()
        if result and result.wp_transfered > 0:
            self.step = 2
        else:
            self.get_logger().error("Upload failed, retrying...")
            self.step = 1

    def mode_callback(self, future):
        self.waiting_for_future = False
        result = future.result()
        if result and result.mode_sent:
            self.step = 3
        else:
            self.step = 2

    def arm_callback(self, future):
        self.waiting_for_future = False
        result = future.result()
        if result and result.success:
            self.get_logger().info("Armed. Executing Takeoff...")
            req_set_wp = WaypointSetCurrent.Request()
            req_set_wp.wp_seq = 1  # <--- FIXED: Changed from 0 to 1 to force execution of WP 1 (Takeoff)
            self.wp_set_current_client.call_async(req_set_wp)
            self.step = 4
        else:
            self.step = 3


def main():
    rclpy.init()
    node = PlaneSquareMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()