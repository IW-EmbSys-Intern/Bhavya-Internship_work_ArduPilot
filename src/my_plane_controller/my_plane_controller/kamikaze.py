#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Import QoS profiles to fix the hidden ROS2 connection drop
from rclpy.qos import qos_profile_sensor_data

# MAVROS Services & Messages
from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, CommandInt, CommandLong
from mavros_msgs.msg import Waypoint, State
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

# Your Custom YOLO Message
from my_plane_controller.msg import TargetDetection

class KamikazeMissionControl(Node):
    def __init__(self):
        super().__init__('kamikaze_mission_control')

        # 1. MAVROS Service Clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.command_int_client = self.create_client(CommandInt,'/mavros/cmd/command_int')

        # 2. Publishers & Subscribers
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.global_pos_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.yolo_sub = self.create_subscription(TargetDetection, '/detection/target', self.yolo_callback, 10)
        
        # ─── UPDATED: Added qos_profile_sensor_data here ──────────────────────
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos_profile=qos_profile_sensor_data
        )
        self.global_gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.global_gps_cb, qos_profile=qos_profile_sensor_data
        )
        # ──────────────────────────────────────────────────────────────────────

        # 3. State Machine Variables
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        self.step = 0
        self.current_state = None
        self.yolo_triggered = False
        self.tracking_mode_type = "NONE"

        self.current_local_pose = None
        self.current_global_gps = None
        self.current_yaw = 0.0

        self.wp_future = None
        self.mode_future = None
        self.arm_future = None
        self.guided_target = None

        self.home_lat = -35.36325846
        self.home_lon = 149.16523276

        self.get_logger().info("🛫 Kamikaze Controller waiting for MAVROS services...")
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.wp_clear_client.wait_for_service()
        self.wp_push_client.wait_for_service()

        self.command_int_client.wait_for_service()

        self.speed_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.speed_client.wait_for_service()        

        self.fast_track_timer = self.create_timer(0.05, self.fast_tracking_loop)
        self.target_pose_local = None 
        self.target_pose_global = None

    # Keep all other methods (state_cb, local_pos_cb, global_gps_cb, yolo_callback, etc.) exactly the same as before...

    def local_pos_cb(self, msg):
        self.current_local_pose = msg

        # convert quaternion → yaw
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def shift_forward(self, lat, lon, distance_m=5.0):
        dx = distance_m * math.cos(self.current_yaw)
        dy = distance_m * math.sin(self.current_yaw)

        dlat = dy / 111111.0
        dlon = dx / (111111.0 * math.cos(math.radians(lat)))

        return lat + dlat, lon + dlon

    def state_cb(self, msg):
        self.current_state = msg

    def local_pos_cb(self, msg):
        self.current_local_pose = msg

    def global_gps_cb(self, msg):
        self.current_global_gps = msg

    def set_speed(self, speed_mps):
        req = CommandLong.Request()
        
        req.command = 178  # MAV_CMD_DO_CHANGE_SPEED
        req.confirmation = 0

        req.param1 = 1.0   # speed type: 1 = airspeed
        req.param2 = speed_mps
        req.param3 = -1.0  # throttle (ignore)
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0

        self.speed_client.call_async(req)

    def yolo_callback(self, msg):
        if msg.target_found:
            self.get_logger().info(
                f"📥 Received Tracker Update -> Target Pixel Coords: X: {msg.pixel_x:.1f}, Y: {msg.pixel_y:.1f}"
            )
        else:
            self.get_logger().info("📥 Received Tracker Update -> Scanning... No target visible.", throttle_duration_sec=2.0)
            return

        # # Trigger intercept if target is seen during search phase (Step 4)                 DEBUG!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        # if msg.target_found and self.step == 4 and not self.yolo_triggered:
        #     self.get_logger().warn("🧪 DEBUG: Sending fixed GUIDED test waypoint")
        #     test_lat = -35.36526256
        #     test_lon = 149.16525587
        #     test_alt = 2.0
        #     # self.send_guided_goto(test_lat, test_lon, test_alt)
        #     self.guided_target = (test_lat, test_lon, test_alt)
        #     self.yolo_triggered = True
        #     self.step = 5

        ####################################################################################################
            
#         # CRITICAL ATTEMPT 1: Try Local Coordinates Frame
        if msg.gps_lat != 0.0 and msg.gps_lon != 0.0:
            self.get_logger().warn("🎯 VEHICLE SPOTTED -> SENDING GUIDED COMMAND")

            self.target_lat = msg.gps_lat
            self.target_lon = msg.gps_lon

# # # ***********************************************************************************************
# #             base_lat = msg.gps_lat
# #             base_lon = msg.gps_lon

# #             # shift 5 meters forward in aircraft heading
# #             self.target_lat, self.target_lon = self.shift_forward(
# #                 base_lat,
# #                 base_lon,
# #                 distance_m=5.0
# #             )

# #             self.target_alt = 4.0  # or your preferred altitude

# #             self.guided_target = (self.target_lat, self.target_lon, self.target_alt)
# # # ***********************************************************************************************

            self.get_logger().info(f"{self.target_lat}, {self.target_lon}")

            # self.send_guided_goto(self.target_lat, self.target_lon, 0.0)
            self.guided_target = (self.target_lat, self.target_lon, 2.0)

            self.tracking_mode_type = "GLOBAL"
            self.yolo_triggered = True
            # self.step = 5
            self.yolo_triggered = True
        else:
            self.get_logger().error("❌ Invalid GPS from YOLO (0,0) — ignoring target")
        
        #####################################################################################################################

    def meters_to_latlon(self, dx, dy):
        lat = self.home_lat + (dy / 111111.0)
        lon = self.home_lon + (dx / (111111.0 * math.cos(math.radians(self.home_lat + 1e-6))))
        return lat, lon
    
    def send_guided_goto(self, lat, lon, alt=0.0):
        req = CommandInt.Request()

        req.broadcast = False
        req.frame = 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT  

        req.command = 192  # MAV_CMD_DO_REPOSITION ⭐

        req.current = 0
        req.autocontinue = True

        req.param1 = -1.0   # ground speed (default)
        req.param2 = 0.0    # no change to loiter radius
        req.param3 = 0.0
        req.param4 = 0.0

        req.x = int(lat * 1e7)
        req.y = int(lon * 1e7)
        req.z = float(alt)

        self.command_int_client.call_async(req)

    def fast_tracking_loop(self):
        if self.step == 6:
            # Stream along the chosen channel that successfully locked data
            if self.tracking_mode_type == "LOCAL" and self.target_pose_local is not None:
                self.target_pose_local.header.stamp = self.get_clock().now().to_msg()
                self.pos_pub.publish(self.target_pose_local)
                
            elif self.tracking_mode_type == "GLOBAL" and self.target_pose_global is not None:
                self.target_pose_global.header.stamp = self.get_clock().now().to_msg()
                self.global_pos_pub.publish(self.target_pose_global)

    def create_wp(self, lat, lon, alt, command, is_current=False):
        wp = Waypoint()
        wp.frame = 3  
        wp.command = command
        wp.is_current = is_current
        wp.autocontinue = True
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = alt
        return wp

    def build_search_mission(self):
        alt = 25.0  
        p_takeoff = (0, 0)
        p_search1 = (50, 800)
        p_search2 = (200, 200)
        p_search3 = (300, 300)
        p_search4 = (-150, -150)
        p_search5 = (-100, -100)

        lat_tk, lon_tk = self.meters_to_latlon(*p_takeoff)
        lat1, lon1 = self.meters_to_latlon(*p_search1)
        lat2, lon2 = self.meters_to_latlon(*p_search2)
        lat3, lon3 = self.meters_to_latlon(*p_search3)
        lat4, lon4 = self.meters_to_latlon(*p_search4)
        lat5, lon5 = self.meters_to_latlon(*p_search5)

        return [
            self.create_wp(lat_tk, lon_tk, alt, 16),                  
            self.create_wp(lat1, lon1, alt, 22, is_current=True),     
            self.create_wp(lat2, lon2, alt, 16),
            self.create_wp(lat3, lon3, alt, 16),
            self.create_wp(lat4, lon4, alt, 16),
            self.create_wp(lat5, lon5, alt, 21),
            self.create_wp(lat_tk, lon_tk, alt, 16)                    
        ]

    def state_machine_loop(self):
        if self.current_state and not self.current_state.connected:
            self.get_logger().warn("Waiting for FCU connection...", throttle_duration_sec=3.0)
            return

        # STEP 0: Clear background mission entries
        if self.step == 0:
            self.get_logger().info("Clearing old flight plans...")
            self.wp_clear_client.call_async(WaypointClear.Request())
            self.step = 1
            return

        # STEP 1: Upload Takeoff & Search pattern
        if self.step == 1:
            mission_wps = self.build_search_mission()
            self.get_logger().info(f"Uploading takeoff and search pattern ({len(mission_wps)} WPs)...")
            
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = mission_wps
            self.wp_future = self.wp_push_client.call_async(req)
            self.step = 1.5
            return

        # STEP 1.5: Validate successful upload
        if self.step == 1.5:
            if self.wp_future is None or not self.wp_future.done():
                return
            result = self.wp_future.result()
            if not result or result.wp_transfered == 0:
                self.get_logger().error("Mission plan upload rejected! Retrying...")
                self.step = 1
                return
            self.get_logger().info("Flight plan successfully synchronized with ArduPilot.")
            self.step = 2
            return

        # STEP 2: Switch to AUTO flight mode
        if self.step == 2:
            self.get_logger().info("Requesting switching to AUTO mode...")
            req = SetMode.Request()
            req.custom_mode = "AUTO"
            self.mode_future = self.mode_client.call_async(req)
            self.step = 2.5
            return

        # STEP 2.5: Validate AUTO activation
        if self.step == 2.5:
            if self.mode_future is None or not self.mode_future.done():
                return
            result = self.mode_future.result()
            if not result or not result.mode_sent:
                self.get_logger().error("AUTO mode initialization failed. Retrying...")
                self.step = 2
                return
            self.get_logger().info("AUTO mode engaged.")
            self.step = 3
            return

        # STEP 3: Arm aircraft to trigger Takeoff
        if self.step == 3:
            self.get_logger().info("Arming propulsion system...")
            req = CommandBool.Request()
            req.value = True
            self.arm_future = self.arm_client.call_async(req)
            self.step = 3.5
            return

        # STEP 3.5: Confirm Arming
        if self.step == 3.5:
            if self.arm_future is None or not self.arm_future.done():
                return
            result = self.arm_future.result()
            if not result or not result.success:
                self.get_logger().error("Arming execution failed. Re-trying arm...")
                self.step = 3
                return
            self.get_logger().info("🚀 ARMED! Running automated takeoff sequence...")
            self.step = 4
            return

        # STEP 4: In-flight searching loop
        if self.step == 4:
            if self.yolo_triggered:
                self.get_logger().info("YOLO TRIGGERED GUIDED!!!!!")
                self.step = 5
            else:
                self.get_logger().info("✈️ Airborne. Flying search coordinates... YOLO stream active.", throttle_duration_sec=5.0)
                return

        # STEP 5: Swapping vehicle mode to GUIDED
        if self.step == 5:
            if self.current_state.mode != "GUIDED":
                self.get_logger().warn(f"Swapping vehicle mode to GUIDED (Active Channel: {self.tracking_mode_type})...")
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self.mode_client.call_async(req)
                self.step = 5.5
            return

        if self.step == 5.5:
            if self.current_state.mode == "GUIDED":
                self.get_logger().info("GUIDED mode confirmed. Sending target...")
                if self.guided_target is not None:
                    self.get_logger().info("TARGET SENT")
                    lat, lon, alt = self.guided_target
                    self.send_guided_goto(lat, lon, alt)

                self.step = 6
            return

def main(args=None):
    rclpy.init(args=args)
    node = KamikazeMissionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()