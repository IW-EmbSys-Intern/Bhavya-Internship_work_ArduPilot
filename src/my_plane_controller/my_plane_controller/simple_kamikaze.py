#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from rclpy.qos import qos_profile_sensor_data

from mavros_msgs.srv import CommandBool, SetMode, WaypointPush, WaypointClear, CommandInt, CommandLong
from mavros_msgs.msg import Waypoint, State, ExtendedState, GlobalPositionTarget
from geometry_msgs.msg import TwistStamped, PoseStamped
from sensor_msgs.msg import NavSatFix, CameraInfo
from geographic_msgs.msg import GeoPoseStamped
from my_plane_controller.msg import TargetDetection, TargetGPS


class KamikazeMissionControl(Node):
    def __init__(self):
        super().__init__('kamikaze_mission_control')

        # ---------- MAVROS Service Clients ----------
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.wp_clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.wp_push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.command_int_client = self.create_client(CommandInt, '/mavros/cmd/command_int')
        self.speed_client = self.create_client(CommandLong, '/mavros/cmd/command')

        # ---------- Publishers ----------
        self.vel_pub = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.global_pos_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        self.raw_global_pub = self.create_publisher(GlobalPositionTarget, '/mavros/setpoint_raw/global', 10)

        # ---------- Subscribers ----------
        self.ext_state_sub = self.create_subscription(
            ExtendedState, "/mavros/extended_state", self.ext_state_cb, qos_profile_sensor_data)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_cb, 10)
        self.yolo_sub = self.create_subscription(TargetGPS, '/detection/target_gps', self.yolo_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_cb, 10)
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_pos_cb, qos_profile=qos_profile_sensor_data)
        self.global_gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.global_gps_cb, qos_profile=qos_profile_sensor_data)
        

        # ---------- State machine ----------
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        self.fast_track_timer = self.create_timer(0.05, self.fast_tracking_loop)  # 20 Hz

        self.step = 0
        self.current_state = None
        self.extended_state = None

        self.current_local_pose = None
        self.current_global_gps = None
        self.current_yaw = 0.0

        self.wp_future = None
        self.mode_future = None
        self.arm_future = None

        self.camera_info = None

        # Target tracking
        self.yolo_triggered = False
        self.tracking_mode_type = "NONE"
        self.target_lat = None
        self.target_lon = None
        self.target_alt = 5.0
        self.target_pose_global = None
        self.target_pose_local = None

        # Terminal-phase bookkeeping
        self.terminal_speed_sent = False
        self.PHASE_B_RANGE_M = 40.0     # switch to terminal/forced-speed inside this range
        self.IMPACT_RANGE_M = 3.0       # trigger pull-up inside this range
        self.CRUISE_SPEED_MPS = 18.0
        self.TERMINAL_SPEED_MPS = 10.0

        # Simple EMA filter state for noisy geolocation
        self.EMA_ALPHA = 0.3
        self.filtered_lat = None
        self.filtered_lon = None

        self.home_lat = -35.36325846
        self.home_lon = 149.16523276

        self.get_logger().info("🛫 Kamikaze Controller waiting for MAVROS services...")
        self.arm_client.wait_for_service()
        self.mode_client.wait_for_service()
        self.wp_clear_client.wait_for_service()
        self.wp_push_client.wait_for_service()
        self.command_int_client.wait_for_service()
        self.speed_client.wait_for_service()
        self.get_logger().info("✅ All MAVROS services available.")

    # ---------------- Callbacks ----------------

    def local_pos_cb(self, msg):
        self.current_local_pose = msg
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def state_cb(self, msg):
        self.current_state = msg

    def ext_state_cb(self, msg):
        self.extended_state = msg
        self.get_logger().info(
            f"Extended state received: landed_state={msg.landed_state}",
            throttle_duration_sec=5.0
        )

    def global_gps_cb(self, msg):
        self.current_global_gps = msg

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def set_airspeed(self, speed_mps: float):
        if not self.speed_client.service_is_ready():
            self.get_logger().warn('cmd/command service not ready, skipping speed set')
            return

        req = CommandLong.Request()
        req.command = 178        # MAV_CMD_DO_CHANGE_SPEED
        req.param1 = 0.0         # 0 = airspeed
        req.param2 = float(speed_mps)
        req.param3 = -1.0        # no throttle change
        req.param4 = 0.0

        future = self.speed_client.call_async(req)
        future.add_done_callback(self._on_speed_response)

    def _on_speed_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Airspeed set (result={resp.result})')
            else:
                self.get_logger().warn('Airspeed change rejected')
        except Exception as e:
            self.get_logger().error(f'Airspeed service call failed: {e}')

    def yolo_callback(self, msg):
        # if not msg.target_found:
        #     self.get_logger().info("📥 Scanning... no target visible.", throttle_duration_sec=2.0)
        #     return

        # if msg.gps_lat == 0.0 and msg.gps_lon == 0.0:
        #     self.get_logger().error("❌ Invalid GPS from detector (0,0) — ignoring.")
        #     return
        
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().error("❌ Invalid GPS from geolocator (0,0) — ignoring.")
            return

        # ---- EMA filter to smooth noisy per-frame geolocation ----
        if self.filtered_lat is None:
            # self.filtered_lat = msg.gps_lat
            # self.filtered_lon = msg.gps_lon
            self.filtered_lat = msg.latitude
            self.filtered_lon = msg.longitude
        else:
            a = self.EMA_ALPHA
            # self.filtered_lat = a * msg.gps_lat + (1 - a) * self.filtered_lat
            # self.filtered_lon = a * msg.gps_lon + (1 - a) * self.filtered_lon
            self.filtered_lat = a * msg.latitude + (1 - a) * self.filtered_lat
            self.filtered_lon = a * msg.longitude + (1 - a) * self.filtered_lon

        self.target_lat = self.filtered_lat
        self.target_lon = self.filtered_lon

        geopose = GeoPoseStamped()
        geopose.header.frame_id = "map"
        geopose.pose.position.latitude = self.target_lat
        geopose.pose.position.longitude = self.target_lon
        geopose.pose.position.altitude = self.target_alt

        self.target_pose_global = geopose
        self.tracking_mode_type = "GLOBAL"

        self.get_logger().info(
            f"🎯 Target update -> lat={self.target_lat:.7f}, lon={self.target_lon:.7f}",
            throttle_duration_sec=1.0
        )

        # This is the event that actually forces mode-switch — not a polled flag.
        if not self.yolo_triggered:
            self.get_logger().warn("🎯 TARGET LOCKED -> forcing GUIDED transition")
            self.yolo_triggered = True
            self.step = 5.1  # jump in regardless of current step (4 or 6)

    # ---------------- Helpers ----------------

    def meters_to_latlon(self, dx, dy):
        lat = self.home_lat + (dy / 111111.0)
        lon = self.home_lon + (dx / (111111.0 * math.cos(math.radians(self.home_lat + 1e-6))))
        return lat, lon

    def haversine_dist(self, lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
        return 2 * R * math.asin(math.sqrt(a))

    def set_speed(self, speed_mps):
        req = CommandLong.Request()
        req.command = 178  # MAV_CMD_DO_CHANGE_SPEED
        req.confirmation = 0
        req.param1 = 1.0   # speed type: 1 = airspeed
        req.param2 = speed_mps
        req.param3 = -1.0  # throttle: ignore
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        self.speed_client.call_async(req)
        self.get_logger().info(f"⚡ Requested airspeed change -> {speed_mps} m/s")

    def send_guided_goto(self, lat, lon, alt=0.0):
        """One-shot reposition command. Used only as a coarse nudge into GUIDED —
        NOT the primary tracking mechanism. fast_tracking_loop() does the real work."""
        req = CommandInt.Request()
        req.broadcast = False
        req.frame = 6  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        req.command = 192  # MAV_CMD_DO_REPOSITION
        req.current = 0
        req.autocontinue = True
        req.param1 = -1.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.x = int(lat * 1e7)
        req.y = int(lon * 1e7)
        req.z = float(alt)
        self.command_int_client.call_async(req)

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
        alt = 50.0
        lat_tk, lon_tk = self.meters_to_latlon(0, 0)
        return [
            self.create_wp(lat_tk, lon_tk, alt, 16),
            self.create_wp(-35.35167973, 149.16520960, alt, 22, is_current=True),
            self.create_wp(-35.35985380, 149.16593628, alt, 16),
            self.create_wp(-35.36321719, 149.16872562, alt, 16),
            self.create_wp(-35.36555750, 149.16587950, alt, 16),
            self.create_wp(-35.36586979, 149.16529781, alt, 16),
            self.create_wp(-35.36555540, 149.16525650, alt, 16),
            self.create_wp(-35.36333009, 149.16522300, 0.0, 21),  # Land
        ]

    # ---------------- 20 Hz fast loop: does the actual intercept work ----------------

    def fast_tracking_loop(self):
        if self.step != 6.1:
            return
        if self.target_pose_global is None or self.current_global_gps is None:
            return

        # 1. Calculate distance to target
        r = self.haversine_dist(
            self.current_global_gps.latitude, self.current_global_gps.longitude,
            self.target_lat, self.target_lon
        )

        # 2. Build the explicit raw target message
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # FORCE FRAME 6: MAV_FRAME_GLOBAL_RELATIVE_ALT (Altitude is meters above home/ground)
        msg.coordinate_frame = 6 
        
        msg.latitude = self.target_lat
        msg.longitude = self.target_lon
        
        # Set a safe cruise approach altitude (e.g., 50m above ground) until you are ready to dive
        msg.altitude = 50.0 
        
        # Ignore velocity/acceleration/yaw rates, track position only
        msg.type_mask = 4087 

        # 3. Publish directly to the raw endpoint
        self.raw_global_pub.publish(msg)

        # 4. Handle speed/terminal phases based on range
        if r <= self.PHASE_B_RANGE_M:
            if not self.terminal_speed_sent:
                self.get_logger().warn(f"🔥 TERMINAL PHASE (range={r:.1f} m) — forcing strike speed")
                self.set_speed(self.TERMINAL_SPEED_MPS)
                self.terminal_speed_sent = True

        if r < self.IMPACT_RANGE_M:
            self.get_logger().warn(f"💥 IMPACT RANGE (range={r:.1f} m) — pulling up")
            self.step = 7

    # ---------------- 1 Hz state machine ----------------

    def state_machine_loop(self):
        if self.current_state is None or not self.current_state.connected:
            self.get_logger().warn("Waiting for FCU connection...", throttle_duration_sec=3.0)
            return

        # STEP 0: Clear old mission
        if self.step == 0:
            self.get_logger().info("Clearing old flight plans...")
            self.wp_clear_client.call_async(WaypointClear.Request())
            self.step = 1
            return

        # STEP 1: Upload search mission
        if self.step == 1:
            mission_wps = self.build_search_mission()
            self.get_logger().info(f"Uploading takeoff and search pattern ({len(mission_wps)} WPs)...")
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = mission_wps
            self.wp_future = self.wp_push_client.call_async(req)
            self.step = 1.5
            return

        if self.step == 1.5:
            if self.wp_future is None or not self.wp_future.done():
                return
            result = self.wp_future.result()
            if not result or result.wp_transfered == 0:
                self.get_logger().error("Mission upload rejected! Retrying...")
                self.step = 1
                return
            self.get_logger().info("Flight plan synchronized with ArduPilot.")
            self.step = 2
            return

        # STEP 2: AUTO mode
        if self.step == 2:
            self.get_logger().info("Requesting AUTO mode...")
            req = SetMode.Request()
            req.custom_mode = "AUTO"
            self.mode_future = self.mode_client.call_async(req)
            self.step = 2.5
            return

        if self.step == 2.5:
            if self.mode_future is None or not self.mode_future.done():
                return
            result = self.mode_future.result()
            if not result or not result.mode_sent:
                self.get_logger().error("AUTO mode failed. Retrying...")
                self.step = 2
                return
            self.get_logger().info("AUTO mode engaged.")
            self.step = 3
            return

        # STEP 3: Arm
        if self.step == 3:
            self.get_logger().info("Arming...")
            req = CommandBool.Request()
            req.value = True
            self.arm_future = self.arm_client.call_async(req)
            self.step = 3.5
            return

        if self.step == 3.5:
            if self.arm_future is None or not self.arm_future.done():
                return
            result = self.arm_future.result()
            if not result or not result.success:
                self.get_logger().error("Arming failed. Retrying...")
                self.step = 3
                return
            self.get_logger().info("🚀 ARMED! Executing takeoff.")
            self.step = 4
            return

        # STEP 4 / STEP 6: Idle search states.
        # These NO LONGER auto-advance based on yolo_triggered — that transition
        # is now event-driven directly from yolo_callback() to avoid the dead-end
        # bug where step 4 only checks the flag once and then gets stuck at 6.
        if self.step == 4:
            self.get_logger().info("✈️ Airborne, climbing to search altitude...", throttle_duration_sec=5.0)
            return

        if self.step == 6:
            self.get_logger().info("✈️ Flying search pattern, YOLO active.", throttle_duration_sec=5.0)
            return

        # STEP 5.1: Force GUIDED mode (triggered by yolo_callback, reachable from any step)
        if self.step == 5.1:
            if self.current_state.mode != "GUIDED":
                self.get_logger().warn("Swapping vehicle mode to GUIDED...")
                req = SetMode.Request()
                req.custom_mode = "GUIDED"
                self.mode_client.call_async(req)
                return
            self.get_logger().info("GUIDED confirmed. Sending initial reposition + entering intercept.")
            if self.target_lat is not None:
                self.send_guided_goto(self.target_lat, self.target_lon, self.target_alt)
                # self.set_speed(self.CRUISE_SPEED_MPS)
                self.set_speed(10.0)
                self.set_airspeed(10.0)
            self.terminal_speed_sent = False
            self.step = 6.1
            return

        # STEP 6.1: Committed intercept — all real work happens in fast_tracking_loop()
        if self.step == 6.1:
            self.get_logger().info("🎯 Intercepting target...", throttle_duration_sec=2.0)
            return

        # STEP 7: Pull-up after impact range reached
        if self.step == 7:
            self.get_logger().warn("Executing pull-up / RTL.")
            req = SetMode.Request()
            req.custom_mode = "RTL"
            self.mode_client.call_async(req)
            self.step = 8
            return

        # STEP 8: Done — idle
        if self.step == 8:
            self.get_logger().info("Mission complete, RTL commanded.", throttle_duration_sec=10.0)
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