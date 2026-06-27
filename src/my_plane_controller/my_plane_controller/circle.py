import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import time


class PlaneMission(Node):

    def __init__(self):
        super().__init__('plane_mission')

        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

        self.get_logger().info("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        self.get_logger().info("Connected to SITL")

        self.run()

    # -------------------------
    # HOME POSITION
    # -------------------------
    def get_home(self):
        msg = self.master.recv_match(type='HOME_POSITION', blocking=True)
        return msg.latitude / 1e7, msg.longitude / 1e7

    # -------------------------
    # MISSION BUILDER (FIXED)
    # -------------------------
    def build_mission(self, lat, lon):

        mission = []

        # 0 - TAKEOFF (must be first)
        mission.append({
            "cmd": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "params": [15, 0, 0, 0],  # IMPORTANT: pitch=15 deg
            "lat": lat,
            "lon": lon,
            "alt": 80
        })

        # 1 - CRITICAL: initial forward motion trigger (THIS FIXES YOUR ISSUE)
        mission.append({
            "cmd": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "params": [0, 0, 0, 0],
            "lat": lat + 0.0002,
            "lon": lon + 0.0002,
            "alt": 80
        })

        # 2 - Circle 1
        mission.append({
            "cmd": mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "params": [1, 200, 0, 0],
            "lat": lat + 0.001,
            "lon": lon + 0.001,
            "alt": 80
        })

        # 3 - Circle 2 (not concentric)
        mission.append({
            "cmd": mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "params": [1, 250, 0, 0],
            "lat": lat + 0.002,
            "lon": lon,
            "alt": 80
        })

        # 4 - LAND (home)
        mission.append({
            "cmd": mavutil.mavlink.MAV_CMD_NAV_LAND,
            "frame": mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            "params": [0, 0, 0, 0],
            "lat": lat,
            "lon": lon,
            "alt": 0
        })

        return mission

    # -------------------------
    # UPLOAD MISSION (SAFE)
    # -------------------------
    def upload_mission(self, mission):

        self.get_logger().info(f"Uploading mission items: {len(mission)}")

        self.master.waypoint_clear_all_send()
        time.sleep(0.5)

        self.master.waypoint_count_send(len(mission))

        for i, wp in enumerate(mission):

            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                self.master.target_system,
                self.master.target_component,
                i,
                wp["frame"],
                wp["cmd"],
                0,   # current
                1,   # autocontinue
                wp["params"][0],
                wp["params"][1],
                wp["params"][2],
                wp["params"][3],
                int(wp["lat"] * 1e7),
                int(wp["lon"] * 1e7),
                float(wp["alt"])
            )

            self.master.mav.send(msg)
            time.sleep(0.05)

        self.get_logger().info("Mission uploaded")

    # -------------------------
    # FORCE MISSION START (CRITICAL FIX)
    # -------------------------
    def start_mission(self):

        self.get_logger().info("Forcing mission start...")

        # Set mission index
        self.master.mav.mission_set_current_send(
            self.master.target_system,
            self.master.target_component,
            0
        )

        time.sleep(1)

        # Force AUTO mission start trigger
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,
            0, 0, 0, 0, 0, 0, 0
        )

    # -------------------------
    # ARM + MODE + START
    # -------------------------
    def arm_and_start(self):

        self.get_logger().info("Arming...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()

        time.sleep(1)
        self.master.set_mode("TAKEOFF") #mode id 13
        time.sleep(1)

        self.get_logger().info("AUTO mode...")
        self.master.set_mode_auto()

        time.sleep(2)

        self.start_mission()

    # -------------------------
    # RUN
    # -------------------------
    def run(self):

        lat, lon = self.get_home()

        mission = self.build_mission(lat, lon)

        self.upload_mission(mission)

        self.arm_and_start()


def main(args=None):
    rclpy.init(args=args)
    node = PlaneMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()