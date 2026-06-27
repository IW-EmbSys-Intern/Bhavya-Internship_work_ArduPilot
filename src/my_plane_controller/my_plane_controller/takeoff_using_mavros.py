import sys
import time
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool

class PlaneAutoTakeoff(Node):
    def __init__(self):
        super().__init__('plane_auto_takeoff_node')
        
        # Initialize service clients
        self.wp_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        # Wait for all services to be available
        self.get_logger().info('Waiting for MAVROS services...')
        while not self.wp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/mission/push not available, waiting...')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/set_mode not available, waiting...')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/mavros/cmd/arming not available, waiting...')
            
        self.get_logger().info('All MAVROS services available. Node initialized.')

    def run_sequence(self):
        # --- STEP 1: Upload Takeoff Mission ---
        if not self.upload_takeoff_mission():
            self.get_logger().error('Mission upload failed. Aborting.')
            return

        # --- STEP 2: Switch to AUTO Mode ---
        if not self.change_mode('AUTO'):
            self.get_logger().error('Failed to set AUTO mode. Aborting.')
            return

        # --- STEP 3: Arm the Aircraft ---
        if not self.arm_vehicle():
            self.get_logger().error('Arming failed. Aborting.')
            return

        # --- STEP 4: Sleep while taking off ---
        sleep_duration = 15.0  # Adjust this value (seconds) depending on how high you want it to climb
        self.get_logger().info(f'Sleeping for {sleep_duration} seconds to allow takeoff climb...')
        time.sleep(sleep_duration)

        # --- STEP 5: Switch to GUIDED Mode ---
        if self.change_mode('GUIDED'):
            self.get_logger().info('Successfully switched to GUIDED mode! Ready for guided commands.')
        else:
            self.get_logger().error('Failed to switch to GUIDED mode.')

    def upload_takeoff_mission(self):
        req = WaypointPush.Request()
        req.start_index = 0
        
        # Waypoint 0: Fixed Home Position definition for ArduPilot
        wp0 = Waypoint()
        wp0.frame = 0          
        wp0.command = 16       
        wp0.is_current = False
        wp0.autocontinue = True
        wp0.x_lat = 0.0        
        wp0.y_long = 0.0       
        wp0.z_alt = 0.0
        req.waypoints.append(wp0)
        
        # Waypoint 1: NAV_TAKEOFF Command
        wp1 = Waypoint()
        wp1.frame = 3          # MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp1.command = 22        # MAV_CMD_NAV_TAKEOFF
        wp1.is_current = True
        wp1.autocontinue = True
        wp1.param1 = 15.0       # Minimum pitch angle (degrees)
        wp1.z_alt = 40.0        # Target takeoff altitude (meters)
        wp1.x_lat = 0.0         
        wp1.y_long = 0.0        
        req.waypoints.append(wp1)
        
        self.get_logger().info('Uploading takeoff mission...')
        future = self.wp_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res and res.success:
            self.get_logger().info(f'Mission uploaded! Items sent: {res.wp_transfered}')
            return True
        return False

    def change_mode(self, mode_name):
        req = SetMode.Request()
        req.custom_mode = mode_name
        
        self.get_logger().info(f'Switching flight mode to {mode_name}...')
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res and res.mode_sent:
            self.get_logger().info(f'Mode change to {mode_name} accepted.')
            return True
        return False

    def arm_vehicle(self):
        req = CommandBool.Request()
        req.value = True
        
        self.get_logger().info('Sending arming command...')
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        res = future.result()
        if res and res.success:
            self.get_logger().info('Vehicle ARMED! Takeoff sequence initiated.')
            return True
        return False

def main(args=None):
    rclpy.init(args=args)
    
    node = PlaneAutoTakeoff()
    node.run_sequence()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()