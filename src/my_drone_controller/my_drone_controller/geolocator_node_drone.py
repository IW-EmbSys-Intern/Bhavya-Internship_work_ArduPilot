#!/usr/bin/env python3
"""
geolocator_node.py

Converts YOLO pixel-space detections into GPS coordinates for a fixed-wing
UAV (Skywalker X8) in a ROS2 + Gazebo Harmonic + ArduPilot SITL simulation.

Pipeline
--------
1. Pixel (u, v)  -->  normalized ray in the camera OPTICAL frame (pinhole model)
2. Optical frame -->  camera LINK frame (fixed axis-convention rotation)
3. Camera LINK   -->  aircraft BODY frame (fixed extrinsic transform, from SDF)
4. Aircraft BODY -->  WORLD/ENU (map) frame (from MAVROS orientation quaternion)
5. Intersect the world-frame ray with the ground plane z = 0
6. Convert the resulting ENU offset into a GPS delta (flat-earth approximation)
   applied on top of the aircraft's *current* global fix.

Design note on step 6
----------------------
TargetDetectionDrone.msg has no header/timestamp, and MAVROS does not publish a
dedicated "home lat/lon" topic by default. /mavros/local_position/pose (ENU)
and /mavros/global_position/global (lat/lon) share the same local ENU origin,
so instead of tracking a separate "home" fix, this node computes the ENU
delta between the ray/ground intersection point and the aircraft's *current*
ENU position, and applies that same delta (converted to a lat/lon delta) on
top of the aircraft's *current* global fix. This is mathematically equivalent
to referencing a fixed home point, but avoids relying on an origin that could
be stale, unavailable, or reset mid-flight.

Frame conventions assumed (all standard, not simulation-specific hacks):
  * Camera OPTICAL frame (as published by ros_gz_bridge / gz camera sensor):
        x = right, y = down, z = forward   (OpenCV / REP 104 style)
  * Camera LINK frame and aircraft BODY frame (REP 103):
        x = forward, y = left, z = up
  * MAVROS local_position/pose orientation rotates BODY (FLU) -> MAP (ENU).
  * World/"map" frame: ENU (x = East, y = North, z = Up).

No TF is used anywhere -- all fixed transforms are hardcoded from the SDF
values given by the user and exposed as ROS parameters for tunability.
"""

import math
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import CameraInfo, NavSatFix
from geometry_msgs.msg import PoseStamped

# Custom interfaces -- see the accompanying package.xml / CMakeLists / msg
# files for how to build these into your workspace.
from my_drone_controller.msg import TargetDetectionDrone, TargetGPSDrone


# ---------------------------------------------------------------------------
# Earth radius used for the flat-earth lat/lon <-> ENU conversion.
# ---------------------------------------------------------------------------
EARTH_RADIUS_M = 6378137.0  # WGS84 equatorial radius, good enough for flat-earth deltas


def quaternion_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Convert a (x, y, z, w) quaternion into a 3x3 rotation matrix.

    Implemented manually (no tf_transformations dependency) so this node
    only depends on numpy + rclpy + message packages.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    return np.array([
        [1 - 2 * (yy + zz),     2 * (xy - wz),     2 * (xz + wy)],
        [    2 * (xy + wz), 1 - 2 * (xx + zz),     2 * (yz - wx)],
        [    2 * (xz - wy),     2 * (yz + wx), 1 - 2 * (xx + yy)],
    ], dtype=np.float64)


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Standard extrinsic XYZ (roll, pitch, yaw) -> rotation matrix, i.e.
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll), matching SDF/URDF <pose> convention.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)

    return rz @ ry @ rx


def slerp_quaternion(q0: np.ndarray, q1: np.ndarray, frac: float) -> np.ndarray:
    """Spherical linear interpolation between two (x,y,z,w) quaternions.

    Falls back to normalized linear interpolation when the quaternions are
    nearly identical (avoids a division-by-zero in the slerp formula).
    """
    q0 = q0 / (np.linalg.norm(q0) + 1e-12)
    q1 = q1 / (np.linalg.norm(q1) + 1e-12)

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        # Take the shorter path on the hypersphere.
        q1 = -q1
        dot = -dot

    dot = max(-1.0, min(1.0, dot))

    if dot > 0.9995:
        result = q0 + frac * (q1 - q0)
        return result / (np.linalg.norm(result) + 1e-12)

    theta_0 = math.acos(dot)
    theta = theta_0 * frac
    sin_theta_0 = math.sin(theta_0)
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return s0 * q0 + s1 * q1


# Fixed rotation: camera OPTICAL frame (x-right, y-down, z-forward)
#              -> camera LINK frame    (x-forward, y-left, z-up)
# x_link = z_opt ; y_link = -x_opt ; z_link = -y_opt
R_LINK_FROM_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)


class GeolocatorNode(Node):
    def __init__(self) -> None:
        super().__init__('geolocator_node')

        # ---------------- Parameters (defaults from the user's SDF) --------
        self.declare_parameter('camera_offset_xyz', [0.35, 0.0, -0.10])
        self.declare_parameter('camera_offset_rpy', [0.0, 0.785, 0.0])
        self.declare_parameter('fallback_fx', 467.7427)
        self.declare_parameter('fallback_fy', 467.7427)
        self.declare_parameter('fallback_cx', 320.0)
        self.declare_parameter('fallback_cy', 240.0)
        self.declare_parameter('max_ray_ground_distance_m', 5000.0)
        self.declare_parameter('debug_verbose', False)
        self.debug_verbose = self.get_parameter('debug_verbose').value
        self.declare_parameter('debug', True)


        offset = self.get_parameter('camera_offset_xyz').get_parameter_value().double_array_value
        rpy = self.get_parameter('camera_offset_rpy').get_parameter_value().double_array_value
        self.cam_offset_body = np.array(offset if len(offset) == 3 else [0.35, 0.0, -0.10])
        self.R_body_from_link = rpy_to_rotation_matrix(*(rpy if len(rpy) == 3 else [0.0, 0.785, 0.0]))
        self.max_ray_ground_distance_m = self.get_parameter('max_ray_ground_distance_m').value
        self.debug = self.get_parameter('debug').value

        self.declare_parameter('history_duration_s', 2.0)
        self.declare_parameter('max_extrapolation_s', 0.25)
        self.history_duration_s = self.get_parameter('history_duration_s').value
        self.max_extrapolation_s = self.get_parameter('max_extrapolation_s').value

        # ---------------- Cached live state ---------------------------------
        self.fx = self.get_parameter('fallback_fx').value
        self.fy = self.get_parameter('fallback_fy').value
        self.cx = self.get_parameter('fallback_cx').value
        self.cy = self.get_parameter('fallback_cy').value
        self.camera_info_received = False

        # Time-tagged history buffers, each entry (t_sec: float, ...).
        # Used to interpolate pose/orientation/GPS to the exact detection
        # timestamp instead of just using "whatever's latest" -- see
        # detection_cb / _interpolate_pose / _interpolate_global.
        self.pose_history: deque = deque()    # (t_sec, pos[3], quat[4] xyzw)
        self.global_history: deque = deque()  # (t_sec, lat, lon)

        # Fallback "latest" values, kept for backward compatibility with a
        # TargetDetectionDrone.msg that has no (or zero) header.stamp.
        self.aircraft_pos_enu: Optional[np.ndarray] = None      # (3,) [E, N, U]
        self.aircraft_R_map_from_body: Optional[np.ndarray] = None  # (3,3)
        self.aircraft_lat: Optional[float] = None
        self.aircraft_lon: Optional[float] = None

        # ---------------- QoS --------------------------------------------
        # MAVROS topics are typically published with a "sensor data" style
        # QoS (best effort, volatile, small queue). Adjust if your MAVROS
        # config uses reliable QoS instead.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---------------- Subscribers --------------------------------------
        self.create_subscription(
            CameraInfo, '/copter/camera/camera_info', self.camera_info_cb, sensor_qos)

        self.create_subscription(
            PoseStamped, '/copter/mavros/local_position/pose', self.pose_cb, sensor_qos)

        self.create_subscription(
            NavSatFix, '/copter/mavros/global_position/global', self.global_cb, sensor_qos)

        self.create_subscription(
            TargetDetectionDrone, '/copter/detection/target', self.detection_cb, 10)

        # ---------------- Publisher -----------------------------------------
        self.pub_gps = self.create_publisher(TargetGPSDrone, '/detection/target_gps', 10)

        self.get_logger().info('geolocator_node started.')

    # -------------------------------------------------------------------
    # Subscriber callbacks
    # -------------------------------------------------------------------
    def camera_info_cb(self, msg: CameraInfo) -> None:
        # msg.k is the row-major 3x3 intrinsic matrix [fx 0 cx; 0 fy cy; 0 0 1]
        if len(msg.k) == 9 and msg.k[0] > 0.0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _prune_history(self, buf: deque, now_sec: float) -> None:
        cutoff = now_sec - self.history_duration_s
        while buf and buf[0][0] < cutoff:
            buf.popleft()

    def pose_cb(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        pos = np.array([p.x, p.y, p.z], dtype=np.float64)
        quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

        # Keep the old "latest" cache for backward compatibility (used when
        # a detection has no usable header.stamp).
        self.aircraft_pos_enu = pos
        self.aircraft_R_map_from_body = quaternion_to_rotation_matrix(*quat)

        t_sec = self._stamp_to_sec(msg.header.stamp)
        if t_sec <= 0.0:
            # Some setups don't stamp this topic -- fall back to node time.
            t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.pose_history.append((t_sec, pos, quat))
        self._prune_history(self.pose_history, t_sec)

    def global_cb(self, msg: NavSatFix) -> None:
        self.aircraft_lat = msg.latitude
        self.aircraft_lon = msg.longitude

        t_sec = self._stamp_to_sec(msg.header.stamp)
        if t_sec <= 0.0:
            t_sec = self.get_clock().now().nanoseconds * 1e-9
        self.global_history.append((t_sec, msg.latitude, msg.longitude))
        self._prune_history(self.global_history, t_sec)

    # -------------------------------------------------------------------
    # Time interpolation helpers
    # -------------------------------------------------------------------
    def _interpolate_pose(self, t_query: float) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
        """Return (position[3], rotation_matrix[3,3], sync_quality_s) at
        t_query by interpolating self.pose_history, or None if the buffer
        is empty. sync_quality_s is 0 for interpolation between two real
        samples, or the extrapolation distance in seconds if t_query fell
        outside the buffered range.
        """
        buf = self.pose_history
        if not buf:
            return None
        if len(buf) == 1 or t_query <= buf[0][0]:
            t0, pos0, quat0 = buf[0]
            return pos0, quaternion_to_rotation_matrix(*quat0), abs(t_query - t0)
        if t_query >= buf[-1][0]:
            t1, pos1, quat1 = buf[-1]
            return pos1, quaternion_to_rotation_matrix(*quat1), abs(t_query - t1)

        # Find the bracketing pair (linear scan -- buffers are small,
        # a few seconds of pose data at typical MAVROS rates).
        for i in range(len(buf) - 1):
            t0, pos0, quat0 = buf[i]
            t1, pos1, quat1 = buf[i + 1]
            if t0 <= t_query <= t1:
                frac = 0.0 if t1 == t0 else (t_query - t0) / (t1 - t0)
                pos = pos0 + frac * (pos1 - pos0)
                quat = slerp_quaternion(quat0, quat1, frac)
                return pos, quaternion_to_rotation_matrix(*quat), 0.0

        # Should not reach here given the bounds checks above.
        t1, pos1, quat1 = buf[-1]
        return pos1, quaternion_to_rotation_matrix(*quat1), abs(t_query - t1)

    def _interpolate_global(self, t_query: float) -> Optional[Tuple[float, float, float]]:
        """Return (lat, lon, sync_quality_s) at t_query, or None if the
        buffer is empty. Linear interpolation is adequate here since the
        time gaps and distances involved are small (flat-earth regime).
        """
        buf = self.global_history
        if not buf:
            return None
        if len(buf) == 1 or t_query <= buf[0][0]:
            t0, lat0, lon0 = buf[0]
            return lat0, lon0, abs(t_query - t0)
        if t_query >= buf[-1][0]:
            t1, lat1, lon1 = buf[-1]
            return lat1, lon1, abs(t_query - t1)

        for i in range(len(buf) - 1):
            t0, lat0, lon0 = buf[i]
            t1, lat1, lon1 = buf[i + 1]
            if t0 <= t_query <= t1:
                frac = 0.0 if t1 == t0 else (t_query - t0) / (t1 - t0)
                return lat0 + frac * (lat1 - lat0), lon0 + frac * (lon1 - lon0), 0.0

        t1, lat1, lon1 = buf[-1]
        return lat1, lon1, abs(t_query - t1)

    # -------------------------------------------------------------------
    # Main detection -> GPS callback
    # -------------------------------------------------------------------
    def detection_cb(self, msg: TargetDetectionDrone) -> None:
        if not msg.target_found:
            return

        if not self.camera_info_received:
            self.get_logger().warn(
                'No /camera/camera_info received yet - using fallback intrinsics '
                'from parameters.', throttle_duration_sec=5.0)

        # ---- Resolve the timestamp to synchronize against -----------------
        # If TargetDetectionDrone.msg has a populated header.stamp (recommended:
        # set it to the source image's header.stamp in your YOLO node), we
        # interpolate pose/GPS to that exact instant. Otherwise we fall back
        # to the old "use whatever's latest" behavior, which is less
        # accurate but keeps this node working with an un-updated message.
        t_det = None
        if hasattr(msg, 'header'):
            t_candidate = self._stamp_to_sec(msg.header.stamp)
            if t_candidate > 0.0:
                t_det = t_candidate

        pos = None
        R = None
        lat = None
        lon = None
        sync_note = ''

        if t_det is not None:
            pose_result = self._interpolate_pose(t_det)
            global_result = self._interpolate_global(t_det)
            if pose_result is not None and global_result is not None:
                pos, R, pose_dt = pose_result
                lat, lon, global_dt = global_result
                max_dt = max(pose_dt, global_dt)
                if max_dt > self.max_extrapolation_s:
                    self.get_logger().warn(
                        'Detection timestamp is %.3fs outside buffered pose/GPS '
                        'history (extrapolating) - accuracy may be degraded. '
                        'Consider increasing history_duration_s.' % max_dt,
                        throttle_duration_sec=2.0)
                sync_note = ' [t-synced, max_dt=%.3fs]' % max_dt

        if pos is None or R is None or lat is None or lon is None:
            # Fallback: old "latest cached value" behavior.
            if self.aircraft_pos_enu is None or self.aircraft_R_map_from_body is None:
                self.get_logger().warn(
                    'No aircraft pose yet (/mavros/local_position/pose) - '
                    'dropping detection.', throttle_duration_sec=2.0)
                return
            if self.aircraft_lat is None or self.aircraft_lon is None:
                self.get_logger().warn(
                    'No global fix yet (/mavros/global_position/global) - '
                    'dropping detection.', throttle_duration_sec=2.0)
                return
            pos = self.aircraft_pos_enu
            R = self.aircraft_R_map_from_body
            lat = self.aircraft_lat
            lon = self.aircraft_lon
            sync_note = ' [unsynced: no header.stamp on TargetDetectionDrone - see README]'

        # zoom_factor defaults to 1.0 for backward compatibility with older
        # bags/messages that predate this field.
        zoom_factor = float(getattr(msg, 'zoom_factor', 1.0)) or 1.0

        result = self.pixel_to_gps(msg.pixel_x, msg.pixel_y, pos, R, lat, lon, zoom_factor)
        if result is None:
            self.get_logger().warn(
                'Ray from pixel (%.1f, %.1f) does not intersect the ground plane '
                '(target above horizon or numerically invalid) - dropping detection.'
                % (msg.pixel_x, msg.pixel_y), throttle_duration_sec=2.0)
            return

        out_lat, out_lon = result

        out = TargetGPSDrone()
        out.class_name = msg.class_name
        out.confidence = msg.confidence
        out.latitude = out_lat
        out.longitude = out_lon
        self.pub_gps.publish(out)

        self.get_logger().info(
            'Target "%s" (conf=%.2f) -> lat=%.7f lon=%.7f%s'
            % (msg.class_name, msg.confidence, out_lat, out_lon, sync_note))

    # -------------------------------------------------------------------
    # Core geometry
    # -------------------------------------------------------------------
    def pixel_to_gps(self, u: float, v: float, aircraft_pos_enu: np.ndarray,
                      R_map_from_body: np.ndarray, aircraft_lat: float,
                      aircraft_lon: float, zoom_factor: float = 1.0) -> Optional[tuple]:
        """Project a pixel coordinate onto the ground plane and return
        (lat, lon), or None if the ray does not hit the ground in front
        of the aircraft.

        aircraft_pos_enu, R_map_from_body, aircraft_lat/lon are the
        (ideally timestamp-synchronized) aircraft state to use for this
        specific detection -- passed in explicitly rather than read from
        `self` so the caller controls exactly which synchronized sample
        is used (see detection_cb).

        zoom_factor accounts for any digital crop+resize ("zoom") applied
        to the frame upstream (in yolo_node.py) BEFORE running detection.
        Cropping to the center 1/zoom_factor of the frame and resizing
        back up increases the effective focal length by zoom_factor while
        leaving the principal point unchanged (crop is centered), so
        fx/fy from camera_info -- which describe the RAW, un-zoomed
        camera -- must be scaled up by zoom_factor to correctly interpret
        pixel coordinates measured on the zoomed frame. Skipping this
        silently under-corrects the ray angle for any off-center
        detection and is a systematic (not random) source of ground
        position error that grows with distance from the image center.
        """
        fx = self.fx * zoom_factor
        fy = self.fy * zoom_factor

        # 1) Pixel -> normalized ray in the camera OPTICAL frame.
        x_n = (u - self.cx) / fx
        y_n = (v - self.cy) / fy
        ray_optical = np.array([x_n, y_n, 1.0], dtype=np.float64)

        # 2) Optical frame -> camera LINK frame (fixed axis convention).
        ray_link = R_LINK_FROM_OPTICAL @ ray_optical

        # 3) Camera LINK -> aircraft BODY frame (fixed SDF extrinsic rotation).
        ray_body = self.R_body_from_link @ ray_link

        # 4) Aircraft BODY -> WORLD/ENU (map) frame.
        ray_world = R_map_from_body @ ray_body
        # Do not normalize -- only the direction matters for the intersection,
        # and normalizing does not change the result of the ratio below.

        # Camera origin in world/ENU frame:
        #   p_cam_world = p_body_world + R_map_from_body @ p_cam_body_offset
        cam_origin_world = aircraft_pos_enu + R_map_from_body @ self.cam_offset_body

        # 5) Intersect with ground plane z (Up) = 0:
        #    cam_origin_world.z + t * ray_world.z = 0  ->  t = -cam_origin_world.z / ray_world.z
        dz = ray_world[2]
        if dz >= -1e-6:
            # Ray is horizontal or pointing upward -> never reaches the ground.
            return None

        t = -cam_origin_world[2] / dz
        if t <= 0.0 or t > self.max_ray_ground_distance_m:
            # Intersection behind the camera, or implausibly far away.
            return None

        ground_point_world = cam_origin_world + t * ray_world  # [E, N, U] ; U == 0

        if self.debug_verbose:
            roll, pitch, yaw = self._rotation_matrix_to_rpy(R_map_from_body)
            self.get_logger().info(
                '\n--- geolocation debug ---\n'
                'pixel: u=%.2f v=%.2f\n'
                'intrinsics (raw): fx=%.3f fy=%.3f cx=%.2f cy=%.2f (camera_info_received=%s)\n'
                'zoom_factor=%.3f -> intrinsics (effective): fx=%.3f fy=%.3f\n'
                'ray_optical=%s\nray_link=%s\nray_body=%s\nray_world=%s\n'
                'aircraft_pos_enu(x=E,y=N,z=U)=%s\n'
                'aircraft roll/pitch/yaw (deg) = %.2f / %.2f / %.2f\n'
                'cam_origin_world=%s\n'
                't=%.3f\n'
                'ground_point_world(E,N,U)=%s\n'
                'aircraft (synced) lat/lon = %.7f, %.7f'
                % (u, v, self.fx, self.fy, self.cx, self.cy, self.camera_info_received,
                   zoom_factor, fx, fy,
                   ray_optical, ray_link, ray_body, ray_world,
                   aircraft_pos_enu,
                   math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                   cam_origin_world, t, ground_point_world,
                   aircraft_lat, aircraft_lon))

        # 6) ENU delta (target - synced aircraft position) -> lat/lon delta,
        #    applied on top of the aircraft's SYNCED global fix (see module
        #    docstring for why this is used instead of a separate home fix).
        delta_east = ground_point_world[0] - aircraft_pos_enu[0]
        delta_north = ground_point_world[1] - aircraft_pos_enu[1]

        lat_rad = math.radians(aircraft_lat)
        dlat_deg = math.degrees(delta_north / EARTH_RADIUS_M)
        dlon_deg = math.degrees(
            delta_east / (EARTH_RADIUS_M * max(math.cos(lat_rad), 1e-6)))

        target_lat = aircraft_lat + dlat_deg
        target_lon = aircraft_lon + dlon_deg

        if self.debug:
            ray_world_unit = ray_world / (np.linalg.norm(ray_world) + 1e-9)
            grazing_deg = math.degrees(math.asin(max(-1.0, min(1.0, -ray_world_unit[2]))))
            slant_range = math.sqrt(delta_east**2 + delta_north**2)
            self.get_logger().info(
                '--- geolocation debug ---\n'
                f'  pixel (u,v)            = ({u:.2f}, {v:.2f})\n'
                f'  intrinsics raw fx,fy,cx,cy = ({self.fx:.3f}, {self.fy:.3f}, {self.cx:.3f}, {self.cy:.3f})\n'
                f'  zoom_factor = {zoom_factor:.3f} -> effective fx,fy = ({fx:.3f}, {fy:.3f})\n'
                f'  ray_optical             = {ray_optical}\n'
                f'  ray_link                = {ray_link}\n'
                f'  ray_body                = {ray_body}\n'
                f'  ray_world (unnormalized)= {ray_world}\n'
                f'  ray downward angle deg  = {grazing_deg:.3f}  (90=straight down, 0=horizontal, <0=upward)\n'
                f'  aircraft_pos_enu (x,y,z)= {aircraft_pos_enu}\n'
                f'  aircraft altitude (Up)  = {aircraft_pos_enu[2]:.2f} m\n'
                f'  cam_origin_world        = {cam_origin_world}\n'
                f'  t (ray param, meters)   = {t:.3f}\n'
                f'  ground_point_world      = {ground_point_world}\n'
                f'  delta_east, delta_north = ({delta_east:.3f}, {delta_north:.3f})\n'
                f'  horizontal slant range  = {slant_range:.3f} m\n'
                f'  aircraft (synced) lat,lon = ({aircraft_lat:.8f}, {aircraft_lon:.8f})\n'
                f'  target lat,lon          = ({target_lat:.8f}, {target_lon:.8f})'
            )

        return target_lat, target_lon


def _rotation_matrix_to_rpy(R: np.ndarray) -> tuple:
    """Extract (roll, pitch, yaw) in radians from a rotation matrix built as
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Debug-only helper, not used in the
    core math path.
    """
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


GeolocatorNode._rotation_matrix_to_rpy = staticmethod(_rotation_matrix_to_rpy)


def main(args=None):
    rclpy.init(args=args)
    node = GeolocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()