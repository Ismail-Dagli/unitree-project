"""
g1_tf_bridge.py — Unitree G1 Humanoid SDK -> ROS 2 Bridge.
ROS 2 Foxy + Python 3.8 compatible.

Architecture:
  - Uses unitree_sdk2py ChannelSubscriber to read from Unitree's native DDS
    (domain 0, topic: rt/lf/lowstate with unitree_hg LowState_)
  - Receives Livox Mid-360 point cloud via UDP multicast (224.1.1.5:56301)
  - Accumulates multiple Livox packets into one cloud before publishing
  - Converts cloud to LaserScan inline (no pointcloud_to_laserscan dependency)
  - Subscribes to /cmd_vel and forwards to G1 LocoClient
  - Publishes into our isolated ROS 2 domain (set via ROS_DOMAIN_ID env)

Data flow:
  SDK thread -> buffer -> ROS timer -> publish
  Livox UDP thread -> accumulator -> ROS timer -> publish
  /cmd_vel -> LocoClient.SetVelocity()
"""

from __future__ import annotations

import math
import socket
import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


class G1TfBridge(Node):

    # PointCloud2 field definitions (reused)
    _PC2_FIELDS = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    _POINT_STEP = 16

    # LaserScan parameters
    _SCAN_MIN_HEIGHT = -1.1  # catch obstacles near floor (lidar at 1.1m)
    _SCAN_MAX_HEIGHT = 0.5
    _SCAN_ANGLE_INC = math.radians(0.25)  # 0.00436 rad
    _SCAN_NUM_BINS = 1440  # 360 deg / 0.25 deg
    _SCAN_ANGLE_MIN = -math.pi
    _SCAN_ANGLE_MAX = -math.pi + (_SCAN_NUM_BINS - 1) * _SCAN_ANGLE_INC
    _SCAN_RANGE_MIN = 0.20  # filter self-hits from robot body/arms
    _SCAN_RANGE_MAX = 12.0

    # Obstacle avoidance parameters
    _OA_STOP_DIST = 0.40       # full stop if obstacle closer than this (m)
    _OA_SLOW_DIST = 0.90       # start slowing down at this distance (m)
    _OA_FRONT_HALF_ANGLE = math.radians(35)  # +/- 35 deg cone for forward
    _OA_SIDE_HALF_ANGLE = math.radians(25)   # +/- 25 deg cone for lateral
    _OA_SPEED_SCALE = 0.3      # speed multiplier in slow zone
    _OA_STEER_RATE = 0.6       # max injected yaw rate for auto-steer (rad/s)
    _OA_SCAN_HALF_ANGLE = math.radians(15)   # half-angle of each gap-finding sector

    def __init__(self):
        super().__init__('g1_tf_bridge')

        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lidar_frame', 'lidar_link')
        self.declare_parameter('lidar_x', 0.05)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 1.1)
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('livox_multicast_group', '224.1.1.5')
        self.declare_parameter('livox_multicast_port', 56301)
        self.declare_parameter('livox_enabled', True)
        self.declare_parameter('cloud_accumulate_ms', 100)
        self.declare_parameter('cmd_vel_enabled', True)
        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_vyaw', 0.8)

        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._lidar_frame = self.get_parameter('lidar_frame').value
        self._lidar_x = self.get_parameter('lidar_x').value
        self._lidar_y = self.get_parameter('lidar_y').value
        self._lidar_z = self.get_parameter('lidar_z').value
        self._net_iface = self.get_parameter('network_interface').value
        self._livox_group = self.get_parameter('livox_multicast_group').value
        self._livox_port = self.get_parameter('livox_multicast_port').value
        self._livox_enabled = self.get_parameter('livox_enabled').value
        self._accum_sec = self.get_parameter('cloud_accumulate_ms').value / 1000.0
        self._cmd_vel_enabled = self.get_parameter('cmd_vel_enabled').value
        self._max_vx = self.get_parameter('max_vx').value
        self._max_vy = self.get_parameter('max_vy').value
        self._max_vyaw = self.get_parameter('max_vyaw').value

        # TF
        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_lidar_tf()

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._cloud_pub = self.create_publisher(PointCloud2, '/utlidar/cloud', 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Shared buffers (SDK/UDP threads -> ROS timer)
        self._lock = threading.Lock()
        self._imu_data = None       # (qw, qx, qy, qz, gx, gy, gz)
        self._cloud_data = None     # (PointCloud2, LaserScan) pair
        self._got_imu = False
        self._latest_ranges = None  # latest scan ranges for obstacle avoidance

        # ROS timer: publish odom + TF at 20 Hz
        self._pub_timer = self.create_timer(0.05, self._publish_tick)

        # Initialize Unitree SDK
        self.get_logger().info(
            'Initializing Unitree SDK2 ChannelFactory on %s ...' % self._net_iface)
        ChannelFactoryInitialize(0, self._net_iface)

        self._lowstate_sub = ChannelSubscriber('rt/lf/lowstate', LowState_)
        self._lowstate_sub.Init(self._lowstate_cb, 10)
        self.get_logger().info('Subscribed to rt/lf/lowstate (unitree_hg LowState_)')

        # LocoClient for cmd_vel
        self._loco = None
        if self._cmd_vel_enabled:
            try:
                self._loco = LocoClient()
                self._loco.SetTimeout(10.0)
                self._loco.Init()
                self.get_logger().info('LocoClient initialized')

                # Put robot into walking mode (FSM 200) — required for
                # IMU streaming and velocity commands
                self._loco.Start()
                self.get_logger().info('LocoClient.Start() — robot in walking mode (FSM 200)')

                self._cmd_vel_sub = self.create_subscription(
                    Twist, '/cmd_vel', self._cmd_vel_cb, 10)
                self._last_cmd_time = time.monotonic()
                self._cmd_vel_timer = self.create_timer(0.5, self._cmd_vel_watchdog)
                self._cmd_active = False
                self.get_logger().info(
                    'cmd_vel bridge active (max vx=%.2f vy=%.2f vyaw=%.2f)' % (
                        self._max_vx, self._max_vy, self._max_vyaw))
            except Exception as e:
                self.get_logger().error('LocoClient init failed: %s' % str(e))
                self._loco = None

        # Livox receiver thread
        if self._livox_enabled:
            self._livox_thread = threading.Thread(
                target=self._livox_receiver_loop, daemon=True)
            self._livox_thread.start()
            self.get_logger().info(
                'Livox receiver thread started (%s:%d, accum=%dms)' % (
                    self._livox_group, self._livox_port,
                    int(self._accum_sec * 1000)))

        self.get_logger().info('G1 TF Bridge ready')

    # ---- Static TF ----
    def _publish_static_lidar_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._base_frame
        t.child_frame_id = self._lidar_frame
        t.transform.translation.x = self._lidar_x
        t.transform.translation.y = self._lidar_y
        t.transform.translation.z = self._lidar_z
        t.transform.rotation.w = 1.0
        self._static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            'Static TF: %s -> %s (%.2f, %.2f, %.2f)' % (
                self._base_frame, self._lidar_frame,
                self._lidar_x, self._lidar_y, self._lidar_z))

    # ---- SDK Callback (runs on CycloneDDS thread) ----
    def _lowstate_cb(self, msg):
        imu = msg.imu_state
        qw = float(imu.quaternion[0])
        qx = float(imu.quaternion[1])
        qy = float(imu.quaternion[2])
        qz = float(imu.quaternion[3])
        gx = float(imu.gyroscope[0])
        gy = float(imu.gyroscope[1])
        gz = float(imu.gyroscope[2])
        n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if n > 0.001:
            qw /= n; qx /= n; qy /= n; qz /= n
        with self._lock:
            self._imu_data = (qw, qx, qy, qz, gx, gy, gz)

    # ---- Obstacle avoidance helpers ----
    def _min_range_in_cone(self, center_angle, half_angle):
        """Return minimum range within an angular cone from latest scan."""
        ranges = self._latest_ranges
        if ranges is None:
            return float('inf')

        lo = center_angle - half_angle
        hi = center_angle + half_angle
        amin = self._SCAN_ANGLE_MIN
        inc = self._SCAN_ANGLE_INC
        n = len(ranges)

        idx_lo = max(0, int((lo - amin) / inc))
        idx_hi = min(n - 1, int((hi - amin) / inc))

        min_r = float('inf')
        for i in range(idx_lo, idx_hi + 1):
            r = ranges[i]
            if r > self._SCAN_RANGE_MIN and r < min_r:
                min_r = r
        return min_r

    def _oa_scale(self, min_range):
        """Return speed scaling factor [0..1] based on nearest obstacle."""
        if min_range <= self._OA_STOP_DIST:
            return 0.0
        if min_range >= self._OA_SLOW_DIST:
            return 1.0
        # Linear ramp between stop and slow distances
        return self._OA_SPEED_SCALE + (1.0 - self._OA_SPEED_SCALE) * (
            (min_range - self._OA_STOP_DIST) / (self._OA_SLOW_DIST - self._OA_STOP_DIST))

    def _best_steer_direction(self):
        """Find which side has more clearance for steering around obstacles.

        Samples sectors from -90 to +90 in 15-degree steps and returns a
        yaw rate: positive = turn left, negative = turn right.
        """
        ranges = self._latest_ranges
        if ranges is None:
            return 0.0

        half = self._OA_SCAN_HALF_ANGLE
        best_angle = 0.0
        best_range = 0.0

        # Check 13 sectors covering -90 to +90 degrees
        for deg in range(-90, 91, 15):
            rad = math.radians(deg)
            mr = self._min_range_in_cone(rad, half)
            if mr > best_range:
                best_range = mr
                best_angle = rad

        # Convert best gap angle to yaw direction: positive angle -> turn left
        if abs(best_angle) < math.radians(10):
            return 0.0  # best path is straight ahead, no steering needed
        return math.copysign(self._OA_STEER_RATE, best_angle)

    # ---- cmd_vel -> LocoClient ----
    def _cmd_vel_cb(self, msg):
        if self._loco is None:
            return
        vx = max(-self._max_vx, min(self._max_vx, msg.linear.x))
        vy = max(-self._max_vy, min(self._max_vy, msg.linear.y))
        vyaw = max(-self._max_vyaw, min(self._max_vyaw, msg.angular.z))

        # Obstacle avoidance: check scan in direction of travel
        if self._latest_ranges is not None:
            # Forward
            if vx > 0.01:
                front_dist = self._min_range_in_cone(0.0, self._OA_FRONT_HALF_ANGLE)
                s = self._oa_scale(front_dist)
                vx *= s
                # Active steering: when front is partially/fully blocked,
                # inject yaw toward the clearest gap direction
                if s < 0.95 and front_dist < self._OA_SLOW_DIST:
                    steer = self._best_steer_direction()
                    urgency = 1.0 - s  # 0 at full speed, 1 at full stop
                    vyaw += steer * urgency
            # Backward
            elif vx < -0.01:
                s = self._oa_scale(self._min_range_in_cone(math.pi, self._OA_FRONT_HALF_ANGLE))
                s2 = self._oa_scale(self._min_range_in_cone(-math.pi + 0.01, self._OA_FRONT_HALF_ANGLE))
                vx *= min(s, s2)

            # Lateral left (positive vy = left, angle = +pi/2)
            if vy > 0.01:
                s = self._oa_scale(self._min_range_in_cone(math.pi / 2, self._OA_SIDE_HALF_ANGLE))
                vy *= s
            elif vy < -0.01:
                s = self._oa_scale(self._min_range_in_cone(-math.pi / 2, self._OA_SIDE_HALF_ANGLE))
                vy *= s

            # Re-clamp yaw after steering injection
            vyaw = max(-self._max_vyaw, min(self._max_vyaw, vyaw))

            # If everything blocked, log once
            if abs(vx) < 0.001 and abs(vy) < 0.001 and (
                    abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01):
                self.get_logger().warn(
                    'Obstacle too close — velocity blocked, steering',
                    throttle_duration_sec=2.0)

        try:
            self._loco.SetVelocity(vx, vy, vyaw, duration=1.0)
            self._last_cmd_time = time.monotonic()
            if not self._cmd_active:
                self._cmd_active = True
                self.get_logger().info(
                    'cmd_vel active: vx=%.2f vy=%.2f vyaw=%.2f' % (vx, vy, vyaw))
        except Exception as e:
            self.get_logger().warn('SetVelocity error: %s' % str(e))

    def _cmd_vel_watchdog(self):
        """Stop robot if no cmd_vel received for 1 second."""
        if self._cmd_active and self._loco is not None:
            if time.monotonic() - self._last_cmd_time > 1.0:
                try:
                    self._loco.StopMove()
                except Exception:
                    pass
                self._cmd_active = False
                self.get_logger().info('cmd_vel timeout, robot stopped')

    # ---- ROS Timer: publish everything (runs on ROS executor thread) ----
    def _publish_tick(self):
        now = self.get_clock().now().to_msg()
        transforms = []

        with self._lock:
            imu = self._imu_data
            cloud_scan = self._cloud_data
            self._cloud_data = None

        if imu is not None:
            if not self._got_imu:
                self._got_imu = True
                self.get_logger().info('Receiving G1 IMU data!')

            qw, qx, qy, qz, gx, gy, gz = imu

            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.rotation.w = qw
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            transforms.append(t)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self._odom_frame
            odom.child_frame_id = self._base_frame
            odom.pose.pose.orientation.w = qw
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.twist.twist.angular.x = gx
            odom.twist.twist.angular.y = gy
            odom.twist.twist.angular.z = gz
            self._odom_pub.publish(odom)
        else:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.rotation.w = 1.0
            transforms.append(t)

        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'map'
        t2.child_frame_id = self._odom_frame
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        self._tf_broadcaster.sendTransform(transforms)

        if cloud_scan is not None:
            cloud, scan = cloud_scan
            cloud.header.stamp = now
            scan.header.stamp = now
            self._cloud_pub.publish(cloud)
            self._scan_pub.publish(scan)
            self._latest_ranges = scan.ranges

    # ---- Livox Mid-360 UDP multicast receiver (background thread) ----
    def _livox_receiver_loop(self):
        while rclpy.ok():
            try:
                self._run_livox_receiver()
            except Exception as e:
                self.get_logger().error('Livox error: %s' % str(e))
                time.sleep(2.0)

    def _run_livox_receiver(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # SO_REUSEPORT allows coexistence with the Livox SDK init process
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        sock.bind(('192.168.123.164', self._livox_port))

        # Also try multicast join (works if LiDAR is in multicast mode)
        try:
            group = socket.inet_aton(self._livox_group)
            mreq = struct.pack('4sL', group, socket.INADDR_ANY)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        except Exception:
            pass
        sock.settimeout(5.0)

        self.get_logger().info('Livox UDP socket bound, waiting for data...')
        logged_first = False
        logged_accum = False

        # Accumulator: collect points from multiple UDP packets
        accum_cloud_buf = bytearray()
        accum_points = []  # list of (x, y, z) tuples for scan conversion
        accum_count = 0
        accum_start = time.monotonic()

        while rclpy.ok():
            try:
                data, addr = sock.recvfrom(65536)
            except socket.timeout:
                if accum_count > 0:
                    cloud, scan = self._build_cloud_and_scan(
                        accum_cloud_buf, accum_points, accum_count)
                    with self._lock:
                        self._cloud_data = (cloud, scan)
                    accum_cloud_buf = bytearray()
                    accum_points = []
                    accum_count = 0
                    accum_start = time.monotonic()
                continue

            if not logged_first:
                logged_first = True
                self.get_logger().info(
                    'First Livox packet from %s, size=%d' % (str(addr), len(data)))

            # Parse points from this packet
            n_added = self._parse_into_buffer(data, accum_cloud_buf, accum_points)
            accum_count += n_added

            # Check if accumulation window elapsed
            elapsed = time.monotonic() - accum_start
            if elapsed >= self._accum_sec and accum_count > 0:
                if not logged_accum:
                    logged_accum = True
                    self.get_logger().info(
                        'First cloud: %d points accumulated in %.0fms' % (
                            accum_count, elapsed * 1000))
                cloud, scan = self._build_cloud_and_scan(
                    accum_cloud_buf, accum_points, accum_count)
                with self._lock:
                    self._cloud_data = (cloud, scan)
                accum_cloud_buf = bytearray()
                accum_points = []
                accum_count = 0
                accum_start = time.monotonic()

        sock.close()

    def _parse_into_buffer(self, raw_data, cloud_buf, points_list):
        # type: (bytes, bytearray, list) -> int
        """Parse a Livox UDP packet. Appends to cloud_buf and points_list."""
        if len(raw_data) < 36:
            return 0
        try:
            dot_num = struct.unpack_from('<H', raw_data, 5)[0]
            data_type = raw_data[10]
            if dot_num == 0:
                return 0

            # Livox SDK2 header: 24 bytes fields + 4 bytes CRC32 + 8 bytes timestamp = 36
            header_size = 36
            if data_type == 1:
                point_size = 14  # Cartesian: x(i32)+y(i32)+z(i32)+ref(u8)+tag(u8)
            elif data_type == 2:
                point_size = 10  # Spherical
            else:
                return 0

            added = 0
            off = header_size
            for _ in range(dot_num):
                if off + point_size > len(raw_data):
                    break
                if data_type == 1:
                    xm, ym, zm = struct.unpack_from('<iii', raw_data, off)
                    ref = raw_data[off + 12]
                    x = xm / 1000.0
                    y = ym / 1000.0
                    z = zm / 1000.0
                else:
                    depth, theta, phi = struct.unpack_from('<IHH', raw_data, off)
                    ref = raw_data[off + 8]
                    d = depth / 1000.0
                    tr = theta / 100.0 * math.pi / 180.0
                    pr = phi / 100.0 * math.pi / 180.0
                    x = d * math.sin(tr) * math.cos(pr)
                    y = d * math.sin(tr) * math.sin(pr)
                    z = d * math.cos(tr)

                # Skip zero/invalid points
                if x == 0.0 and y == 0.0 and z == 0.0:
                    off += point_size
                    continue

                cloud_buf += struct.pack('<ffff', x, y, z, float(ref))
                points_list.append((x, y, z))
                added += 1
                off += point_size

            return added
        except Exception:
            return 0

    def _build_cloud_and_scan(self, cloud_buf, points_list, count):
        # type: (bytearray, list, int) -> tuple
        """Build both PointCloud2 and LaserScan from accumulated data."""
        # Build PointCloud2
        cloud = PointCloud2()
        cloud.header.frame_id = self._lidar_frame
        cloud.height = 1
        cloud.width = count
        cloud.fields = self._PC2_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = self._POINT_STEP
        cloud.row_step = self._POINT_STEP * count
        cloud.is_dense = True
        cloud.data = bytes(cloud_buf[:self._POINT_STEP * count])

        # Build LaserScan from points
        scan = LaserScan()
        scan.header.frame_id = self._lidar_frame
        scan.angle_min = self._SCAN_ANGLE_MIN
        scan.angle_max = self._SCAN_ANGLE_MAX
        scan.angle_increment = self._SCAN_ANGLE_INC
        scan.range_min = self._SCAN_RANGE_MIN
        scan.range_max = self._SCAN_RANGE_MAX
        scan.time_increment = 0.0
        scan.scan_time = 0.1

        num_bins = self._SCAN_NUM_BINS
        ranges = [float('inf')] * num_bins

        for x, y, z in points_list:
            # Height filter
            if z < self._SCAN_MIN_HEIGHT or z > self._SCAN_MAX_HEIGHT:
                continue

            r = math.sqrt(x * x + y * y)
            if r < self._SCAN_RANGE_MIN or r > self._SCAN_RANGE_MAX:
                continue

            angle = math.atan2(y, x)
            if angle < self._SCAN_ANGLE_MIN or angle > self._SCAN_ANGLE_MAX:
                continue

            idx = int((angle - self._SCAN_ANGLE_MIN) / self._SCAN_ANGLE_INC)
            if 0 <= idx < num_bins and r < ranges[idx]:
                ranges[idx] = r

        scan.ranges = ranges
        return cloud, scan


def main(args=None):
    rclpy.init(args=args)
    node = G1TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
