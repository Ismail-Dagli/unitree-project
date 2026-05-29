#!/usr/bin/env python3
"""
Autonomous wandering node for Unitree G1 — Survey-based.

State machine:
  CRUISE   — Walk forward, gently steer toward open space.
  SURVEY   — Front blocked: rotate 360° to record ranges in all directions.
             Builds a full panoramic clearance map.
  COMMIT   — Survey done: turn toward best gap, then walk into it.
  BACKUP   — Nose against wall: reverse for 2s, then enter SURVEY.

Much smarter than reactive: the robot looks around before deciding.
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class Wanderer(Node):

    # Speeds
    CRUISE_SPEED = 0.35
    SLOW_SPEED = 0.15
    MAX_YAW = 0.6
    SURVEY_YAW = 0.35         # slow rotation during 360° survey
    COMMIT_YAW = 0.50         # rotation speed when turning to chosen heading
    BACKUP_SPEED = -0.12      # reverse speed

    # Thresholds
    CLEAR_DIST = 2.0          # fully clear -> cruise
    APPROACH_DIST = 1.0       # start adjusting
    BLOCKED_DIST = 0.45       # trigger survey
    BACKUP_DIST = 0.30        # too close -> reverse first

    # Survey config
    SURVEY_BINS = 36          # 10° bins for 360° panoramic map
    SURVEY_BIN_WIDTH = math.radians(10)

    # Timing
    RATE_HZ = 10
    BACKUP_DURATION = 2.0     # seconds to reverse
    CRUISE_RANDOM_SEC = 5.0   # random yaw nudge interval
    COMMIT_MIN_WALK = 3.0     # walk at least this long after committing

    def __init__(self):
        super().__init__('g1_wanderer')

        # Scan data
        self._scan_ranges = None
        self._scan_amin = 0.0
        self._scan_inc = 0.0
        self._scan_n = 0
        self._range_min = 0.20

        # Yaw tracking from odometry
        self._yaw = 0.0
        self._yaw_valid = False

        # State machine
        self._state = 'INIT'
        self._state_start = time.monotonic()

        # Survey data: panoramic[i] = best range seen at heading bin i
        self._panoramic = [0.0] * self.SURVEY_BINS
        self._survey_start_yaw = 0.0
        self._survey_max_yaw_delta = 0.0

        # Commit data
        self._commit_heading = 0.0  # world yaw to walk toward
        self._commit_walk_start = 0.0

        # Cruise
        self._random_yaw = 0.0
        self._last_random_time = time.monotonic()

        # ROS interfaces
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / self.RATE_HZ, self._tick)

        self.get_logger().info('Wanderer v2 started — waiting for scan + odom...')

    # ---- Callbacks ----

    def _scan_cb(self, msg):
        self._scan_ranges = msg.ranges
        self._scan_amin = msg.angle_min
        self._scan_inc = msg.angle_increment
        self._scan_n = len(msg.ranges)
        self._range_min = msg.range_min

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        self._yaw_valid = True

    # ---- Helpers ----

    def _front_min_range(self):
        """Minimum range in front ±30° cone."""
        return self._cone_min_range(0.0, math.radians(30))

    def _cone_min_range(self, center_rad, half_angle):
        """Minimum range within a cone in the scan."""
        if self._scan_ranges is None:
            return float('inf')
        lo = center_rad - half_angle
        hi = center_rad + half_angle
        idx_lo = max(0, int((lo - self._scan_amin) / self._scan_inc))
        idx_hi = min(self._scan_n - 1, int((hi - self._scan_amin) / self._scan_inc))
        min_r = float('inf')
        for i in range(idx_lo, idx_hi + 1):
            r = self._scan_ranges[i]
            if r > self._range_min and r < min_r:
                min_r = r
        return min_r

    def _record_panoramic(self):
        """Record current scan into the panoramic map using current yaw."""
        if self._scan_ranges is None or not self._yaw_valid:
            return
        # For each scan bin that has a valid range, record it into the
        # panoramic bin corresponding to its world heading
        for i in range(self._scan_n):
            r = self._scan_ranges[i]
            if r <= self._range_min:
                continue
            # Scan angle in robot frame
            scan_angle = self._scan_amin + i * self._scan_inc
            # Only use front hemisphere of each scan (avoid body self-hits)
            if abs(scan_angle) > math.radians(70):
                continue
            # World heading
            world_heading = self._normalize(self._yaw + scan_angle)
            # Bin index (0..35)
            bin_idx = int((world_heading + math.pi) / self.SURVEY_BIN_WIDTH) % self.SURVEY_BINS
            # Keep the minimum range per bin (most conservative/accurate)
            if self._panoramic[bin_idx] == 0.0 or r < self._panoramic[bin_idx]:
                self._panoramic[bin_idx] = r

    def _best_panoramic_heading(self):
        """Find the best heading from the panoramic map.
        
        Scores contiguous groups of open bins, preferring wide corridors
        over narrow gaps.
        """
        bins = self._panoramic
        n = self.SURVEY_BINS
        
        # Score each bin: range value, 0 = not seen (treat as unknown/open)
        best_score = -1.0
        best_heading = 0.0

        # Sliding window: find the widest/deepest gap
        # Window of 5 bins (50°) — want a corridor at least this wide
        window = 5
        for start in range(n):
            min_range = float('inf')
            for j in range(window):
                idx = (start + j) % n
                v = bins[idx]
                if v > 0 and v < min_range:
                    min_range = v
            # Score: the minimum range in the window (deeper = better)
            score = min_range if min_range < float('inf') else 5.0
            if score > best_score:
                best_score = score
                # Center of the window
                center_idx = (start + window // 2) % n
                best_heading = -math.pi + (center_idx + 0.5) * self.SURVEY_BIN_WIDTH

        return best_heading, best_score

    @staticmethod
    def _normalize(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _angle_diff(self, target, current):
        """Signed shortest angular difference (target - current)."""
        return self._normalize(target - current)

    def _set_state(self, new_state):
        if new_state != self._state:
            self.get_logger().info('State: %s -> %s' % (self._state, new_state))
            self._state = new_state
            self._state_start = time.monotonic()

    # ---- Main tick ----

    def _tick(self):
        if self._scan_ranges is None or not self._yaw_valid:
            return

        if self._state == 'INIT':
            self.get_logger().info('Scan + odom ready — starting autonomous wander!')
            self._set_state('CRUISE')
            return

        twist = Twist()

        if self._state == 'CRUISE':
            twist = self._do_cruise()
        elif self._state == 'BACKUP':
            twist = self._do_backup()
        elif self._state == 'SURVEY':
            twist = self._do_survey()
        elif self._state == 'COMMIT':
            twist = self._do_commit()

        self._cmd_pub.publish(twist)

    def _do_cruise(self):
        """Cruise forward, gently steering toward open space."""
        twist = Twist()
        front = self._front_min_range()

        # Check if we need to stop
        if front < self.BACKUP_DIST:
            self._set_state('BACKUP')
            return twist
        if front < self.BLOCKED_DIST:
            self._start_survey()
            return twist

        # Random perturbation for exploration variety
        now = time.monotonic()
        if now - self._last_random_time > self.CRUISE_RANDOM_SEC:
            self._random_yaw = random.uniform(-0.2, 0.2)
            self._last_random_time = now

        # Speed based on clearance
        if front > self.CLEAR_DIST:
            twist.linear.x = self.CRUISE_SPEED
        elif front > self.APPROACH_DIST:
            frac = (front - self.APPROACH_DIST) / (self.CLEAR_DIST - self.APPROACH_DIST)
            twist.linear.x = self.SLOW_SPEED + (self.CRUISE_SPEED - self.SLOW_SPEED) * frac
        else:
            frac = (front - self.BLOCKED_DIST) / (self.APPROACH_DIST - self.BLOCKED_DIST)
            frac = max(0.0, min(1.0, frac))
            twist.linear.x = self.SLOW_SPEED * frac

            # Steer away from close obstacles — find more open side
            left = self._cone_min_range(math.radians(40), math.radians(20))
            right = self._cone_min_range(math.radians(-40), math.radians(20))
            if left > right:
                twist.angular.z = self.MAX_YAW * (1.0 - frac) * 0.6
            else:
                twist.angular.z = -self.MAX_YAW * (1.0 - frac) * 0.6

        twist.angular.z += self._random_yaw
        return twist

    def _do_backup(self):
        """Reverse for BACKUP_DURATION, then survey."""
        twist = Twist()
        elapsed = time.monotonic() - self._state_start

        if elapsed > self.BACKUP_DURATION:
            self._start_survey()
            return twist

        twist.linear.x = self.BACKUP_SPEED
        # Gentle turn while backing up so we don't reverse into a corner
        left = self._cone_min_range(math.radians(45), math.radians(25))
        right = self._cone_min_range(math.radians(-45), math.radians(25))
        if left > right:
            twist.angular.z = 0.15
        else:
            twist.angular.z = -0.15

        return twist

    def _start_survey(self):
        """Begin a 360° survey rotation."""
        self._panoramic = [0.0] * self.SURVEY_BINS
        self._survey_start_yaw = self._yaw
        self._survey_max_yaw_delta = 0.0
        self._set_state('SURVEY')
        self.get_logger().info(
            'Starting 360° survey at yaw=%.1f°' % math.degrees(self._yaw))

    def _do_survey(self):
        """Rotate in place, recording ranges into panoramic map."""
        twist = Twist()

        # Record current scan into panoramic
        self._record_panoramic()

        # Track total rotation
        delta = abs(self._angle_diff(self._yaw, self._survey_start_yaw))
        if delta > self._survey_max_yaw_delta:
            self._survey_max_yaw_delta = delta

        # Check if we've completed ~360° (or close enough after min time)
        elapsed = time.monotonic() - self._state_start
        completed_full = self._survey_max_yaw_delta > math.radians(150) and delta < math.radians(45)
        timed_out = elapsed > 25.0  # safety: don't spin forever

        if (completed_full and elapsed > 5.0) or timed_out:
            # Survey complete — pick best direction
            best_heading, best_score = self._best_panoramic_heading()
            filled = sum(1 for v in self._panoramic if v > 0)
            self.get_logger().info(
                'Survey done! %.0f° scanned, %d/%d bins filled. '
                'Best heading: %.0f° (range: %.2fm)' % (
                    math.degrees(self._survey_max_yaw_delta),
                    filled, self.SURVEY_BINS,
                    math.degrees(best_heading), best_score))

            # Log the panoramic map
            summary = []
            for i in range(self.SURVEY_BINS):
                angle = -180 + i * 10
                r = self._panoramic[i]
                if r > 0:
                    bar = '#' * min(10, int(r * 3))
                    summary.append('%+4d°: %4.1fm %s' % (angle, r, bar))
                else:
                    summary.append('%+4d°: ???' % angle)
            self.get_logger().info('Panoramic map:\n' + '\n'.join(summary))

            self._commit_heading = best_heading
            self._set_state('COMMIT')
            self._commit_walk_start = 0.0
            return twist

        # Keep rotating
        twist.angular.z = self.SURVEY_YAW
        return twist

    def _do_commit(self):
        """Turn toward chosen heading, then walk into it."""
        twist = Twist()

        heading_error = self._angle_diff(self._commit_heading, self._yaw)

        if self._commit_walk_start == 0.0:
            # Phase 1: turning toward target heading
            if abs(heading_error) > math.radians(15):
                twist.angular.z = math.copysign(
                    min(self.COMMIT_YAW, abs(heading_error) * 1.5),
                    heading_error)
                return twist
            else:
                # Close enough — start walking
                self._commit_walk_start = time.monotonic()
                self.get_logger().info(
                    'Heading aligned (err=%.0f°), walking!' % math.degrees(heading_error))

        # Phase 2: walk forward with gentle heading correction
        front = self._front_min_range()

        # Abort conditions
        if front < self.BACKUP_DIST:
            self._set_state('BACKUP')
            return twist
        if front < self.BLOCKED_DIST:
            walk_time = time.monotonic() - self._commit_walk_start
            if walk_time > self.COMMIT_MIN_WALK:
                # Walked enough, survey again
                self._start_survey()
                return twist
            # Haven't walked long enough yet — slow down and try
            twist.linear.x = self.SLOW_SPEED * 0.5
            twist.angular.z = math.copysign(self.MAX_YAW * 0.5, heading_error)
            return twist

        # Walk forward with heading correction
        if front > self.CLEAR_DIST:
            twist.linear.x = self.CRUISE_SPEED
        else:
            frac = (front - self.BLOCKED_DIST) / (self.CLEAR_DIST - self.BLOCKED_DIST)
            twist.linear.x = self.SLOW_SPEED + (self.CRUISE_SPEED - self.SLOW_SPEED) * frac

        # Gentle heading correction
        twist.angular.z = heading_error * 0.8  # proportional controller

        # After walking long enough, transition to open cruise
        walk_time = time.monotonic() - self._commit_walk_start
        if walk_time > self.COMMIT_MIN_WALK * 2 and front > self.APPROACH_DIST:
            self._set_state('CRUISE')

        return twist


def main():
    rclpy.init()
    node = Wanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        twist = Twist()
        node._cmd_pub.publish(twist)
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
