#!/usr/bin/env python3
"""
g1_wander.py — Intelligent autonomous wandering for Unitree G1.

State machine:
  CRUISE  — clear path ahead, walk forward with gentle exploration drift
  SLOW    — obstacle approaching, slow down and steer toward best gap
  SURVEY  — blocked: do a slow 360 degree rotation recording the range at
             each heading, then pick the widest open corridor
  GO_GAP  — turn to face the widest gap found during survey, then cruise
  BACKUP  — dangerously close: reverse for 2.5 s, then survey

The bridge's obstacle avoidance (g1_tf_bridge.py) remains as a physical
safety net underneath — this node provides high-level exploration logic.

Run on robot:  python3 /tmp/g1_wander.py
Stop:          Ctrl+C (robot stops via cmd_vel watchdog)
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Wanderer(Node):

    # ---- Speeds ----
    CRUISE_VX = 0.35        # m/s full forward
    SLOW_VX = 0.12          # m/s minimum forward
    MAX_STEER = 0.55        # rad/s max steering in cruise/slow
    SURVEY_VYAW = 0.45      # rad/s survey spin (~14 s for 360 deg)
    BACKUP_VX = -0.12       # m/s reverse

    # ---- Distance thresholds ----
    CLEAR_DIST = 1.8        # above this, full cruise
    SLOW_DIST = 1.0         # below this, slow + steer
    BLOCKED_DIST = 0.40     # below this, start survey
    BACKUP_DIST = 0.28      # below this, emergency reverse

    # ---- Gap analysis ----
    GAP_THRESH = 0.80       # min range to count as "open" (m)
    HEADING_TOL = 0.18      # rad (~10 deg) for go_gap alignment
    N_SURVEY_BINS = 72      # 5-degree bins for survey data

    # ---- Timing ----
    RATE_HZ = 10
    BACKUP_DUR = 2.5        # seconds to back up
    SURVEY_TIMEOUT = 20.0   # seconds max for survey
    RANDOM_INTERVAL = 5.0
    RANDOM_STRENGTH = 0.20

    def __init__(self):
        super().__init__('g1_wanderer')

        # Scan data
        self._scan = None
        self._scan_amin = 0.0
        self._scan_inc = 0.0
        self._scan_n = 0

        # IMU yaw
        self._yaw = 0.0
        self._has_data = False

        # Subscriptions
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        # Publisher
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / self.RATE_HZ, self._tick)

        # State
        self._state = 'init'
        self._state_t = 0.0

        # Survey state
        self._surv_start_yaw = 0.0
        self._surv_prev_yaw = 0.0
        self._surv_total = 0.0
        self._surv_data = []        # [(offset_rad, front_range)]

        # Go-gap state
        self._target_yaw = 0.0

        # Backup state
        self._backup_t = 0.0

        # Random exploration drift
        self._rand_yaw = 0.0
        self._rand_t = time.monotonic()

        self.get_logger().info('Wanderer ready, waiting for scan + IMU...')

    # ---- Callbacks ----

    def _scan_cb(self, msg):
        self._scan = msg.ranges
        self._scan_amin = msg.angle_min
        self._scan_inc = msg.angle_increment
        self._scan_n = len(msg.ranges)

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._has_data = True

    # ---- Helpers ----

    @staticmethod
    def _wrap(a):
        """Normalize angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    @staticmethod
    def _adiff(a, b):
        """Shortest signed angle from b to a."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def _sector_min(self, center, half):
        """Min range in an angular sector of the scan."""
        if self._scan is None:
            return float('inf')
        lo = max(0, int((center - half - self._scan_amin) / self._scan_inc))
        hi = min(self._scan_n - 1,
                 int((center + half - self._scan_amin) / self._scan_inc))
        mr = float('inf')
        for i in range(lo, hi + 1):
            r = self._scan[i]
            if 0.20 < r < mr:
                mr = r
        return mr

    def _front_min(self):
        """Min range in a +/-30 deg forward cone."""
        return self._sector_min(0.0, math.radians(30))

    def _best_sector(self):
        """Best forward-hemisphere direction from current scan.

        Checks 13 sectors (-90 to +90 in 15-deg steps) and weights by
        forward preference (forward sectors score higher).
        Returns (angle_rad, range).
        """
        if self._scan is None:
            return 0.0, float('inf')
        half = math.radians(7.5)
        best_a, best_score = 0.0, 0.0
        best_r = 0.0
        for deg in range(-90, 91, 15):
            a = math.radians(deg)
            r = self._sector_min(a, half)
            w = 0.4 + 0.6 * math.cos(a)   # 1.0 forward, 0.4 at +-90
            score = r * w
            if score > best_score:
                best_a = a
                best_score = score
                best_r = r
        return best_a, best_r

    def _publish(self, vx=0.0, vyaw=0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = vyaw
        self._pub.publish(t)

    # ---- Main loop ----

    def _tick(self):
        if self._scan is None or not self._has_data:
            return

        now = time.monotonic()

        if self._state == 'init':
            self.get_logger().info(
                'Data received — autonomous wandering active!')
            self._state = 'cruise'
            self._state_t = now

        # Random drift refresh
        if now - self._rand_t > self.RANDOM_INTERVAL:
            self._rand_yaw = random.uniform(
                -self.RANDOM_STRENGTH, self.RANDOM_STRENGTH)
            self._rand_t = now

        front = self._front_min()

        # Emergency backup — but NOT during survey (let it finish)
        if front < self.BACKUP_DIST and self._state not in ('backup', 'survey'):
            self._state = 'backup'
            self._state_t = now
            self._backup_t = now
            self.get_logger().info(
                'Too close (%.2fm)! Backing up...' % front)

        # Dispatch
        if self._state == 'backup':
            self._do_backup(now)
        elif self._state == 'survey':
            self._do_survey()
        elif self._state == 'go_gap':
            self._do_go_gap()
        else:
            self._do_cruise_slow(front)

    # ---- BACKUP ----

    def _do_backup(self, now):
        gap_a, _ = self._best_sector()
        self._publish(self.BACKUP_VX, math.copysign(0.25, gap_a))
        if now - self._backup_t >= self.BACKUP_DUR:
            self.get_logger().info('Backup done, starting survey...')
            self._begin_survey()

    # ---- SURVEY ----

    def _begin_survey(self):
        self._state = 'survey'
        self._state_t = time.monotonic()
        self._surv_start_yaw = self._yaw
        self._surv_prev_yaw = self._yaw
        self._surv_total = 0.0
        self._surv_data = []
        self.get_logger().info(
            'Survey: rotating 360 degrees to find the best path...')

    def _do_survey(self):
        # Rotate in place
        self._publish(0.0, self.SURVEY_VYAW)

        # Track cumulative rotation via IMU yaw
        delta = self._adiff(self._yaw, self._surv_prev_yaw)
        self._surv_total += abs(delta)
        self._surv_prev_yaw = self._yaw

        # Record narrow forward range at this heading
        r = self._sector_min(0.0, math.radians(10))
        self._surv_data.append((self._surv_total, r))

        # Debug: log progress every 2 seconds
        elapsed = time.monotonic() - self._state_t
        if int(elapsed * 5) % 10 == 0 and len(self._surv_data) % 20 == 0:
            self.get_logger().info(
                'Survey: %.0f deg / 360, %.1fs elapsed, yaw=%.1f' % (
                    math.degrees(self._surv_total), elapsed,
                    math.degrees(self._yaw)))
        if self._surv_total >= 2.0 * math.pi:
            self._finish_survey()
        elif elapsed > self.SURVEY_TIMEOUT:
            self.get_logger().warn(
                'Survey timeout (%.1fs, %.0f deg accumulated) — '
                'finishing with partial data' % (
                    elapsed, math.degrees(self._surv_total)))
            self._finish_survey()

    def _finish_survey(self):
        data = self._surv_data
        nb = self.N_SURVEY_BINS
        bs = 2.0 * math.pi / nb

        # Bin survey data — keep minimum range per bin
        bins = [0.0] * nb
        cnts = [0] * nb
        for off, r in data:
            i = min(nb - 1, int(off / bs))
            if cnts[i] == 0 or r < bins[i]:
                bins[i] = r
            cnts[i] += 1

        # Unfilled bins count as blocked
        for i in range(nb):
            if cnts[i] == 0:
                bins[i] = 0.0

        # Find widest consecutive run above gap threshold
        best_s, best_l = 0, 0
        cur_s, cur_l = -1, 0
        for i in range(nb):
            if bins[i] > self.GAP_THRESH:
                if cur_s < 0:
                    cur_s = i
                    cur_l = 0
                cur_l += 1
                if cur_l > best_l:
                    best_s = cur_s
                    best_l = cur_l
            else:
                cur_s = -1
                cur_l = 0

        if best_l == 0:
            # Nothing wide enough — pick the single deepest bin
            best_i = max(range(nb), key=lambda j: bins[j])
            gap_off = (best_i + 0.5) * bs
            gap_r = bins[best_i]
            gap_w = bs
        else:
            center_i = best_s + best_l // 2
            gap_off = (center_i + 0.5) * bs
            gap_r = min(bins[best_s + j] for j in range(best_l))
            gap_w = best_l * bs

        # Convert survey-frame offset to absolute heading
        self._target_yaw = self._wrap(self._surv_start_yaw + gap_off)
        err = self._adiff(self._target_yaw, self._yaw)

        self.get_logger().info(
            'Survey done! %d samples, best gap: offset=%.0f deg, '
            'width=%.0f deg, depth=%.1fm — turning %.0f deg' % (
                len(data), math.degrees(gap_off),
                math.degrees(gap_w), gap_r,
                math.degrees(err)))

        self._state = 'go_gap'
        self._state_t = time.monotonic()

    # ---- GO_GAP ----

    def _do_go_gap(self):
        err = self._adiff(self._target_yaw, self._yaw)
        if abs(err) < self.HEADING_TOL:
            self.get_logger().info(
                'Facing gap (err=%.1f deg) — cruising!' % math.degrees(err))
            self._state = 'cruise'
            self._state_t = time.monotonic()
            self._publish(self.CRUISE_VX * 0.5, 0.0)
        else:
            # P-control on heading error
            vyaw = max(-self.MAX_STEER, min(self.MAX_STEER, err * 1.5))
            self._publish(0.02, vyaw)

    # ---- CRUISE / SLOW ----

    def _do_cruise_slow(self, front):
        if front < self.BLOCKED_DIST:
            self.get_logger().info(
                'Path blocked at %.2fm — starting 360 deg survey' % front)
            self._begin_survey()
            self._publish(0.0, 0.0)
            return

        gap_a, gap_r = self._best_sector()

        if front < self.SLOW_DIST:
            # Slow + steer toward best gap
            self._state = 'slow'
            frac = (front - self.BLOCKED_DIST) / \
                   (self.SLOW_DIST - self.BLOCKED_DIST)
            frac = max(0.0, min(1.0, frac))
            vx = self.SLOW_VX + \
                 (self.CRUISE_VX - self.SLOW_VX) * frac * 0.5
            steer = (1.0 - frac) * self.MAX_STEER
            vyaw = math.copysign(steer, gap_a) + self._rand_yaw * 0.3
        else:
            # Full cruise with gentle exploration drift
            self._state = 'cruise'
            sfrac = min(1.0,
                        (front - self.SLOW_DIST) /
                        (self.CLEAR_DIST - self.SLOW_DIST))
            vx = self.SLOW_VX + \
                 (self.CRUISE_VX - self.SLOW_VX) * sfrac
            vyaw = self._rand_yaw
            if gap_r > self.CLEAR_DIST and abs(gap_a) > math.radians(20):
                vyaw += math.copysign(0.15, gap_a)

        self._publish(vx, vyaw)


def main():
    rclpy.init()
    node = Wanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        t = Twist()
        node._pub.publish(t)
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
