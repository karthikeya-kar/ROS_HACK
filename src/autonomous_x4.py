"""
Drone Wars X4 — World-frame goal-point navigation

KEY ARCHITECTURE CHANGE:
  Instead of timer-based PASS/COAST, we now use world-frame waypoints.

  When aligned with a gate:
    1. Record drone's current XYZ from /model/X4/pose  (world frame)
    2. Estimate gate distance from contour area
    3. Compute GOAL = drone_pos + forward_vector * (gate_dist + PASS_OVERSHOOT)
    4. In PASS state: fly toward GOAL using world-frame position error
    5. Gate is "cleared" when drone reaches GOAL — not by timer

  This means the drone KNOWS it has passed through because it physically
  reached a point 2m beyond where the gate was. No more guessing.

  SEARCH after clearing: rotate in place from GOAL position.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import math
import os
HAS_DISPLAY = os.environ.get('DISPLAY', '') != ''

# Check if display is available for visualization


class PID:
    def __init__(self, kp, ki, kd, max_out, name=''):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out = max_out
        self.integral = self.prev_err = 0.0
        self.t = time.time()
        self.name = name

    def update(self, err):
        now = time.time()
        dt  = max(now - self.t, 0.001)
        self.integral = np.clip(self.integral + err * dt, -2.0, 2.0)
        out = (self.kp * err
               + self.ki * self.integral
               + self.kd * (err - self.prev_err) / dt)
        self.prev_err = err
        self.t = now
        return float(np.clip(out, -self.max_out, self.max_out))

    def reset(self):
        self.integral = self.prev_err = 0.0
        self.t = time.time()


class GateDetector:
    RANGES = [
        ((0,   40, 40),  (18,  255, 255)),
        ((152, 40, 40),  (180, 255, 255)),
    ]
    MIN_AREA_S = 60

    def __init__(self, pw=320, ph=240):
        self.pw, self.ph = pw, ph
        self.k3 = np.ones((3, 3), np.uint8)
        self.k5 = np.ones((5, 5), np.uint8)

    def detect(self, bgr):
        """Returns (gates_list_largest_first, whole_frame_coverage)"""
        fh, fw = bgr.shape[:2]
        small  = cv2.resize(bgr, (self.pw, self.ph))
        hsv    = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)

        red = np.zeros((self.ph, self.pw), dtype=np.uint8)
        for lo, hi in self.RANGES:
            red = cv2.bitwise_or(red, cv2.inRange(hsv, lo, hi))
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, self.k5)
        red = cv2.dilate(red, self.k3, iterations=1)

        coverage = cv2.countNonZero(red) / (self.pw * self.ph)

        cnts, _ = cv2.findContours(red, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            return [], coverage

        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        gates = []

        for cnt in cnts:
            area_s = cv2.contourArea(cnt)
            if area_s < self.MIN_AREA_S:
                break

            rect     = cv2.minAreaRect(cnt)
            centre_s = rect[0]
            box_s    = np.int32(cv2.boxPoints(rect))

            # Interior mask
            interior = np.zeros((self.ph, self.pw), np.uint8)
            cv2.fillPoly(interior, [box_s], 255)
            shrink = max(2, int(min(rect[1]) * 0.06))
            mk = np.ones((shrink * 2 + 1, shrink * 2 + 1), np.uint8)
            interior = cv2.erode(interior, mk)

            void_roi = cv2.bitwise_and(cv2.bitwise_not(red), interior)
            void_roi = cv2.morphologyEx(void_roi, cv2.MORPH_OPEN, self.k3)

            vcx_s, vcy_s = int(centre_s[0]), int(centre_s[1])
            void_found = False

            vcnts, _ = cv2.findContours(void_roi, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)
            if vcnts:
                bv = max(vcnts, key=cv2.contourArea)
                if cv2.contourArea(bv) > 40:
                    M = cv2.moments(bv)
                    if M['m00'] > 0:
                        vcx_s = int(M['m10'] / M['m00'])
                        vcy_s = int(M['m01'] / M['m00'])
                        void_found = True

            vcx = int(vcx_s * fw / self.pw)
            vcy = int(vcy_s * fh / self.ph)

            red_full  = cv2.resize(red,      (fw, fh), interpolation=cv2.INTER_NEAREST)
            void_full = cv2.resize(void_roi, (fw, fh), interpolation=cv2.INTER_NEAREST)

            gates.append(dict(
                cx=vcx, cy=vcy,
                void_found=void_found,
                coverage=coverage,
                area_s=area_s,
                red_mask=red_full,
                void_mask=void_full,
                fw=fw, fh=fh,
            ))

        return gates, coverage


class DroneWarsCompetitor(Node):

    # ── TUNING ────────────────────────────────────────────────────────────
    TAKEOFF_CLIMB_TIME  = 6.0    # s
    TAKEOFF_VZ          = 0.90
    TAKEOFF_HOVER_TIME  = 1.5    # s stabilise after climb

    ALIGN_X_PX          = 35     # px dead zone
    ALIGN_Y_PX          = 35
    ALIGN_FRAMES_REQ    = 8

    GATE_OVERFILL       = 0.50
    GATE_MIN_AREA_PASS  = 120    # area_s to commit (gate close enough)
    GATE_ESCAPE_COV     = 0.85

    VX_APPROACH         = 0.65
    VX_PASS             = 1.6    # world-frame pass speed

    # World-frame goal: how far past the gate centre to place the waypoint
    PASS_OVERSHOOT      = 2.5    # m beyond gate

    # Arrival threshold — how close to GOAL counts as "done"
    GOAL_RADIUS         = 0.4    # m

    # Estimated gate distance from area_s (tuned experimentally)
    # dist_est = AREA_DIST_K / sqrt(area_s)
    # At area_s=120 (commit threshold) → ~3.2m
    # At area_s=300 → ~2.0m
    AREA_DIST_K         = 35.0

    VX_ESCAPE           = -0.8
    VZ_ESCAPE           = 0.5

    GATE_COOLDOWN       = 2.0
    SEARCH_YAW          = 0.30
    SEARCH_TIMEOUT      = 12.0

    GATE_AREA_DROP_FRAC = 0.50   # re-lock if area drops >50%
    RECT_CENTER_MAX_AREA = 150   # don't advance if void lost at close range
    # ──────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('competitor_x4')
        self.bridge      = CvBridge()
        self.latest_img  = None
        self.frame_lock  = threading.Lock()
        self.viz_lock    = threading.Lock()
        self.debug_frame = None
        self.detector    = GateDetector()

        self.pid_yaw = PID(kp=0.004, ki=0.0002, kd=0.002,  max_out=0.7)
        self.pid_vz  = PID(kp=0.003, ki=0.0002, kd=0.0015, max_out=0.45)

        # World-frame pose (from TF)
        self.pos        = np.array([0.0, 0.0, 0.0])   # x, y, z world
        self.yaw_world  = 0.0                           # drone heading in world
        self.pose_lock  = threading.Lock()

        self.state          = 'TAKEOFF'
        self._prev_state    = None
        self.enabled        = False
        self.height         = 0.0
        self.height_at_arm  = None
        self.takeoff_start  = None
        self.hover_start    = None
        self.gates_passed   = 0
        self.aligned_frames = 0
        self.search_start   = 0.0
        self.search_dir     = 1.0
        self.last_gate_time = 0.0

        # Gate lock
        self.locked_gate  = None
        self.locked_area  = 0.0

        # World-frame goal point (set when PASS begins)
        self.goal_world   = None   # np.array [x, y, z]

        self.cmd_pub    = self.create_publisher(Twist, '/model/X4/gazebo/command/twist', 1)
        self.enable_pub = self.create_publisher(Bool,  '/X4/enable', 1)
        self.create_subscription(
            Image,
            '/world/aerial_nav_world/model/X4/link/base_link/sensor/camera_front/image',
            self.img_cb, 1)
        self.create_subscription(TFMessage, '/model/X4/pose', self.pose_cb, 1)
        self.create_timer(0.033, self.control_loop)
        self.create_timer(0.5,   self.auto_enable)

        if HAS_DISPLAY:
            self.viz_thread = threading.Thread(target=self._viz_loop, daemon=True)
            self.viz_thread.start()
        else:
            self.get_logger().info('No DISPLAY — visualization disabled')
        self.get_logger().info('=== Drone Wars X4 — Goal-Point Edition ===')

    # ── helpers ───────────────────────────────────────────────────────────
    def _set_state(self, new, reason=''):
        if new != self._prev_state:
            self.get_logger().info(
                f'[STATE] {self._prev_state} -> {new}  ({reason})')
            self._prev_state = new
        self.state = new

    def auto_enable(self):
        if not self.enabled or (self.takeoff_start and
                                 time.time() - self.takeoff_start < 5.0):
            self.enable_pub.publish(Bool(data=True))
            if not self.enabled:
                self.enabled = True
                self.get_logger().info('Drone ENABLED')

    def pose_cb(self, msg):
        for tf in msg.transforms:
            name = tf.child_frame_id
            if 'X4' in name or 'x4' in name.lower() or 'base_link' in name:
                t = tf.transform.translation
                q = tf.transform.rotation
                with self.pose_lock:
                    self.pos = np.array([t.x, t.y, t.z])
                    self.height = t.z
                    # Extract yaw from quaternion
                    siny = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    self.yaw_world = math.atan2(siny, cosy)
                return
        # Fallback
        for tf in msg.transforms:
            if tf.child_frame_id.endswith('base_link'):
                self.height = tf.transform.translation.z
                with self.pose_lock:
                    self.pos[2] = self.height

    def img_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.latest_img = img
        except Exception:
            pass

    def send_vel(self, vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        t = Twist()
        t.linear.x  = float(vx);  t.linear.y  = float(vy)
        t.linear.z  = float(vz);  t.angular.z = float(yaw)
        self.cmd_pub.publish(t)

    def _est_gate_dist(self, area_s):
        """Estimate gate distance in metres from contour area in small frame."""
        if area_s <= 0:
            return 5.0
        return float(np.clip(self.AREA_DIST_K / math.sqrt(area_s), 0.5, 10.0))

    def _compute_goal(self, area_s):
        """
        Compute world-frame goal point:
          goal = drone_pos + heading_vector * (gate_dist + PASS_OVERSHOOT)
        We project along the drone's current world yaw (forward direction).
        The goal Z = current height (pass level).
        """
        with self.pose_lock:
            pos   = self.pos.copy()
            yaw   = self.yaw_world

        gate_dist = self._est_gate_dist(area_s)
        total     = gate_dist + self.PASS_OVERSHOOT

        # Forward vector in world XY at current yaw
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        goal = pos + fwd * total
        goal[2] = pos[2]   # keep same altitude

        self.get_logger().info(
            f'GOAL set: drone=({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f}) '
            f'yaw={math.degrees(yaw):.1f}° '
            f'gate_dist≈{gate_dist:.1f}m  total={total:.1f}m  '
            f'goal=({goal[0]:.1f},{goal[1]:.1f},{goal[2]:.1f})')
        return goal

    def _dist_to_goal(self):
        """3D distance from current position to goal_world."""
        if self.goal_world is None:
            return 999.0
        with self.pose_lock:
            pos = self.pos.copy()
        return float(np.linalg.norm(pos - self.goal_world))

    def _select_gate(self, gates):
        """Always pick the largest (closest) gate."""
        if not gates:
            return None
        best = gates[0]
        if (self.locked_gate is not None and self.locked_area > 0):
            ratio = best['area_s'] / self.locked_area
            if ratio < self.GATE_AREA_DROP_FRAC:
                self.get_logger().warn(
                    f'Gate area jump: {self.locked_area:.0f}->{best["area_s"]:.0f} '
                    f'ratio={ratio:.2f} — re-locking largest')
        return best

    # ── visualisation ─────────────────────────────────────────────────────
    def _viz_loop(self):
        cv2.namedWindow('Drone POV', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Drone POV', 960, 540)
        while True:
            with self.viz_lock:
                f = self.debug_frame.copy() if self.debug_frame is not None else None
            if f is not None:
                cv2.imshow('Drone POV', f)
            if cv2.waitKey(33) & 0xFF == ord('q'):
                break

    def _draw(self, frame, gate, err_x, err_y, n_gates, dist_to_goal):
        viz = frame.copy()
        fh, fw = viz.shape[:2]
        cx, cy = fw // 2, fh // 2

        if gate:
            ov = np.zeros_like(viz)
            ov[gate['red_mask'] > 0] = (180, 0, 200)
            cv2.addWeighted(ov, 0.25, viz, 1.0, 0, viz)
            ov2 = np.zeros_like(viz)
            ov2[gate['void_mask'] > 0] = (0, 230, 140)
            cv2.addWeighted(ov2, 0.5, viz, 1.0, 0, viz)

        cv2.line(viz, (cx-70, cy),   (cx+70, cy),   (255,255,255), 1, cv2.LINE_AA)
        cv2.line(viz, (cx,   cy-70), (cx,   cy+70), (255,255,255), 1, cv2.LINE_AA)

        ac = (0, 255, 0) if self.aligned_frames > 0 else (70, 70, 70)
        cv2.rectangle(viz,
                      (cx - self.ALIGN_X_PX, cy - self.ALIGN_Y_PX),
                      (cx + self.ALIGN_X_PX, cy + self.ALIGN_Y_PX), ac, 2)

        if gate:
            gx, gy = gate['cx'], gate['cy']
            col = (0, 255, 80) if gate['void_found'] else (0, 80, 255)
            cv2.line(viz, (cx, cy), (gx, gy), (0, 220, 255), 2, cv2.LINE_AA)
            cv2.circle(viz, (gx, gy), 14, col, -1)
            cv2.circle(viz, (gx, gy), 16, (255, 255, 255), 1)
            dist_e = self._est_gate_dist(gate['area_s'])
            cv2.putText(viz,
                        f'{"void" if gate["void_found"] else "RC"} '
                        f'a={gate["area_s"]:.0f} ~{dist_e:.1f}m',
                        (gx + 18, gy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)

        sc = {'TAKEOFF': (0,180,255), 'HOVER': (0,230,180),
              'SEARCH':  (255,120,0), 'NAVIGATE': (0,200,0),
              'PASS':    (20,20,255), 'WAIT_VOID': (0,200,200),
              'ESCAPE':  (0,50,255)}.get(self.state, (100,100,100))

        with self.pose_lock:
            px, py, pz = self.pos

        lines = [
            f'State   : {self.state}',
            f'Pos     : ({px:.1f},{py:.1f},{pz:.1f})',
            f'Yaw     : {math.degrees(self.yaw_world):.0f} deg',
            f'Gates   : {self.gates_passed}',
            f'err_x   : {err_x:+d} px' if err_x is not None else 'err_x   : --',
            f'err_y   : {err_y:+d} px' if err_y is not None else 'err_y   : --',
            f'aligned : {self.aligned_frames}/{self.ALIGN_FRAMES_REQ}',
            f'n_gates : {n_gates}',
            f'to_goal : {dist_to_goal:.2f}m' if self.goal_world is not None else 'to_goal : --',
        ]
        for i, ln in enumerate(lines):
            yp = 26 + i * 24
            cv2.putText(viz, ln, (10, yp), cv2.FONT_HERSHEY_SIMPLEX,
                        0.56, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(viz, ln, (10, yp), cv2.FONT_HERSHEY_SIMPLEX,
                        0.56, (255,255,255), 1, cv2.LINE_AA)

        cv2.rectangle(viz, (0, fh-30), (fw, fh), sc, -1)
        cv2.putText(viz, self.state, (fw//2 - 60, fh - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2, cv2.LINE_AA)
        return viz

    # ── control loop ──────────────────────────────────────────────────────
    def control_loop(self):
        if not self.enabled:
            return
        with self.frame_lock:
            if self.latest_img is None:
                return
            frame = self.latest_img.copy()

        gates, cov = self.detector.detect(frame)
        gate = self._select_gate(gates)

        fh, fw = frame.shape[:2]
        cx_f, cy_f = fw // 2, fh // 2

        err_x = err_y = None
        yaw_cmd = vz_cmd = 0.0

        if gate:
            self.locked_gate = gate
            self.locked_area = gate['area_s']
            err_x   = gate['cx'] - cx_f
            err_y   = cy_f - gate['cy']
            yaw_cmd = -self.pid_yaw.update(err_x)
            vz_cmd  =  self.pid_vz.update(err_y)
        elif self.locked_gate and self.state in ('NAVIGATE', 'PASS', 'WAIT_VOID'):
            err_x   = self.locked_gate['cx'] - cx_f
            err_y   = cy_f - self.locked_gate['cy']
            yaw_cmd = -self.pid_yaw.update(err_x)
            vz_cmd  =  self.pid_vz.update(err_y)

        dist_to_goal = self._dist_to_goal()
        if HAS_DISPLAY:
            dbg = self._draw(frame, gate, err_x, err_y, len(gates), dist_to_goal)
            with self.viz_lock:
                self.debug_frame = dbg

        # ── GLOBAL ESCAPE ──────────────────────────────────────────────────
        if self.state not in ('TAKEOFF', 'HOVER') and cov > self.GATE_ESCAPE_COV:
            if self.state != 'ESCAPE':
                self.get_logger().error(
                    f'WALL CRASH! cov={cov:.2f} — REVERSE+CLIMB')
                self._set_state('ESCAPE', f'cov={cov:.2f}')
                self.pid_yaw.reset(); self.pid_vz.reset()
                self.aligned_frames = 0
                self.locked_gate = None
                self.goal_world  = None
            self.send_vel(vx=self.VX_ESCAPE, vz=self.VZ_ESCAPE)
            return

        if self.state == 'ESCAPE' and cov < 0.40:
            self.get_logger().info('Escaped — back to SEARCH')
            self._set_state('SEARCH', 'escaped')
            self.search_start = time.time()
            self.locked_gate  = None

        # ── TAKEOFF ────────────────────────────────────────────────────────
        if self.state == 'TAKEOFF':
            if self.takeoff_start is None:
                self.takeoff_start = time.time()
                self.height_at_arm = self.height
                self.get_logger().info(
                    f'Takeoff: {self.TAKEOFF_CLIMB_TIME}s @ vz={self.TAKEOFF_VZ} '
                    f'h_start={self.height:.2f}')
            elapsed = time.time() - self.takeoff_start
            if elapsed < self.TAKEOFF_CLIMB_TIME:
                self.send_vel(vz=self.TAKEOFF_VZ)
                self.get_logger().info(
                    f'Climbing {elapsed:.1f}/{self.TAKEOFF_CLIMB_TIME}s '
                    f'h={self.height:.2f}m',
                    throttle_duration_sec=1.0)
            else:
                self.send_vel()
                self._set_state('HOVER', f'h={self.height:.2f}m')
                self.hover_start = time.time()

        # ── HOVER ──────────────────────────────────────────────────────────
        elif self.state == 'HOVER':
            self.send_vel()
            if time.time() - self.hover_start >= self.TAKEOFF_HOVER_TIME:
                delta = self.height - (self.height_at_arm or 0)
                self.get_logger().info(
                    f'Hover done h={self.height:.2f}m delta={delta:.2f}m')
                if delta < 0.3:
                    self.get_logger().error(
                        'WARNING: drone barely moved vertically — '
                        'check physics/enable!')
                self._set_state('SEARCH', 'hover done')
                self.search_start = time.time()

        # ── SEARCH ─────────────────────────────────────────────────────────
        elif self.state == 'SEARCH':
            elapsed = time.time() - self.search_start
            if elapsed > self.SEARCH_TIMEOUT:
                self.search_dir  *= -1
                self.search_start = time.time()
                self.get_logger().warn('Timeout — reversing yaw')

            if gate and cov < self.GATE_OVERFILL:
                self.get_logger().info(
                    f'SEARCH locked: area={gate["area_s"]:.0f} '
                    f'void={gate["void_found"]} '
                    f'n={len(gates)} dist≈{self._est_gate_dist(gate["area_s"]):.1f}m')
                self.pid_yaw.reset(); self.pid_vz.reset()
                self.aligned_frames = 0
                self.locked_gate = gate
                self.locked_area = gate['area_s']
                self._set_state('NAVIGATE', f'area={gate["area_s"]:.0f}')
            elif gate and cov >= self.GATE_OVERFILL:
                self.get_logger().warn(f'SEARCH overfill cov={cov:.2f} — back off')
                self.send_vel(vx=-0.4, vz=0.3)
            else:
                vz_h = 0.0
                if   self.height < 1.8: vz_h =  0.4
                elif self.height > 4.0: vz_h = -0.3
                self.send_vel(vz=vz_h, yaw=self.SEARCH_YAW * self.search_dir)

        # ── NAVIGATE ───────────────────────────────────────────────────────
        elif self.state == 'NAVIGATE':
            if not gate and self.locked_gate is None:
                self.get_logger().warn('NAVIGATE: gate lost — SEARCH')
                self._set_state('SEARCH', 'gate lost')
                self.search_start = time.time()
                return

            area_now = gate['area_s'] if gate else 0.0

            # Safety: void lost at close range → hold, wait for void
            if gate and not gate['void_found'] and area_now > self.RECT_CENTER_MAX_AREA:
                self.get_logger().warn(
                    f'Void lost close range area={area_now:.0f} — WAIT_VOID')
                self._set_state('WAIT_VOID', 'void lost close')
                self.send_vel(vz=vz_cmd, yaw=yaw_cmd)
                return

            x_ok = err_x is not None and abs(err_x) < self.ALIGN_X_PX
            y_ok = err_y is not None and abs(err_y) < self.ALIGN_Y_PX
            big_enough = area_now >= self.GATE_MIN_AREA_PASS

            if x_ok and y_ok:
                self.aligned_frames += 1
            else:
                self.aligned_frames = max(0, self.aligned_frames - 1)

            src = 'void' if gate and gate['void_found'] else ('RC' if gate else 'HELD')
            self.get_logger().info(
                f'NAV [{src}] err=({err_x:+d},{err_y:+d}) '
                f'area={area_now:.0f} big={big_enough} '
                f'x={x_ok} y={y_ok} '
                f'al={self.aligned_frames}/{self.ALIGN_FRAMES_REQ}')

            if self.aligned_frames >= self.ALIGN_FRAMES_REQ and big_enough:
                # ── COMMIT: set world-frame goal ──────────────────────────
                self.goal_world = self._compute_goal(area_now)
                self.get_logger().info(
                    f'*** COMMIT — goal set {self.PASS_OVERSHOOT}m past gate ***')
                self._set_state('PASS', 'aligned+close')
                return

            self.send_vel(vx=self.VX_APPROACH, vz=vz_cmd, yaw=yaw_cmd)

        # ── WAIT_VOID ──────────────────────────────────────────────────────
        elif self.state == 'WAIT_VOID':
            if not gate:
                self._set_state('NAVIGATE', 'no gate in wait')
                return
            if gate['void_found']:
                self.get_logger().info('WAIT_VOID: void back')
                self._set_state('NAVIGATE', 'void reacquired')
                return
            if gate['area_s'] <= self.RECT_CENTER_MAX_AREA:
                self._set_state('NAVIGATE', 'drifted back — small')
                return
            self.send_vel(vz=vz_cmd, yaw=yaw_cmd)

        # ── PASS — fly to world-frame goal point ──────────────────────────
        elif self.state == 'PASS':
            if self.goal_world is None:
                self.get_logger().error('PASS but no goal — back to SEARCH')
                self._set_state('SEARCH', 'no goal')
                self.search_start = time.time()
                return

            dist = self._dist_to_goal()
            self.get_logger().info(
                f'PASS → goal  dist={dist:.2f}m  '
                f'goal=({self.goal_world[0]:.1f},'
                f'{self.goal_world[1]:.1f},'
                f'{self.goal_world[2]:.1f})',
                throttle_duration_sec=0.3)

            if dist <= self.GOAL_RADIUS:
                # ── GOAL REACHED — gate definitely cleared ─────────────────
                self.get_logger().info(
                    f'*** GOAL REACHED  dist={dist:.2f}m — gate cleared! ***')
                now = time.time()
                if now - self.last_gate_time >= self.GATE_COOLDOWN:
                    self.gates_passed  += 1
                    self.last_gate_time = now
                    self.get_logger().info(
                        f'*** GATE {self.gates_passed} CLEARED ***')
                else:
                    self.get_logger().warn('Goal reached but cooldown — not counting')
                self.goal_world     = None
                self.locked_gate    = None
                self.locked_area    = 0.0
                self.aligned_frames = 0
                self.pid_yaw.reset(); self.pid_vz.reset()
                self._set_state('SEARCH', 'goal reached')
                self.search_start = time.time()
                self.send_vel()
                return

            # Still flying toward goal — maintain forward speed + altitude hold
            # Also keep yaw correction so we stay pointed at gate
            self.send_vel(vx=self.VX_PASS,
                          vz=float(np.clip(vz_cmd, -0.35, 0.35)),
                          yaw=yaw_cmd * 0.5)   # gentle yaw during sprint


def main():
    rclpy.init()
    node = DroneWarsCompetitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_vel()
        except Exception:
            pass
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
