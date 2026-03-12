import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class AutonomousX4(Node):
    def __init__(self):
        super().__init__('autonomous_x4')

        self.bridge = CvBridge()

        # ---- State ----
        self.enabled = False
        self.current_height = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.gates_passed = 0
        self.state = 'TAKEOFF'  # TAKEOFF, SEARCH, NAVIGATE, PASS, LAND

        # ---- Image ----
        self.IMAGE_W = 640
        self.IMAGE_H = 480
        self.CENTER_X = self.IMAGE_W // 2
        self.CENTER_Y = self.IMAGE_H // 2
        self.DEADZONE = 40

        # ---- Publishers ----
        self.twist_pub = self.create_publisher(
            Twist, '/X4/gazebo/command/twist', 10)

        self.enable_pub = self.create_publisher(
            Bool, '/X4/enable', 10)

        # ---- Subscribers ----
        self.camera_sub = self.create_subscription(
            Image,
            '/world/multicopter/model/X4/link/base_link/sensor/camera_front/image',
            self.camera_callback, 10)

        self.pose_sub = self.create_subscription(
            TFMessage,
            '/model/X4/pose',
            self.pose_callback, 10)

        # ---- Timers ----
        self.create_timer(0.1, self.control_loop)   # 10Hz control
        self.create_timer(2.0, self.enable_drone)   # enable after 2s

        self.get_logger().info('Autonomous X4 Started!')

    def enable_drone(self):
        if not self.enabled:
            msg = Bool()
            msg.data = True
            self.enable_pub.publish(msg)
            self.enabled = True
            self.get_logger().info('Drone ENABLED!')

    def send_velocity(self, vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw)
        self.twist_pub.publish(msg)

    def pose_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'X4/base_footprint':
                self.current_height = transform.transform.translation.z
                self.current_x = transform.transform.translation.x
                self.current_y = transform.transform.translation.y
                break

    def detect_gate(self, image):
        """Find gate hole center using inner contour"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Red mask
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]),
                                 np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 100, 100]),
                                 np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)

        # Clean noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # RETR_CCOMP gives outer + inner contours
        contours, hierarchy = cv2.findContours(
            mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        if contours is None or len(contours) == 0:
            return None

        # Find hole — inner contour (hierarchy[0][i][3] != -1 means it has a parent)
        hole_cx, hole_cy, hole_area = None, None, 0

        for i, cnt in enumerate(contours):
            # Check if this is an inner contour (hole)
            if hierarchy[0][i][3] != -1:
                area = cv2.contourArea(cnt)
                if area > hole_area:
                    hole_area = area
                    M = cv2.moments(cnt)
                    if M['m00'] != 0:
                        hole_cx = int(M['m10'] / M['m00'])
                        hole_cy = int(M['m01'] / M['m00'])

        # Fallback — use bounding box center if no inner contour found
        if hole_cx is None:
            all_points = np.concatenate(contours)
            x, y, w, h = cv2.boundingRect(all_points)
            gate_area = w * h
            if gate_area < 300:
                return None
            hole_cx = x + w // 2
            hole_cy = y + h // 2
            hole_area = gate_area

        return (hole_cx, hole_cy, hole_area)

    def camera_callback(self, msg):
        """Process camera image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.latest_image = cv_image

            detection = self.detect_gate(cv_image)

            # Draw detection on image
            display = cv_image.copy()
            if detection:
                cx, cy, area = detection
                cv2.circle(display, (cx, cy), 10, (0,255,0), -1)
                cv2.rectangle(display,
                    (cx-50, cy-50), (cx+50, cy+50),
                    (0,255,0), 2)
                cv2.putText(display, f'Gate area:{int(area)}',
                    (10,30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0,255,0), 2)

            cv2.putText(display, f'State: {self.state}',
                (10,60), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255,255,0), 2)
            cv2.putText(display, f'Height: {self.current_height:.2f}m',
                (10,90), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255,255,0), 2)
            cv2.putText(display, f'Gates: {self.gates_passed}',
                (10,120), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0,255,255), 2)

            cv2.imshow('X4 Camera', display)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')

    def control_loop(self):
        """Main control loop at 10Hz"""
        if not self.enabled:
            return

        if self.state == 'TAKEOFF':
            self.do_takeoff()
        elif self.state == 'SEARCH':
            self.do_search()
        elif self.state == 'NAVIGATE':
            self.do_navigate()
        elif self.state == 'PASS':
            self.do_pass()

    def do_takeoff(self):
        """Take off to 1.5m then start searching"""
        TARGET = 1.5
        if self.current_height < TARGET - 0.1:
            self.send_velocity(vz=1.0)
            self.get_logger().info(
                f'Taking off... height:{self.current_height:.2f}m')
        else:
            self.send_velocity(vz=0.0)
            self.state = 'SEARCH'
            self.get_logger().info('Reached height! Searching for gate...')

    def do_search(self):
        """Search for gate by rotating slowly"""
        if not hasattr(self, 'latest_image'):
            return

        detection = self.detect_gate(self.latest_image)
        if detection:
            self.state = 'NAVIGATE'
            self.get_logger().info('Gate FOUND! Navigating...')
        else:
            # Rotate slowly to search
            self.send_velocity(yaw=0.3)

    def do_navigate(self):
        """Visual servoing — follow straight line to gate center"""
        if not hasattr(self, 'latest_image'):
            return

        detection = self.detect_gate(self.latest_image)

        if detection is None:
            self.send_velocity()
            self.state = 'SEARCH'
            return

        cx, cy, area = detection
        error_x = cx - self.CENTER_X  # positive = gate is to the right
        error_y = cy - self.CENTER_Y  # positive = gate is below center

        # Draw straight line from image center to gate center
        display = self.latest_image.copy()
        cv2.line(display,
                (self.CENTER_X, self.CENTER_Y),
                (cx, cy),
                (0, 255, 255), 2)
        cv2.circle(display, (cx, cy), 8, (0, 255, 0), -1)
        cv2.circle(display, (self.CENTER_X, self.CENTER_Y), 5, (255, 0, 0), -1)
        cv2.putText(display, f'err_x:{error_x} err_y:{error_y}',
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.imshow('X4 Camera', display)
        cv2.waitKey(1)

        # Gate is very close — pass through!
        if area > 35000:
            self.state = 'PASS'
            return

        # Proportional gains
        YAW_GAIN   = 0.05  # horizontal correction
        ALT_GAIN   = 0.05  # vertical correction
        FWD_SPEED  = 0.2    # constant forward speed

        # Calculate corrections
        yaw = -YAW_GAIN * error_x   # negative = turn right when gate is right
        vz  = -ALT_GAIN * error_y   # negative = go up when gate is above

        # Clamp corrections
        yaw = max(-1.0, min(1.0, yaw))
        vz  = max(-1.0, min(1.0, vz))

        self.get_logger().info(
            f'Gate:({cx},{cy}) Area:{int(area)} '
            f'err_x:{error_x} err_y:{error_y} '
            f'yaw:{yaw:.2f} vz:{vz:.2f}')

        self.send_velocity(vx=FWD_SPEED, vz=vz, yaw=yaw)

    def do_pass(self):
        """Fly through the gate"""
        self.get_logger().info('PASSING THROUGH GATE!')
        self.send_velocity(vx=2.0)  # full speed forward

        # After 2 seconds switch back to search
        if not hasattr(self, 'pass_timer'):
            self.pass_timer = self.get_clock().now()

        elapsed = (self.get_clock().now() -
                   self.pass_timer).nanoseconds / 1e9

        if elapsed > 2.0:
            self.gates_passed += 1
            self.get_logger().info(
                f'Gate passed! Total: {self.gates_passed}')
            del self.pass_timer
            self.state = 'SEARCH'

def main():
    rclpy.init()
    node = AutonomousX4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send_velocity()
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
