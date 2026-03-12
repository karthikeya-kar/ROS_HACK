import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import time

class AutonomousDrone(Node):
    def __init__(self):
        super().__init__('autonomous_drone')
        
        # CV Bridge converts ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to drone camera
        self.camera_sub = self.create_subscription(
            Image,
            '/quadrotor/front_camera',
            self.camera_callback,
            10
        )
        
        # Image dimensions
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.CENTER_X = self.IMAGE_WIDTH // 2
        self.CENTER_Y = self.IMAGE_HEIGHT // 2
        self.DEADZONE = 50
        
        # Motor topic
        self.MOTOR_TOPIC = '/quadrotor/command/motor_speed'
        
        # State
        self.is_flying = False
        self.gate_passed_count = 0
        self.last_detection_time = time.time()
        
        self.get_logger().info('Autonomous Drone Started!')
        self.get_logger().info('Taking off in 3 seconds...')
        
        # Takeoff timer
        self.create_timer(3.0, self.takeoff)

    def set_motors(self, fl, fr, bl, br):
        """Send motor speed commands to Gazebo"""
        cmd = f"gz topic -t {self.MOTOR_TOPIC} --msgtype gz.msgs.Actuators -p 'velocity:[{fl},{fr},{bl},{br}]'"
        subprocess.Popen(cmd, shell=True)

    def takeoff(self):
        """Takeoff sequence"""
        if not self.is_flying:
            self.get_logger().info('Taking off!')
            end_time = time.time() + 3.0
            while time.time() < end_time:
                self.set_motors(1000, 1000, 1000, 1000)
                time.sleep(0.1)
            self.is_flying = True
            self.get_logger().info('Hovering — looking for gates!')

    def detect_gate(self, image):
        """
        Detect red gate in image
        Returns (center_x, center_y, area) or None
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Red color ranges in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Clean up noise
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
            
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        
        if area < 500:
            return None
            
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return None
            
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        
        return (cx, cy, area)

    def navigate(self, detection):
        """
        Navigate drone toward gate center
        """
        if detection is None:
            # No gate — search by rotating slowly
            self.get_logger().info('Searching for gate...')
            self.set_motors(700, 600, 700, 600)
            return

        cx, cy, area = detection
        error_x = cx - self.CENTER_X
        error_y = cy - self.CENTER_Y

        self.get_logger().info(f'Gate at ({cx},{cy}) | Area:{int(area)} | Error x:{error_x} y:{error_y}')

        # Gate is very close — fly through it!
        if area > 40000:
            self.get_logger().info('CLOSE TO GATE — flying through!')
            self.set_motors(800, 800, 600, 600)
            time.sleep(1.5)
            self.gate_passed_count += 1
            self.get_logger().info(f'Gates passed: {self.gate_passed_count}')
            # Stabilize after passing
            self.set_motors(650, 650, 650, 650)
            time.sleep(1.0)
            return

        # Gate centered — fly forward
        if abs(error_x) < self.DEADZONE and abs(error_y) < self.DEADZONE:
            self.get_logger().info('Gate CENTERED — moving forward!')
            self.set_motors(720, 720, 620, 620)
            return

        # Adjust based on gate position
        if error_x > self.DEADZONE:
            self.get_logger().info('Gate RIGHT — turning right')
            self.set_motors(720, 620, 720, 620)
        elif error_x < -self.DEADZONE:
            self.get_logger().info('Gate LEFT — turning left')
            self.set_motors(620, 720, 620, 720)
        elif error_y < -self.DEADZONE:
            self.get_logger().info('Gate HIGH — going up')
            self.set_motors(720, 720, 720, 720)
        elif error_y > self.DEADZONE:
            self.get_logger().info('Gate LOW — going down')
            self.set_motors(600, 600, 600, 600)

    def camera_callback(self, msg):
        """Called every time camera sends a new image"""
        if not self.is_flying:
            return
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Detect gate
            detection = self.detect_gate(cv_image)
            
            # Navigate toward gate
            self.navigate(detection)
            
            # Show camera feed with detection
            if detection:
                cx, cy, area = detection
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                cv2.putText(cv_image, f'Gate ({cx},{cy})', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(cv_image, f'Gates passed: {self.gate_passed_count}', (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            else:
                cv2.putText(cv_image, 'Searching...', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                           
            cv2.imshow('Drone Camera', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

def main():
    rclpy.init()
    drone = AutonomousDrone()
    
    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        pass
    finally:
        drone.set_motors(0, 0, 0, 0)
        cv2.destroyAllWindows()
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
