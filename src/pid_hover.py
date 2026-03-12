import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage
import subprocess
import math

class StabilityController(Node):
    def __init__(self):
        super().__init__('stability_controller')
        
        

        # ---- ALTITUDE PID ----
        self.Kp_alt = 3.0
        self.Ki_alt = 0.1
        self.Kd_alt = 8.0
        self.TARGET_HEIGHT = 1.5
        self.BASE_SPEED = 656.0

        # ---- ATTITUDE PID ----
        self.Kp_roll  = 40.0
        self.Kd_roll  = 15.0
        self.Kp_pitch = 40.0
        self.Kd_pitch = 15.0

        # ---- State ----
        self.height       = 0.0
        self.roll         = 0.0
        self.pitch        = 0.0
        self.roll_rate    = 0.0
        self.pitch_rate   = 0.0

        self.prev_error_alt = 0.0
        self.integral_alt   = 0.0

        self.is_armed = False

        # ---- Subscribers ----
        # IMU at 100Hz — attitude data
        self.imu_sub = self.create_subscription(
            Imu,
            '/quadrotor/imu',
            self.imu_callback,
            10
        )

        # Pose at 50Hz — height data
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/our_world/dynamic_pose/info',
            self.pose_callback,
            10
        )

        # ---- Control loop at 50Hz ----
        self.create_timer(0.02, self.control_loop)

        # ---- Arm after 2 seconds ----
        self.create_timer(2.0, self.arm)

        self.get_logger().info('Stability Controller Ready!')

    def arm(self):
        if not self.is_armed:
            self.is_armed = True
            self.get_logger().info('ARMED — Taking off!')

    def set_motors(self, fl, fr, bl, br):
        fl = max(200, min(1000, int(fl)))
        fr = max(200, min(1000, int(fr)))
        bl = max(200, min(1000, int(bl)))
        br = max(200, min(1000, int(br)))
        cmd = (f"gz topic -t /quadrotor/command/motor_speed "
               f"--msgtype gz.msgs.Actuators "
               f"-p 'velocity:[{fl},{fr},{bl},{br}]'")
        subprocess.Popen(cmd, shell=True,
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL)

    def imu_callback(self, msg):
        """Called at 100Hz — get roll, pitch, rates"""
        # Extract quaternion
        ox = msg.orientation.x
        oy = msg.orientation.y
        oz = msg.orientation.z
        ow = msg.orientation.w

        # Convert to roll/pitch angles
        self.roll  = math.atan2(2*(ow*ox + oy*oz),
                                1 - 2*(ox*ox + oy*oy))
        self.pitch = math.asin(max(-1, min(1, 2*(ow*oy - oz*ox))))

        # Angular rates directly from IMU
        self.roll_rate  = msg.angular_velocity.x
        self.pitch_rate = msg.angular_velocity.y

    def pose_callback(self, msg):
        """Called at 50Hz — get height"""
        for transform in msg.transforms:
            if 'quadrotor' in transform.child_frame_id or transform.child_frame_id == '':
                z = transform.transform.translation.z
                if z > 0:
                    self.height = z
                    break

    def altitude_pid(self):
        """Altitude PID — returns base motor speed"""
        error = self.TARGET_HEIGHT - self.height

        self.integral_alt += error * 0.02
        self.integral_alt = max(-30, min(30, self.integral_alt))

        derivative = (error - self.prev_error_alt) / 0.02
        self.prev_error_alt = error

        correction = (self.Kp_alt * error +
                     self.Ki_alt * self.integral_alt +
                     self.Kd_alt * derivative)

        return self.BASE_SPEED + correction

    def attitude_pid(self):
        """
        Attitude PID — returns per motor corrections
        Uses angular rate as derivative (more accurate than calculating it)
        
        Motor layout:
        rotor_0 = front left  (index 0)
        rotor_1 = back left   (index 1)
        rotor_2 = back right  (index 2)
        rotor_3 = front right (index 3)
        """
        # Roll correction
        roll_corr  = (self.Kp_roll  * self.roll +
                     self.Kd_roll  * self.roll_rate)

        # Pitch correction
        pitch_corr = (self.Kp_pitch * self.pitch +
                     self.Kd_pitch * self.pitch_rate)

        # Apply to motors
        # Roll right → increase left motors, decrease right
        # Pitch forward → increase back motors, decrease front
        fl = -pitch_corr - roll_corr  # front left
        fr = -pitch_corr + roll_corr  # front right
        bl =  pitch_corr - roll_corr  # back left
        br =  pitch_corr + roll_corr  # back right

        return fl, fr, bl, br

    def control_loop(self):
        """Main control loop at 50Hz"""
        if not self.is_armed:
            return

        # Get altitude base speed
        base = self.altitude_pid()

        # Get attitude corrections
        fl_c, fr_c, bl_c, br_c = self.attitude_pid()

        # Combine
        fl = base + fl_c
        fr = base + fr_c
        bl = base + bl_c
        br = base + br_c

        self.set_motors(fl, fr, bl, br)

        self.get_logger().info(
            f'H:{self.height:.2f}m | '
            f'Roll:{math.degrees(self.roll):.1f}° | '
            f'Pitch:{math.degrees(self.pitch):.1f}° | '
            f'Base:{int(base)} | '
            f'FL:{int(fl)} FR:{int(fr)} BL:{int(bl)} BR:{int(br)}'
        )

def main():
    rclpy.init()
    node = StabilityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_motors(0, 0, 0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
