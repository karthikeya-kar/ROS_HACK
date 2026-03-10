import sys
sys.path.append('/home/kar/ros_hackathon/src')

from gate_detection.detect import detect_gate
import subprocess
import time

# Image center (assuming 640x480 camera)
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CENTER_X = IMAGE_WIDTH // 2
CENTER_Y = IMAGE_HEIGHT // 2

# How far from center before we correct (pixels)
DEADZONE = 50

# Motor speeds
HOVER = 650
HIGH = 700
LOW = 600

def set_motors(fl, fr, bl, br):
    """Send motor commands to Gazebo"""
    cmd = f"gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[{fl},{fr},{bl},{br}]'"
    subprocess.run(cmd, shell=True)

def navigate_to_gate(gate_x, gate_y):
    """
    Given gate center position in image,
    calculate which direction drone needs to move.
    
    Logic:
    - Image center = where drone is pointing
    - Gate center = where gate is
    - Difference = direction to move
    
    X axis: left/right
    Y axis: up/down
    """

    error_x = gate_x - CENTER_X  # positive = gate is right
    error_y = gate_y - CENTER_Y  # positive = gate is below center

    print(f'Gate at ({gate_x},{gate_y}) | Error: x={error_x}, y={error_y}')

    # Both centered — fly forward through gate!
    if abs(error_x) < DEADZONE and abs(error_y) < DEADZONE:
        print('Gate CENTERED — moving forward!')
        set_motors(680, 680, 620, 620)  # forward
        return 'forward'

    # Gate is too far right — rotate right
    if error_x > DEADZONE:
        print('Gate is RIGHT — rotating right')
        set_motors(HIGH, LOW, HIGH, LOW)
        return 'right'

    # Gate is too far left — rotate left
    if error_x < -DEADZONE:
        print('Gate is LEFT — rotating left')
        set_motors(LOW, HIGH, LOW, HIGH)
        return 'left'

    # Gate is too high — go up
    if error_y < -DEADZONE:
        print('Gate is HIGH — going up')
        set_motors(HIGH, HIGH, HIGH, HIGH)
        return 'up'

    # Gate is too low — go down
    if error_y > DEADZONE:
        print('Gate is LOW — going down')
        set_motors(LOW, LOW, LOW, LOW)
        return 'down'

def search_for_gate():
    """Slowly rotate to find gate if not visible"""
    print('No gate visible — searching...')
    set_motors(670, 630, 670, 630)  # slow rotation

def run_navigation(image):
    """
    Main navigation function.
    Takes image, detects gate, moves drone.
    """
    detection = detect_gate(image)

    if detection is None:
        search_for_gate()
        return 'searching'

    gate_x, gate_y, area = detection
    print(f'Gate detected! Area: {area}')

    # If gate is very large — we're close, go through!
    if area > 50000:
        print('VERY CLOSE TO GATE — full speed forward!')
        set_motors(750, 750, 600, 600)
        time.sleep(1.0)
        return 'passing'

    direction = navigate_to_gate(gate_x, gate_y)
    return direction

if __name__ == '__main__':
    import cv2
    print('Navigation test starting...')
    print('Show something RED to test navigation logic')
    print('Press Q to quit')

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        status = run_navigation(frame)
        cv2.putText(frame, f'Status: {status}', (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

        # Draw center crosshair
        cv2.line(frame, (CENTER_X, 0), (CENTER_X, IMAGE_HEIGHT), (255,0,0), 1)
        cv2.line(frame, (0, CENTER_Y), (IMAGE_WIDTH, CENTER_Y), (255,0,0), 1)

        cv2.imshow('Navigation', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
