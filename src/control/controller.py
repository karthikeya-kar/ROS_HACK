import subprocess

import time

def set_motors(fl, fr, bl, br):

    cmd = f"gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[{fl},{fr},{bl},{br}]'"

    subprocess.run(cmd, shell=True)

def run_motors_for(fl, fr, bl, br, duration):

    """Send motor commands continuously for a given duration"""

    end_time = time.time() + duration

    while time.time() < end_time:

        set_motors(fl, fr, bl, br)

def takeoff():

    print('Taking off...')

    run_motors_for(670, 670, 670, 670, 1.5)  # full thrust for 2 seconds

    print('Hovering...')

    run_motors_for(650, 650, 650, 650, 5.0)       # hover for 3 seconds

def move_forward():

    print('Moving forward...')

    run_motors_for(680, 680, 620, 620, 2.0)       # forward for 2 seconds

    print('Stabilizing...')

    run_motors_for(650, 650, 650, 650, 1.0)       # stabilize

def land():

    print('Landing...')

    run_motors_for(400, 400, 400, 400, 2.0)       # reduce thrust

    run_motors_for(0, 0, 0, 0, 1.0)              # cut motors

    print('Landed!')

if __name__ == '__main__':

    takeoff()

    move_forward()

    land()
