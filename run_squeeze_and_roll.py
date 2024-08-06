# imports
import atexit
import tty
import termios
import sys
import os

from GripperPlatform import GripperPlatform
from utils.can_utils import *

# initialization
print("Starting init.")
init_settings = termios.tcgetattr(sys.stdin)

# platform
GP = GripperPlatform(log_path=None)

GP.control_dt = 1.0/300.0
GP.hand_control_mode = HandControlMode.CURRENT_CONTROL
GP.bus2_enable = Bus2Enable.DISABLE

# controller
from controllers.squeeze_and_roll.SqueezeRollFSM import SqueezeRollFSM
controller = SqueezeRollFSM()

atexit.register(GP.shutdown)
print("Finished init.")

# start experiment
try:
    tty.setcbreak(sys.stdin.fileno())
    GP.initialize() # TODO: make sure that this waits for gripper to be initialized
    controller.begin(GP)
    GP.apply_control()
    print("Starting main loop.")
    while(1):
            # step in time to update data from hardware or sim
            GP.update_data()
            # run controller and update commands
            if GP.run_control:
                GP.run_control = False
                GP.check_user_input()
                controller.update(GP)
                GP.apply_control()
                GP.log_data()

# end experiment
finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, init_settings)