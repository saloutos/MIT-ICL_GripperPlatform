# imports
import numpy as np
import time
from controllers.base.baseState import *
from .fsm_config import *

# define state
class Roll(BaseState):
    def __init__(self):
        self.name = "Roll"
        self.enabled = 0

    def enter(self, GP):
        print(self.name)
        self.enabled = 1

        # get initial time
        self.start_time = time.time()

    def exit(self, GP):
        self.enabled = 0

    def execute(self, GP):

        # stay in this state
        next_state = self.name

        # print joint data
        q_cur = GP.q
        qd_cur = GP.qd
        tau_cur = GP.tau
        print("\033c", end="")
        print("State:", self.name)
        print("Joints:")
        print("q: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4]))
        print("qd: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(qd_cur[0], qd_cur[1], qd_cur[2], qd_cur[3], qd_cur[4]))
        print("tau: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(tau_cur[0], tau_cur[1], tau_cur[2], tau_cur[3], tau_cur[4]))

        # rolling object with fingers
        # base is holding pose, left finger is squeeze with feedforward torque
        # right finger is following trajectory with impedance control
        q_des_right = fsm_params.q_des_right_amp_roll*np.sin(2*np.pi*fsm_params.q_des_right_freq_roll*(time.time()-self.start_time))

        # set finger commands
        GP.q_des = np.array([fsm_params.q_des_base_squeeze, 0.0, q_des_right, 0.0, 0.0])
        GP.qd_des = np.zeros((5,))
        GP.tau_ff = np.array([0.0, fsm_params.tau_ff_left_roll, 0.0, 0.0, 0.0])
        GP.kp = np.array([fsm_params.kp_base_squeeze, 0.0, fsm_params.kp_right_roll, 0.0, 0.0])
        GP.kd = np.array([fsm_params.kd_base_squeeze, 0.0, fsm_params.kd_right_roll, 0.0, 0.0])

        # check for manual state transition to reset
        if GP.char_in=='x':
            next_state = "Release"

        return next_state