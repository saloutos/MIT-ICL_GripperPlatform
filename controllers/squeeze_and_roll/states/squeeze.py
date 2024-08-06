# imports
import numpy as np
import time
from controllers.base.baseState import *
from .fsm_config import *

# define state
class Squeeze(BaseState):
    def __init__(self):
        self.name = "Squeeze"
        self.enabled = 0
        self.start_time = 0

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

        # squeezing with impedance controller at base motor, trying to hold pose with finger joints

        # set finger commands
        GP.q_des = np.array([fsm_params.q_des_base_squeeze, 0.0, 0.0, 0.0, 0.0])
        GP.qd_des = np.zeros((5,))
        GP.tau_ff = np.zeros((5,))
        GP.kp = np.array([fsm_params.kp_base_squeeze, fsm_params.kp_default, fsm_params.kp_default, 0.0, 0.0])
        GP.kd = np.array([fsm_params.kd_base_squeeze, fsm_params.kd_default, fsm_params.kd_default, 0.0, 0.0])

        # check for manual state transition to rolling
        if GP.char_in=='r':
            next_state = "Roll"

        return next_state