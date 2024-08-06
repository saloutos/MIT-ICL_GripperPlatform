# imports
import numpy as np
import time
from controllers.base.baseState import *
from .fsm_config import *

# define state
class Waiting(BaseState):
    def __init__(self):
        self.name = "Waiting"
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

        # waiting, so fingers should not be applying any torque

        # set finger commands
        GP.q_des = fsm_params.q_des_waiting.copy()
        GP.qd_des = fsm_params.qd_des_waiting.copy()
        GP.tau_ff = fsm_params.tau_ff_waiting.copy()
        GP.kp = fsm_params.kp_waiting.copy()
        GP.kd = fsm_params.kd_waiting.copy()

        # check for manual state transition to squeezing
        if GP.char_in=='s':
            next_state = "Squeeze"

        return next_state