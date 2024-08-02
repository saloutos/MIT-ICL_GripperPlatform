import time
import numpy as np
import select
import sys

# define controller class
class ImpedanceControlDemo:

    def __init__(self):
        self.name = "Impedance Control Demo"
        self.started = False


    def begin(self, GP):
        self.started = True

        # set some defaults
        self.q_des = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.qd_des = np.zeros((5,))
        self.tau_ff = np.zeros((5,))
        self.kp = np.array([2.5, 2.5, 2.5, 2.5, 2.5])
        self.kd = np.array([0.05, 0.05, 0.05, 0.05, 0.05])


    def update(self, GP):
        # run controller

        # print joint data
        q_cur = GP.q
        qd_cur = GP.qd
        tau_cur = GP.tau
        print("\033c", end="")
        print("Joints:")
        print("q: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(q_cur[0], q_cur[1], q_cur[2], q_cur[3], q_cur[4]))
        print("qd: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(qd_cur[0], qd_cur[1], qd_cur[2], qd_cur[3], qd_cur[4]))
        print("tau: {:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(tau_cur[0], tau_cur[1], tau_cur[2], tau_cur[3], tau_cur[4]))

        # set finger commands
        GP.q_des = self.q_des.copy()
        GP.qd_des = self.qd_des.copy()
        GP.tau_ff = self.tau_ff.copy()
        GP.kp = self.kp.copy()
        GP.kd = self.kd.copy()