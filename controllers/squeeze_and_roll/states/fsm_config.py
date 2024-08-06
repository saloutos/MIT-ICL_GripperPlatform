# define config variables for all states here

# imports
import numpy as np

# general params class to add attributes to for each state
class FSMparams:
    pass

fsm_params = FSMparams()

# NOTE: (from gripper data)
# base is [0]
# left is [1]
# right is [2]
# [3,4] are finger twist DOFs, not used yet

# default gains
fsm_params.kp_default = 2.5
fsm_params.kd_default = 0.05

# waiting finger states
fsm_params.q_des_waiting = np.zeros((5,))
fsm_params.qd_des_waiting = np.zeros((5,))
fsm_params.tau_ff_waiting = np.zeros((5,))
fsm_params.kp_waiting = np.zeros((5,))
fsm_params.kd_waiting = np.zeros((5,))

# squeezing params
fsm_params.q_des_base_squeeze = 0.0
fsm_params.kp_base_squeeze = 2.5
fsm_params.kd_base_squeeze = 0.05

# rolling params
fsm_params.tau_ff_left_roll = 0.1
fsm_params.q_des_right_amp_roll = 0.5
fsm_params.q_des_right_freq_roll = 2.0
fsm_params.kp_right_roll = 2.5
fsm_params.kd_right_roll = 0.05

# release finger states
fsm_params.q_des_release = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
fsm_params.qd_des_release = np.zeros((5,))
fsm_params.tau_ff_release = np.zeros((5,))
fsm_params.kp_release = np.array([2.5, 2.5, 2.5, 2.5, 2.5])
fsm_params.kd_release = np.array([0.05, 0.05, 0.05, 0.05, 0.05])
