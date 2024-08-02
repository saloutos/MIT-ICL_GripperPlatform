# imports
import time
import can
import sys
import yaml
import numpy as np
import csv
import math
from datetime import datetime as dt
import select
import os
from enum import Enum
from utils.can_utils import *

# define the Gripper Platform class
class GripperPlatform:
    def __init__(self, control_dt=0.01, hand_control_mode=HandControlMode.MOTOR_DEBUG_MODE, bus2_enable=Bus2Enable.DISABLE, log_path=None):

        # gripper data init
        self.q = np.zeros((5,))
        self.qd = np.zeros((5,))
        self.tau = np.zeros((5,))

        self.q_des = np.zeros((5,))
        self.qd_des = np.zeros((5,))
        self.tau_ff = np.zeros((5,))
        self.kp = np.zeros((5,))
        self.kd = np.zeros((5,))

        # general init for platform
        self.current_t = 0.0
        self.last_control_t = 0.0
        self.control_dt = control_dt # 100Hz default
        self.run_control = False
        self.char_in = None
        self.new_char = False

        # general init for logging
        self.log_enable = (log_path is not None)
        if self.log_enable:
            # TODO: better log name convention here?
            self.log_name = log_path+"log_"+str(dt.now()).replace(" ", "_")+".csv"
            self.log_header = ['t']+\
                            ['q1', 'q2', 'q3', 'q4', 'q5']+\
                            ['qd1', 'qd2', 'qd3', 'qd4', 'qd5']+\
                            ['tau1', 'tau2', 'tau3', 'tau4', 'tau5']+\
                            ['q1_des', 'q2_des', 'q3_des', 'q4_des', 'q5_des']+\
                            ['qd1_des', 'qd2_des', 'qd3_des', 'qd4_des', 'qd5_des']+\
                            ['tau_ff1', 'tau_ff2', 'tau_ff3', 'tau_ff4', 'tau_ff5']
            self.log = [ [0]+ 5*[0] + 5*[0] + 5*[0] + 5*[0] + 5*[0] + 5*[0] ]
            self.log_file = open(self.log_name, mode='w')
            self.log_writer = csv.writer(self.log_file, delimiter=',')
            self.log_writer.writerows([self.log_header])
            self.log_start = 0.0 # will udpate this in initialize later

        # specific init for hardware
        # default modes, but this could change before initialize()
        self.hand_control_mode = hand_control_mode
        self.bus2_enable = bus2_enable
        # start CAN bus
        self.CAN_bus_1 = None
        # start CAN busses
        can_config_filepath = 'utils/can_config.yaml'
        with open(can_config_filepath,'r') as file:
            can_config = yaml.safe_load(file)
        try:
            # TODO: remove dependence on specific device_id
            for PCAN_DEVICE in can.detect_available_configs('pcan'):
                # first CAN bus for finger board
                self.CAN_bus_1 = can.interface.Bus(bustype=can_config['Interface1']['bustype'], channel=can_config['Interface1']['channel'],
                            device_id=eval(can_config['Interface1']['device_id']), state=eval(can_config['Interface1']['state']),
                            fd=can_config['Interface1']['fd'], f_clock_mhz=can_config['Interface1']['f_clock_mhz'],
                            bitrate=can_config['Nominal']['bitrate'], nom_brp=can_config['Nominal']['brp'], data_brp=can_config['Data']['brp'],
                            nom_tseg1=can_config['Nominal']['tseg1'], nom_tseg2=can_config['Nominal']['tseg2'], nom_sjw=can_config['Nominal']['sjw'],
                            data_tseg1=can_config['Data']['tseg1'], data_tseg2=can_config['Data']['tseg2'], data_sjw=can_config['Data']['sjw'])
                print(f"CAN_BUS_1: {PCAN_DEVICE['device_id']}")
            time.sleep(1.0)
        except Exception as e:
            print(f"CAN init failed: {e}")

    def initialize(self):

        # send initialize message to hand
        print("Waiting for gripper startup.")
        # TODO: add bus2_enable to hand startup message?
        self.CAN_bus_1.send(can.Message(arbitration_id=GRIPPER_ENABLE_ID, dlc=48, data=HAND_MODE_MSGS[(self.hand_control_mode,self.bus2_enable)], is_fd=True, is_extended_id=False))
        enable_time = time.time()
        # TODO: re-time this since there are fewer motors
        # NOTE: timed finger init to take just under 8 seconds
        while (time.time() - enable_time < 8.0):
            time.sleep(0.0005)
            # TODO: necessary to update_data here? this was necessary for SPI, may not be for CAN
            self.update_data()
            time.sleep(0.0005)
        print("Gripper started.")
        # TODO: any sensor init here? offset forces?

        # save log start time
        if self.log_enable:
            self.log_start = time.time()

    def shutdown(self):
        # send disable messages to gripper
        self.CAN_bus_1.send(can.Message(arbitration_id=GRIPPER_ENABLE_ID, dlc=48, data=HAND_MODE_MSGS[HandControlMode.DISABLE_CONTROL], is_fd=True, is_extended_id=False))
        print("CAN bus disabled.")
        # log recorded data
        if self.log_enable:
            print("Logging data.")
            self.log_writer.writerows(self.log)
            self.log_file.close()
            print("Log saved.")
        print("Shutdown.")

    def log_data(self, extra_data=None):
        if self.log_enable:
            t_log = time.time() - self.log_start
            log_line = [t_log] + \
                    self.q.tolist() + self.qd.tolist() + self.tau.tolist() + \
                    self.q_des.tolist() + self.qd_des.tolist() + self.tau_ff.tolist()
            # TODO: option to log extra data
            if extra_data is not None:
                log_line = log_line + extra_data
            self.log.append(log_line)

    def check_user_input(self):
        # check terminal input
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            self.char_in = sys.stdin.read(1)
        else:
            self.char_in = None

    def update_data(self):
        # get current time
        self.previous_t = self.current_t
        self.current_t = time.time()
        # print(self.current_t-self.previous_t)

        # check if control flag needs to be set
        if self.current_t - self.last_control_t > self.control_dt:
            self.last_control_t = self.current_t
            self.run_control = True

        # poll for CAN messages
        # TODO: maybe remove while loop?
        num_tries = 0
        while (num_tries < CAN_MSG_NUM):
            try:
                # try to receive CAN message
                can_message_1 = self.CAN_bus_1.recv(0.0)
                if (can_message_1 is not None):
                    # unpack message
                    if (can_message_1.data[0] == 0 and can_message_1.data[1] == 0 and can_message_1.data[2] == 0 and can_message_1.data[3] == 0 ):
                        print("Error Cleared!")
                    else:
                        if (can_message_1.arbitration_id == MOTOR_DATA):
                            dxl_pos, dxl_vel, dxl_tau = self.unpack_joints(can_message_1.data)
                            # set values here
                            self.q = dxl_pos.copy()
                            self.qd = dxl_vel.copy()
                            self.tau = dxl_tau.copy()
            except Exception as e:
                # error = 1
                print(f"CAN is dead: {e}")
            num_tries += 1

    def apply_control(self):
        # TODO: could just pass gr_data and idxs to the pack_command function?
        msg1 = self.pack_joints(self.q_des[0:3], self.qd_des[0:3], self.kp[0:3], self.kd[0:3], self.tau_ff[0:3])
        self.CAN_bus_1.send(can.Message(arbitration_id=BUS1_COMMAND, dlc=48, data=msg1, is_fd=True, is_extended_id=False))
        msg2 = self.pack_joints(self.q_des[3:5], self.qd_des[3:5], self.kp[3:5], self.kd[3:5], self.tau_ff[3:5])
        self.CAN_bus_1.send(can.Message(arbitration_id=BUS2_COMMAND, dlc=48, data=msg2, is_fd=True, is_extended_id=False))

    # packing CAN commands for fingers
    def pack_joints(self, p_des, v_des, kp, kd, t_ff):
        # p_des, v_des, kp, kd, t_ff will be numpy arrays
        command_msg = [ int(0) ] * 48
        num_joints = len(p_des)
        for idx in range(num_joints):
            p_int = float_to_uint(p_des[idx], P_MIN, P_MAX, 16)
            v_int = float_to_uint(v_des[idx], V_MIN, V_MAX, 12)
            kp_int = float_to_uint(kp[idx]*SCALE, KP_MIN, KP_MAX, 12)
            kd_int = float_to_uint(kd[idx]*SCALE, KD_MIN, KD_MAX, 12)
            t_int = float_to_uint(t_ff[idx]*SCALE, T_MIN, T_MAX, 12)
            command_msg[idx*8 + 0] = p_int >> 8
            command_msg[idx*8 + 1] = p_int & 0xFF
            command_msg[idx*8 + 2] = v_int >> 4
            command_msg[idx*8 + 3] = ((v_int & 0xF) << 4) | (kp_int >> 8)
            command_msg[idx*8 + 4] = kp_int & 0xFF
            command_msg[idx*8 + 5] = kd_int >> 4
            command_msg[idx*8 + 6] = ((kd_int & 0xF) << 4) | (t_int >> 8)
            command_msg[idx*8 + 7] = t_int & 0xff
        return command_msg

    # unpacking received joint data message from gripper
    def unpack_joints(self, msg):
        pos=[]
        vel=[]
        tau=[]
        p_int=[]
        v_int=[]
        i_int=[]

        imsg = [msg[i:i + 5] for i in range(0, 25, 5)] # break to each dxls, range(0, 25, 5) = [0, 5, 10, 15, 20] (5 motors)

        for jmsg in imsg:
            p_int.append((jmsg[0] << 8) | jmsg[1])
            v_int.append((jmsg[2] << 4) | (jmsg[3] >> 4))
            i_int.append(((jmsg[3] & 0xF) << 8) | jmsg[4])

        for p, v, i in zip(p_int, v_int, i_int):
            pos.append(uint_to_float(p, P_MIN, P_MAX, 16))
            vel.append(uint_to_float(v, V_MIN, V_MAX, 12))
            tau.append(uint_to_float(i, T_MIN, T_MAX, 12)/SCALE)

        # return (5,) numpy arrays for pos, vel, and tau
        return np.array(pos), np.array(vel), np.array(tau)