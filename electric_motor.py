import numpy as np
import scipy.io as spio


class ElectricMotor(object):
    def __init__(self):
        mat = spio.loadmat('data/em_data.mat', squeeze_me=True)
        self.max_torque_w_map = np.array(mat['maxT_w'])
        self.max_torque_t_map = np.array(mat['maxT_T'])
        self.em_torque = 0

    def compute_step(self, power_request, em_f_w):

        if power_request > 0:
            self.em_torque = np.interp(
                power_request * em_f_w, self.max_torque_w_map, self.max_torque_t_map) * 2
        else:
            self.em_torque = 0

        return {
            "torque": self.em_torque
        }
