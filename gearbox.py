import numpy as np


class Gearbox(object):
    def __init__(self, gearbox_efficiency, gearbox_ratio):
        self.gearbox_efficiency = gearbox_efficiency
        self.gearbox_ratio = gearbox_ratio
        self.em_f_w = 0
        self.torque_out = 0
        self.em_force_w = 0

    def compute_step(self, torque, front_brake_w):

        t_r = self.gearbox_ratio * torque

        if t_r >= 0:
            self.torque_out = self.gearbox_efficiency * t_r
        else:
            self.torque_out = 1 / self.gearbox_efficiency * t_r

        self.em_force_w = front_brake_w * self.gearbox_ratio

        return {
            "torque_out": self.torque_out,
            "em_force_w": self.em_force_w
        }
