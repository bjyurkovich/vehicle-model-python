import numpy as np
import scipy.io as spio


class FrontBrakes(object):
    def __init__(self, brake_front_proporation, tire_radius):
        mat = spio.loadmat('data/brake_data.mat', squeeze_me=True)
        self.brake_front_proporation = brake_front_proporation
        self.brake_torque = 0
        self.front_brake_w = 0
        self.pedal_position_percentage_map = np.array(
            mat['pedal_position_percentage'])
        self.total_brake_force_map = np.array(mat['f_total_brake'])
        self.tire_radius = tire_radius

    def compute_step(self, front_wheel_w, beta):

        if beta < 0:
            self.brake_torque = self.tire_radius * - \
                np.interp(-beta * 100, self.pedal_position_percentage_map,
                          self.total_brake_force_map) * self.brake_front_proporation
        else:
            self.brake_torque = 0

        self.brake_w = front_wheel_w

        return {
            "brake_torque": self.brake_torque,
            "front_brake_w":  self.front_brake_w
        }
