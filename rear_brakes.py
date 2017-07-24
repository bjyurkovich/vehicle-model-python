import numpy as np
import scipy.io as spio


class RearBrakes(object):
    def __init__(self, brake_rear_proporation, tire_radius):

        mat = spio.loadmat('data/brake_data.mat', squeeze_me=True)
        self.brake_rear_proporation = brake_rear_proporation
        self.brake_torque = 0
        self.brake_w = 0
        self.pedal_position_percentage_map = np.array(
            mat['pedal_position_percentage'])
        self.total_brake_force_map = np.array(mat['f_total_brake'])
        self.tire_radius = tire_radius

    def compute_step(self, front_wheel_w, beta):
        #brake_torque_tot, brake_em_max, brake_power_rear,

        # For regen...
        # if brake_em_max - brake_power_rear > 0:
        #     self.brake_torque = brake_torque_tot * self.brake_rear_proporation
        # else:
        # self.brake_torque = torque

        if beta < 0:
            self.brake_torque = self.tire_radius * - \
                np.interp(beta * 100, self.pedal_position_percentage_map,
                          self.total_brake_force_map) * self.brake_rear_proporation
        else:
            self.brake_torque = 0

        self.brake_w = front_wheel_w

        return {
            "brake_torque": self.brake_torque,
            "brake_w": self.brake_w
        }
