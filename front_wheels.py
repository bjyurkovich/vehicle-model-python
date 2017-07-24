import numpy as np


class FrontWheels(object):
    def __init__(self, tire_radius):
        self.tire_radius = tire_radius
        self.force_at_wheel = 0
        self.wheel_w = 0
        self.wheel_torque = 0

    def compute_step(self, torque, brake_torque, vehicle_velocity):

        if torque > 0:
            self.force_at_wheel = torque / self.tire_radius
            self.wheel_torque = torque
        else:
            self.force_at_wheel = brake_torque / self.tire_radius
            self.wheel_torque = brake_torque

        self.wheel_w = vehicle_velocity / self.tire_radius

        return {
            "force_at_wheel": self.force_at_wheel,
            "wheel_torque": torque,
            "wheel_w": self.wheel_w
        }
