import numpy as np


class Vehicle(object):
    def __init__(self, mass, density_air, c_drag, frontal_area, delta_t=0.1):
        self.mass = mass
        self.delta_t = delta_t
        self.density_air = 1.29
        self.c_drag = c_drag
        self.frontal_area = frontal_area
        self.gravity = 9.8
        self.aero_drag = 0

        self.velocity = 0
        self.acceleration = 0
        self.position = 0

    def compute_step(self, force_at_wheel, road_load):
        self.acceleration = (force_at_wheel - self.aero_drag) / self.mass
        self.velocity = self.velocity + self.acceleration * self.delta_t

        if self.velocity < 0:
            self.velocity = 0

        self.aero_drag = self.compute_areo_drag(self.velocity)
        self.position = self.position + self.velocity * self.delta_t

        return {
            "acceleration": self.acceleration,
            "velocity": self.velocity,
            "position": self.position
        }

    def compute_areo_drag(self, velocity):
        return np.power(velocity, 2) * 0.5 * self.density_air * self.c_drag * self.frontal_area
