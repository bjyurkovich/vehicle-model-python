import numpy as np
import scipy as sp


class Controller(object):
    def __init__(self, max_fc_power, vehicle):
        # self.max_battery_power = max_battery_power
        # self.min_battery_power = min_battery_power
        self.max_fc_power = max_fc_power

        self.em_max_torque_w_map = []
        self.em_max_torque_p_map = []
        self.vehicle = vehicle

    def compute_step(self, driver, battery, wheel):
        self.max_positive_power = driver.alpha * \
            self.max_fc_power + battery.compute_max_power()  # kW

        total_brake_force = -1 * np.interp(driver.beta * 100,
                                           self.vehicle.pedal_position_percentage, self.vehicle.total_brake)

        self.total_brake_power = total_brake_force * \
            self.vehicle.tire_radius * 0.001 * \
            self.vehicle.brake_radius_proportion * wheel.front_wheel_w

        if battery.soc > 0.2:
            self.fc_power_request = 10
        else:
            self.fc_power_request = 50

        self.battery_power_request = self.max_positive_power - \
            self.total_brake_power - self.fc_power_request

        self.battery_power_request = self.battery_power_request if self.battery_power_request > 0 else 0

        return {
            "total_brake_power": self.total_brake_power,
            "max_positive_power": self.max_positive_power,
            "battery_power_request": self.battery_power_request,
            "fc_power_request": self.fc_power_request
        }
