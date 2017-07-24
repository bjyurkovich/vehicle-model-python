import numpy as np
import scipy.io as spio


class Battery:
    def __init__(self, soc0, num_p, num_s, delta_t):
        self.soc0 = soc0
        self.soc = self.soc0
        self.dt = delta_t
        self.vc1 = 0
        self.vc2 = 0

        self.num_p = num_p
        self.num_s = num_s
        self.p_pack = 0

        mat = spio.loadmat('data/battery_mappings.mat', squeeze_me=True)
        self.ocv_map = np.array(mat['OCV_curve'])
        self.soc_map = np.linspace(0, 1, len(self.ocv_map))
        self.capacity = mat['capacity']

        self.pack_capacity = self.capacity * num_p

        self.ocv = self.calculate_ocv(self.soc)
        self.v_batt = self.ocv
        self.v_pack = self.v_batt * num_s
        self.r0 = 0.001
        self.r1 = 0.0019
        self.r2 = 0.0014
        self.c1 = 10000
        self.c2 = 200000

    def calculate_soc(self, i):
        return self.soc + (-i / self.capacity / 3600) * self.dt

    def calculate_ocv(self, soc):
        return np.interp(soc, self.soc_map, self.ocv_map)

    def compute_vc1(self, i):
        beta = 1 / (self.r1 * self.c1)
        gamma = 1 / self.c1
        self.vc1 = np.exp(-beta * self.dt) * self.vc1 + gamma * - \
            i / beta * (1 - np.exp(-beta * self.dt))
        return self.vc1

    def compute_vc2(self, i):
        beta = 1 / (self.r2 * self.c2)
        gamma = 1 / self.c2
        self.vc2 = np.exp(-beta * self.dt) * self.vc2 + gamma * - \
            i / beta * (1 - np.exp(-beta * self.dt))
        return self.vc2

    def compute_max_power(self):
        # print(self.v_batt * self.num_p * 5 * self.capacity)
        return self.v_batt * self.num_p * 5 * self.capacity

    def compute_step(self, battery_power_requested):
        i = battery_power_requested / self.v_batt / self.num_s / self.num_p
        self.soc = self.calculate_soc(i)
        self.ocv = self.calculate_ocv(self.soc)

        self.v_batt = self.ocv - i * self.r0 + \
            self.compute_vc1(i) + self.compute_vc2(i)

        self.v_pack = self.v_batt * self.num_s
        self.p_pack = self.v_batt * i * self.num_s * self.num_p

        return {
            "cell_voltage": self.v_batt,
            "pack_voltage": self.v_pack,
            "pack_power": self.p_pack,
            "soc": self.soc
        }
