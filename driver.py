# import numpy


class Driver(object):
    """Driver Model that models a PID controller"""

    def __init__(self, k_p, k_i, k_d, delta_t):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.delta_t = delta_t
        self.v_i = 0
        self.v_d = 0

        self.alpha = 0
        self.beta = 0

    def compute_step(self, v_cyc, velocity):
        """Computes a single simulation step and return `alpha` (throttle) and `beta` (brake)"""
        v_err = v_cyc - velocity

        # Compute PID controller values
        v_p = self.k_p * v_err
        self.v_i = self.v_i + self.k_i * v_err * self.delta_t
        self.v_d = ((self.k_d * v_err) - self.v_d) / self.delta_t

        alpha = v_p + self.v_i + self.v_d
        beta = -(v_p + self.v_i + self.v_d)

        if alpha >= 1:
            self.alpha = 1
        else:
            self.alpha = alpha

        if beta <= -1:
            self.beta = -1
        else:
            self.beta = beta

        if v_err > 0:
            self.beta = 0
        else:
            self.alpha = 0

        return {
            "alpha": self.alpha,
            "beta": self.beta
        }
