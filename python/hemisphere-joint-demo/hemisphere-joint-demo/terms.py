import numpy as np

from numpy import pi, sin, cos, sqrt, arcsin, arctan2


class CommonTerms:
    def __init__(self, L, T_0):
        self.sin_t0 = sin(T_0)
        self.cos_t0 = cos(T_0)

        self.sin_t0_sq = self.sin_t0 ** 2
        self.cos_t0_sq = self.cos_t0 ** 2

