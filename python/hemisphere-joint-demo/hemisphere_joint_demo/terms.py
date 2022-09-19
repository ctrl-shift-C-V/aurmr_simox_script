import numpy as np

from numpy import pi, sin, cos, sqrt, arcsin, arctan2


class CommonTerms:

    def __init__(
            self,
            L,
            T_0,
            a1=None,
            a2=None,
            ex=None,
            ey=None,
            ez=None,
    ):
        t = self

        self.sin_t0 = sin(T_0)
        self.cos_t0 = cos(T_0)

        self.sin_t0_sq = self.sin_t0 ** 2
        self.cos_t0_sq = self.cos_t0 ** 2

        self.l_p2 = L ** 2
        self.l_p3 = L ** 3
        self.l_p4 = L ** 4
        self.l_p5 = L ** 5
        self.l_p6 = L ** 6
        self.l_p8 = L ** 8

        self.two_l_p2_mul_sin_t0_sq = (2 * self.l_p2 * self.sin_t0_sq)
        if None not in [ex, ey]:
            self.ex_p2 = ex ** 2
            self.ey_p2 = ey ** 2
            self.ex_mul_ey = ex * ey
            self.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq = sqrt(4 * self.l_p2 * self.sin_t0_sq - self.ex_p2 - self.ey_p2)
            self.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq_div_two_l_p2_mul_sin_t0_sq = (
                    self.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq / self.two_l_p2_mul_sin_t0_sq)

        if a1 is not None:
            self.a1_p2 = a1 ** 2
            self.a1_p3 = a1 ** 3
            self.a1_p4 = a1 ** 4

            self.asin_a1_div_l = arcsin(a1 / L)
            self.sin_t0_plus_asin_a1_div_l = sin(T_0 + arcsin(a1 / L))
            self.sin_t0_plus_asin_a1_div_l_p2 = self.sin_t0_plus_asin_a1_div_l ** 2
        if a2 is not None:
            self.a2_p2 = a2 ** 2
            self.a2_p3 = a2 ** 3
            self.a2_p4 = a2 ** 4

            self.asin_a2_div_l = arcsin(a2 / L)
            self.sin_t0_plus_asin_a2_div_l = sin(T_0 + arcsin(a2 / L))
            self.sin_t0_plus_asin_a2_div_l_p2 = self.sin_t0_plus_asin_a2_div_l ** 2

        if a1 is not None and a2 is not None:
            self.l_p4_minus_a1_sq_mul_a2_sq = (self.l_p4 - a1 ** 2 * a2 ** 2)

            self.sqrt_01 = sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2
                + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2
                - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq
                - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2
                + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4
            )
            self.sqrt_02 = sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2
                + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2
                - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq
                - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2
                + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4
            )
            self.sqrt_03 = sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2
                + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2
                - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq
                - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2
                + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4
            )
            self.sqrt_04 = sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2
                + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2
                - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq
                - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2
                + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4
            )
            self.sqrt_05 = sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2
                + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2
                - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq
                - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2
                + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4
            )
