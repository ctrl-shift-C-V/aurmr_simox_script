import numpy as np

import transforms3d as tf3d
import typing as ty

from numpy import pi, sin, cos, sqrt
from numpy import pi, sin, cos, sqrt, arcsin, arctan2

from .terms import CommonTerms


def f_zenith(a1, a2, L, T_0):
    t = CommonTerms(L=L, T_0=T_0)

    return -arcsin((2 * L ** 2 - 4 * L ** 2 * (L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
        T_0 + arcsin(a2 / L)) ** 3 - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
        T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 5 * t.sin_t0 - L * sqrt(2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
        T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
        T_0 + arcsin(a2 / L)) - 2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
        T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * t.sin_t0_sq * sin(
        T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) * sin(
        T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) * sin(
        T_0 + arcsin(a2 / L)) + 2 * L ** 8 * t.sin_t0_sq * sin(
        T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq - L ** 8 * sin(
        T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
        T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
        T_0 + arcsin(a1 / L)) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
        T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(
        T_0 + arcsin(a2 / L))) ** 2 / ((-L ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 2) * (
                -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
            T_0 + arcsin(a2 / L)) ** 2 + L ** 4) ** 2) - 4 * L ** 2 * (
                                   L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                               T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(
                               T_0 + arcsin(a1 / L)) ** 2 - L ** 5 * t.sin_t0 * sin(
                               T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + L ** 5 * sin(
                               T_0) - L * sqrt(
                               2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                                   T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                   T_0 + arcsin(a1 / L)) ** 3 * sin(
                                   T_0 + arcsin(a2 / L)) - 2 * L ** 8 * t.sin_t0_sq * sin(
                                   T_0 + arcsin(a1 / L)) ** 2 * sin(
                                   T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * t.sin_t0_sq * sin(
                                   T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                   T_0 + arcsin(a1 / L)) * sin(
                                   T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * t.sin_t0_sq * sin(
                                   T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + 2 * L ** 8 * sin(
                                   T_0) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * sin(
                                   T_0) ** 2 - L ** 8 * sin(T_0 + arcsin(a1 / L)) ** 4 * sin(
                                   T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
                                   T_0 + arcsin(a1 / L)) ** 4 * sin(
                                   T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
                                   T_0 + arcsin(a1 / L)) ** 2 * sin(
                                   T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
                                   T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(
                                   T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(T_0 + arcsin(a1 / L))) ** 2 / (
                                   (-L ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 + L ** 2) * (
                                       -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                   T_0 + arcsin(a2 / L)) ** 2 + L ** 4) ** 2)) / (2 * L ** 2)) + pi / 2


def f_azimuth(a1, a2, L, T_0):
    t = CommonTerms(L=L, T_0=T_0)

    return arctan2(sqrt(4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
            T_0 + arcsin(a2 / L)) ** 3 - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
            T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 5 * sin(
            T_0) - L * sqrt(2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
            T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) ** 3 * sin(T_0 + arcsin(a2 / L)) - 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * sin(
            T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq - L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(
            T_0 + arcsin(a2 / L))) ** 2 * t.sin_t0_sq / (
                                          (-L ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 2) * (
                                              -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                          T_0 + arcsin(a2 / L)) ** 2 + L ** 4) ** 2) - 4 * L ** 2 * (
                                          L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                                      T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(
                                      T_0 + arcsin(a1 / L)) ** 2 - L ** 5 * t.sin_t0 * sin(
                                      T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + L ** 5 * sin(
                                      T_0) - L * sqrt(
                                      2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                                          T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                          T_0 + arcsin(a1 / L)) ** 3 * sin(
                                          T_0 + arcsin(a2 / L)) - 2 * L ** 8 * t.sin_t0_sq * sin(
                                          T_0 + arcsin(a1 / L)) ** 2 * sin(
                                          T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * t.sin_t0_sq * sin(
                                          T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                          T_0 + arcsin(a1 / L)) * sin(
                                          T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * t.sin_t0_sq * sin(
                                          T_0 + arcsin(a1 / L)) * sin(
                                          T_0 + arcsin(a2 / L)) + 2 * L ** 8 * t.sin_t0_sq * sin(
                                          T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * sin(
                                          T_0) ** 2 - L ** 8 * sin(T_0 + arcsin(a1 / L)) ** 4 * sin(
                                          T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
                                          T_0 + arcsin(a1 / L)) ** 4 * sin(
                                          T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
                                          T_0 + arcsin(a1 / L)) ** 2 * sin(
                                          T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
                                          T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(
                                          T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(
                                      T_0 + arcsin(a1 / L))) ** 2 * t.sin_t0_sq / (
                                          (-L ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 + L ** 2) * (
                                              -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                          T_0 + arcsin(a2 / L)) ** 2 + L ** 4) ** 2)) * (
                                  L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
                              T_0 + arcsin(a2 / L)) ** 3 - L ** 5 * t.sin_t0 * sin(
                              T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) - L ** 5 * sin(
                              T_0) * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 5 * t.sin_t0 - L * sqrt(
                              2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                                  T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a1 / L)) ** 3 * sin(T_0 + arcsin(a2 / L)) - 2 * L ** 8 * sin(
                                  T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * sin(
                                  T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) * sin(
                                  T_0 + arcsin(a2 / L)) + 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq - L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 4 * sin(
                                  T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 4 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 2 * sin(
                                  T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(T_0 + arcsin(a2 / L))) / (
                                  L * sqrt(-L ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 2) * (
                                      -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + L ** 4) * t.sin_t0), sqrt(
        4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
            T_0 + arcsin(a2 / L)) ** 3 - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
            T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 5 * sin(
            T_0) - L * sqrt(2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
            T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) ** 3 * sin(T_0 + arcsin(a2 / L)) - 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * sin(
            T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + 2 * L ** 8 * t.sin_t0_sq * sin(
            T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq - L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
            T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(
            T_0 + arcsin(a2 / L))) ** 2 * t.sin_t0_sq / (
                    (-L ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 2) * (
                        -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                    T_0 + arcsin(a2 / L)) ** 2 + L ** 4) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(
                T_0 + arcsin(a1 / L)) ** 2 - L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) * sin(
                T_0 + arcsin(a2 / L)) + L ** 5 * t.sin_t0 - L * sqrt(
                2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                    T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
                    T_0 + arcsin(a1 / L)) ** 3 * sin(T_0 + arcsin(a2 / L)) - 2 * L ** 8 * sin(
                    T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                    T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * t.sin_t0_sq * sin(
                    T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
                    T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * sin(
                    T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + 2 * L ** 8 * sin(
                    T_0) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq - L ** 8 * sin(
                    T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
                    T_0 + arcsin(a1 / L)) ** 4 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
                    T_0 + arcsin(a1 / L)) ** 2 * sin(T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
                    T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(
                T_0 + arcsin(a1 / L))) ** 2 * t.sin_t0_sq / (
                    (-L ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 + L ** 2) * (
                        -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                    T_0 + arcsin(a2 / L)) ** 2 + L ** 4) ** 2)) * (
                                  L ** 5 * t.sin_t0 * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                              T_0 + arcsin(a2 / L)) - L ** 5 * t.sin_t0 * sin(
                              T_0 + arcsin(a1 / L)) ** 2 - L ** 5 * t.sin_t0 * sin(
                              T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) + L ** 5 * sin(
                              T_0) - L * sqrt(
                              2 * L ** 8 * t.sin_t0_sq * sin(T_0 + arcsin(a1 / L)) ** 3 * sin(
                                  T_0 + arcsin(a2 / L)) ** 3 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a1 / L)) ** 3 * sin(T_0 + arcsin(a2 / L)) - 2 * L ** 8 * sin(
                                  T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a1 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a1 / L)) * sin(T_0 + arcsin(a2 / L)) ** 3 + 2 * L ** 8 * sin(
                                  T_0) ** 2 * sin(T_0 + arcsin(a1 / L)) * sin(
                                  T_0 + arcsin(a2 / L)) + 2 * L ** 8 * t.sin_t0_sq * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 - 2 * L ** 8 * t.sin_t0_sq - L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 4 * sin(
                                  T_0 + arcsin(a2 / L)) ** 4 + L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 4 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 2 * sin(
                                  T_0 + arcsin(a2 / L)) ** 4 - L ** 8 * sin(
                                  T_0 + arcsin(a1 / L)) ** 2 - L ** 8 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + L ** 8) * sin(T_0 + arcsin(a1 / L))) / (
                                  L * sqrt(-L ** 2 * sin(T_0 + arcsin(a1 / L)) ** 2 + L ** 2) * (
                                      -L ** 4 * sin(T_0 + arcsin(a1 / L)) ** 2 * sin(
                                  T_0 + arcsin(a2 / L)) ** 2 + L ** 4) * t.sin_t0))


def fk(P1_z, P2_z, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=za3fZw9Rq5d9&line=1&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0)

    PE_x = 2 * L * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * sin(
            T_0) ** 2 - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * sin(
            T_0) ** 2 - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * sin(
            T_0) ** 2 - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * sin(
            T_0) ** 2 + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * sin(
            T_0) ** 2 + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                       sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2))
    PE_y = 2 * L * (L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * sin(
            T_0) ** 2 - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * sin(
            T_0) ** 2 - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * sin(
            T_0) ** 2 - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * sin(
            T_0) ** 2 + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * sin(
            T_0) ** 2 + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                       sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2))
    PE_z = 2 * L * (L * (L ** 4 - P1_z ** 2 * P2_z ** 2) * (P1_z + P2_z) * t.sin_t0 + (L ** 2 + P1_z * P2_z) * sqrt(
        L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                    -L ** 4 + P1_z ** 2 * P2_z ** 2) * (-2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * cos(
            T_0) ** 2 + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                       (L ** 2 + P1_z * P2_z) * (L ** 4 - P1_z ** 2 * P2_z ** 2))

    return np.array([PE_x, PE_y, PE_z])


def fk_ori(PE_x, PE_y, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=aNSv-3Ftv3ps&line=2&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0)

    # @title Orientation vectors
    exx = 1 - PE_x ** 2 / (2 * L ** 2 * t.sin_t0_sq)
    exy = -PE_x * PE_y / (2 * L ** 2 * t.sin_t0_sq)
    exz = -PE_x * sqrt(4 * L ** 2 * t.sin_t0_sq - PE_x ** 2 - PE_y ** 2) / (2 * L ** 2 * t.sin_t0_sq)

    eyx = -PE_x * PE_y / (2 * L ** 2 * t.sin_t0_sq)
    eyy = 1 - PE_y ** 2 / (2 * L ** 2 * t.sin_t0_sq)
    eyz = -PE_y * sqrt(4 * L ** 2 * t.sin_t0_sq - PE_x ** 2 - PE_y ** 2) / (2 * L ** 2 * t.sin_t0_sq)

    ezx = PE_x * sqrt(4 * L ** 2 * t.sin_t0_sq - PE_x ** 2 - PE_y ** 2) / (2 * L ** 2 * t.sin_t0_sq)
    ezy = PE_y * sqrt(4 * L ** 2 * t.sin_t0_sq - PE_x ** 2 - PE_y ** 2) / (2 * L ** 2 * t.sin_t0_sq)
    ezz = (2 * L ** 2 - PE_x ** 2 / t.sin_t0_sq - PE_y ** 2 / t.sin_t0_sq) / (2 * L ** 2)


    # @markdown for base change from Wrist to Base generate matrix from base vectors
    # https://de.wikipedia.org/wiki/Basiswechsel_(Vektorraum)
    r_wrist_to_base = np.array(
        [[exx, eyx, ezx],
         [exy, eyy, ezy],
         [exz, eyz, ezz]]
    )
    print(f"r_wrist_to_base:\n{np.round(r_wrist_to_base, 3)}")

    # @markdown since the mechanism is symmetric to the middle just z axis is inverted for Base to wrist
    r_base_to_wrist = np.array(
        [[ exx,  eyx,  ezx],
         [ exy,  eyy,  ezy],
         [-exz, -eyz, -ezz]]
    )
    print(f"r_base_to_wrist:\n{np.round(r_base_to_wrist, 3)}")

    return r_wrist_to_base



def jacobian(P1_z, P2_z, L, T_0):
    t = CommonTerms(L=L, T_0=T_0)

    jac = np.zeros(6)
    jac[0] = [4 * L * P1_z * P2_z ** 2 * (
            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) + 2 * L * (
                      -L ** 3 * P2_z * t.sin_t0 + L * P2_z ** 3 * t.sin_t0 - P2_z * (
                      2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2)), 4 * L * P1_z ** 2 * P2_z * (
                      L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) + 2 * L * P2_z * (
                      L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      (L ** 2 - P2_z ** 2) ** (3 / 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2)) + 2 * L * (
                      -L ** 3 * P1_z * t.sin_t0 - 2 * L ** 3 * P2_z * t.sin_t0 + 3 * L * P1_z * P2_z ** 2 * t.sin_t0 - P2_z * (
                      L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2))]
    jac[1] = [4 * L * P1_z * P2_z ** 2 * (
            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) + 2 * L * P1_z * (
                      L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      (L ** 2 - P1_z ** 2) ** (3 / 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2)) + 2 * L * (
                      -2 * L ** 3 * P1_z * t.sin_t0 - L ** 3 * P2_z * t.sin_t0 + 3 * L * P1_z ** 2 * P2_z * t.sin_t0 - P1_z * (
                      2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2)), 4 * L * P1_z ** 2 * P2_z * (
                      L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) + 2 * L * (
                      -L ** 3 * P1_z * t.sin_t0 + L * P1_z ** 3 * t.sin_t0 - P1_z * (
                      L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0 / (
                      sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2))]
    jac[2] = [4 * L * P1_z * P2_z ** 2 * (
            L * (L ** 4 - P1_z ** 2 * P2_z ** 2) * (P1_z + P2_z) * t.sin_t0 + (L ** 2 + P1_z * P2_z) * sqrt(
        L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                      (L ** 2 + P1_z * P2_z) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L * P2_z * (
                      L * (L ** 4 - P1_z ** 2 * P2_z ** 2) * (P1_z + P2_z) * t.sin_t0 + (L ** 2 + P1_z * P2_z) * sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                          (L ** 2 + P1_z * P2_z) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2)) + 2 * L * (
                      -2 * L * P1_z * P2_z ** 2 * (P1_z + P2_z) * t.sin_t0 + L * (
                      L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0 + P2_z * sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2)) + (
                              L ** 2 + P1_z * P2_z) * (L ** 2 * (2 * L ** 2 - 4 * P1_z * P2_z - 2 * P2_z ** 2) * (
                      L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) * t.sin_t0_sq / 2 + P1_z * P2_z ** 2 * (
                                                               -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2) + (
                                                               -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                                                               2 * L ** 2 * P1_z * t.cos_t0_sq - 2 * P1_z * P2_z ** 2) / 2) / sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                      (L ** 2 + P1_z * P2_z) * (L ** 4 - P1_z ** 2 * P2_z ** 2)), 4 * L * P1_z ** 2 * P2_z * (
                      L * (L ** 4 - P1_z ** 2 * P2_z ** 2) * (P1_z + P2_z) * t.sin_t0 + (L ** 2 + P1_z * P2_z) * sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                          (L ** 2 + P1_z * P2_z) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L * P1_z * (
                      L * (L ** 4 - P1_z ** 2 * P2_z ** 2) * (P1_z + P2_z) * t.sin_t0 + (L ** 2 + P1_z * P2_z) * sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                          (L ** 2 + P1_z * P2_z) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2)) + 2 * L * (
                      -2 * L * P1_z ** 2 * P2_z * (P1_z + P2_z) * t.sin_t0 + L * (
                          L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0 + P1_z * sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2)) + (
                              L ** 2 + P1_z * P2_z) * (L ** 2 * (2 * L ** 2 - 2 * P1_z ** 2 - 4 * P1_z * P2_z) * (
                      L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) * t.sin_t0_sq / 2 + P1_z ** 2 * P2_z * (
                                                               -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2) + (
                                                               -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                                                               2 * L ** 2 * P2_z * t.cos_t0_sq - 2 * P1_z ** 2 * P2_z) / 2) / sqrt(
                  L ** 2 * (L ** 2 * P1_z + L ** 2 * P2_z - P1_z ** 2 * P2_z - P1_z * P2_z ** 2) ** 2 * t.sin_t0_sq + (
                          -L ** 4 + P1_z ** 2 * P2_z ** 2) * (
                          -2 * L ** 4 * t.cos_t0_sq + L ** 4 + L ** 2 * P1_z ** 2 * t.cos_t0_sq + L ** 2 * P2_z ** 2 * t.cos_t0_sq - P1_z ** 2 * P2_z ** 2))) * t.sin_t0 / (
                          (L ** 2 + P1_z * P2_z) * (L ** 4 - P1_z ** 2 * P2_z ** 2))]
    jac[3] = [

        -(2 * L ** 2 - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                  (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                  L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
              -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                  (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (2 * P1_z * P2_z ** 2 * sqrt(
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                                                                                            T_0) - L ** 3 * P1_z * P2_z * sin(
                                                                                            T_0) + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                            L * sqrt(
                                                                                        L ** 2 - P1_z ** 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0) + P1_z * sqrt(
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                                                                                            T_0) - L ** 3 * P1_z * P2_z * sin(
                                                                                            T_0) + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                            L * (
                                                                                            L ** 2 - P1_z ** 2) ** (
                                                                                                    3 / 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0) + sqrt(
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                -2 * L ** 3 * P1_z * t.sin_t0 - L ** 3 * P2_z * sin(
                                                                                            T_0) + 3 * L * P1_z ** 2 * P2_z * sin(
                                                                                            T_0) - P1_z * (
                                                                                                        2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                            L * sqrt(
                                                                                        L ** 2 - P1_z ** 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0) + (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                                                                                            T_0) - L ** 3 * P1_z * P2_z * sin(
                                                                                            T_0) + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                            -8 * L ** 2 * P1_z * P2_z ** 2 * (
                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P2_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 8 * L ** 2 * P1_z * P2_z ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P1_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 4 * L ** 2 * P1_z * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P1_z ** 2) ** 2 * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                    -2 * L ** 3 * P2_z * t.sin_t0 + 2 * L * P2_z ** 3 * t.sin_t0 - 2 * P2_z * (
                                                                                                    2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P2_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                                    -4 * L ** 3 * P1_z * t.sin_t0 - 2 * L ** 3 * P2_z * t.sin_t0 + 6 * L * P1_z ** 2 * P2_z * t.sin_t0 - 2 * P1_z * (
                                                                                                    2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P1_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                                                                                            L * sqrt(
                                                                                        L ** 2 - P1_z ** 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) * sqrt(
                                                                                        4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                (L ** 2 - P2_z ** 2) * (
                                                                                                L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                (L ** 2 - P1_z ** 2) * (
                                                                                                L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * t.sin_t0)) / (
                    2 * L ** 2 * ((4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                           (L ** 2 - P2_z ** 2) * (
                                           L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                           (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                          L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                      -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                          L ** 2 * (L ** 2 - P1_z ** 2) * (
                                          L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0_sq) + (
                                          2 * L ** 2 - 4 * L ** 2 * (
                                          L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                      -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                  (L ** 2 - P2_z ** 2) * (
                                                  L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                  L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                              -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                  (L ** 2 - P1_z ** 2) * (
                                                  L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 / (4 * L ** 4))) + sqrt(
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                -16 * L ** 2 * P1_z * P2_z ** 2 * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P2_z ** 2) * (
                        L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 16 * L ** 2 * P1_z * P2_z ** 2 * (
                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 8 * L ** 2 * P1_z * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P1_z ** 2) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                        -2 * L ** 3 * P2_z * t.sin_t0 + 2 * L * P2_z ** 3 * t.sin_t0 - 2 * P2_z * (
                        2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        -4 * L ** 3 * P1_z * t.sin_t0 - 2 * L ** 3 * P2_z * t.sin_t0 + 6 * L * P1_z ** 2 * P2_z * t.sin_t0 - 2 * P1_z * (
                        2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                2 * L ** 3 * sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * ((
                                                                                                    4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                            (
                                                                                                                    L ** 2 - P2_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                            (
                                                                                                                    L ** 2 - P1_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                    L ** 2 * (
                                                                                                    L ** 2 - P1_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0_sq) + (
                                                                                                    2 * L ** 2 - 4 * L ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                            (
                                                                                                                    L ** 2 - P2_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                            (
                                                                                                                    L ** 2 - P1_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 / (
                                                                                                    4 * L ** 4)) * t.sin_t0),
        -(2 * L ** 2 - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                  (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                  L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
              -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                  (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (2 * P1_z ** 2 * P2_z * sqrt(
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                            L * sqrt(
                                                                                        L ** 2 - P1_z ** 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0) + sqrt(
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                            -L ** 3 * P1_z * t.sin_t0 + L * P1_z ** 3 * t.sin_t0 - P1_z * (
                                                                                            L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                            L * sqrt(
                                                                                        L ** 2 - P1_z ** 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0) + (
                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                            -8 * L ** 2 * P1_z ** 2 * P2_z * (
                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P2_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 8 * L ** 2 * P1_z ** 2 * P2_z * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P1_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 4 * L ** 2 * P2_z * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P2_z ** 2) ** 2 * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                                    -2 * L ** 3 * P1_z * t.sin_t0 - 4 * L ** 3 * P2_z * t.sin_t0 + 6 * L * P1_z * P2_z ** 2 * t.sin_t0 - 2 * P2_z * (
                                                                                                    L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P2_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                    -2 * L ** 3 * P1_z * t.sin_t0 + 2 * L * P1_z ** 3 * t.sin_t0 - 2 * P1_z * (
                                                                                                    L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                                                                                    (
                                                                                                            L ** 2 - P1_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                                                                                            L * sqrt(
                                                                                        L ** 2 - P1_z ** 2) * (
                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) * sqrt(
                                                                                        4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                (L ** 2 - P2_z ** 2) * (
                                                                                                L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                (L ** 2 - P1_z ** 2) * (
                                                                                                L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * t.sin_t0)) / (
                2 * L ** 2 * ((4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                       (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                                   T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * sin(
                                   T_0) - P1_z * sqrt(
                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                       (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                      L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                                  T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * sin(
                                  T_0) - P1_z * sqrt(
                                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                      L ** 2 * (L ** 2 - P1_z ** 2) * (
                                      L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0_sq) + (2 * L ** 2 - 4 * L ** 2 * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                     (
                                                                                                             L ** 2 - P2_z ** 2) * (
                                                                                                             L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                     L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                 -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                     (
                                                                                                             L ** 2 - P1_z ** 2) * (
                                                                                                             L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 / (
                                      4 * L ** 4))) + sqrt(4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                   (L ** 2 - P2_z ** 2) * (
                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                   L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                               -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                   (L ** 2 - P1_z ** 2) * (
                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                -16 * L ** 2 * P1_z ** 2 * P2_z * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P2_z ** 2) * (
                        L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 16 * L ** 2 * P1_z ** 2 * P2_z * (
                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 8 * L ** 2 * P2_z * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P2_z ** 2) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        -2 * L ** 3 * P1_z * t.sin_t0 - 4 * L ** 3 * P2_z * t.sin_t0 + 6 * L * P1_z * P2_z ** 2 * t.sin_t0 - 2 * P2_z * (
                        L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                        -2 * L ** 3 * P1_z * t.sin_t0 + 2 * L * P1_z ** 3 * t.sin_t0 - 2 * P1_z * (
                        L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                2 * L ** 3 * sqrt(L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * ((
                                                                                                    4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                            (
                                                                                                                    L ** 2 - P2_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                                                                            (
                                                                                                                    L ** 2 - P1_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                    L ** 2 * (
                                                                                                    L ** 2 - P1_z ** 2) * (
                                                                                                            L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0_sq) + (
                                                                                                    2 * L ** 2 - 4 * L ** 2 * (
                                                                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                            (
                                                                                                                    L ** 2 - P2_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                            (
                                                                                                                    L ** 2 - P1_z ** 2) * (
                                                                                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 / (
                                                                                                    4 * L ** 4)) * t.sin_t0)

    ]
    jac[4] = [-(2 * P1_z * P2_z ** 2 * sqrt(4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                    (L ** 2 - P2_z ** 2) * (
                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        L * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0) + sqrt(
        4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                        -L ** 3 * P2_z * t.sin_t0 + L * P2_z ** 3 * t.sin_t0 - P2_z * (
                        2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        L * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0) + (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        -8 * L ** 2 * P1_z * P2_z ** 2 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                (L ** 2 - P2_z ** 2) * (
                                L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 8 * L ** 2 * P1_z * P2_z ** 2 * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 4 * L ** 2 * P1_z * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                (L ** 2 - P1_z ** 2) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                -2 * L ** 3 * P2_z * t.sin_t0 + 2 * L * P2_z ** 3 * t.sin_t0 - 2 * P2_z * (
                                2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                -4 * L ** 3 * P1_z * t.sin_t0 - 2 * L ** 3 * P2_z * t.sin_t0 + 6 * L * P1_z ** 2 * P2_z * t.sin_t0 - 2 * P1_z * (
                                2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                        L * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * sqrt(
                    4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                            (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                            (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * t.sin_t0)) / sqrt(1 - (
            4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * sin(
                T_0) - P1_z * sqrt(
                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                                       L ** 2 * (
                                                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                                               L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0_sq)),
              -(2 * P1_z ** 2 * P2_z * sqrt(4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                      L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                    (L ** 2 - P2_z ** 2) * (
                                                    L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                                                    L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                                    (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        L * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0) + P2_z * sqrt(
                  4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                          L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                      -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                          (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                          L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * sin(
                      T_0) - P1_z * sqrt(
                      -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                          (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        L * (L ** 2 - P2_z ** 2) ** (3 / 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0) + sqrt(
                  4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                          L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                      -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                          (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                          L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                      -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                          (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                        -L ** 3 * P1_z * t.sin_t0 - 2 * L ** 3 * P2_z * t.sin_t0 + 3 * L * P1_z * P2_z ** 2 * t.sin_t0 - P2_z * (
                        L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        L * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * t.sin_t0) + (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        -8 * L ** 2 * P1_z ** 2 * P2_z * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                (L ** 2 - P2_z ** 2) * (
                                L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 8 * L ** 2 * P1_z ** 2 * P2_z * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 4 * L ** 2 * P2_z * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                                (L ** 2 - P2_z ** 2) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                -2 * L ** 3 * P1_z * t.sin_t0 - 4 * L ** 3 * P2_z * t.sin_t0 + 6 * L * P1_z * P2_z ** 2 * t.sin_t0 - 2 * P2_z * (
                                L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * L ** 2 * (
                                -2 * L ** 3 * P1_z * t.sin_t0 + 2 * L * P1_z ** 3 * t.sin_t0 - 2 * P1_z * (
                                L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * t.sin_t0_sq / (
                                (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                        L * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) * sqrt(
                    4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                            L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                            (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                            L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                        -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                            (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * t.sin_t0)) / sqrt(1 - (
                      4 * L ** 2 * t.sin_t0_sq - 4 * L ** 2 * (
                      L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                  -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                              (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 4 * L ** 2 * (
                              L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * sin(
                          T_0) - P1_z * sqrt(
                          -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * t.sin_t0_sq / (
                              (L ** 2 - P1_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                                       L ** 2 * (
                                                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                                               L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * t.sin_t0_sq))

              ]
    jac[5] = [

        (1 - 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                 (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (-8 * P1_z * P2_z ** 2 * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
            T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(
                                                                                               L ** 2 - P1_z ** 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 2 * P1_z * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           (L ** 2 - P1_z ** 2) ** (
                                                                                           3 / 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * (
                                                                                           -L ** 3 * P2_z * t.sin_t0 + L * P2_z ** 3 * t.sin_t0 - P2_z * (
                                                                                           2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(
                                                                                               L ** 2 - P1_z ** 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           -2 * L ** 3 * P1_z * t.sin_t0 - L ** 3 * P2_z * t.sin_t0 + 3 * L * P1_z ** 2 * P2_z * t.sin_t0 - P1_z * (
                                                                                           2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(
                                                                                               L ** 2 - P1_z ** 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                (1 - 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                         (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 + 4 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                    T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P1_z ** 2) * (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 4)) + 2 * (
                -8 * P1_z * P2_z ** 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 2 * (
                        -2 * L ** 3 * P2_z * t.sin_t0 + 2 * L * P2_z ** 3 * t.sin_t0 - 2 * P2_z * (
                        2 * L ** 6 * P1_z * t.sin_t0_sq - L ** 6 * P1_z + L ** 6 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq - L ** 4 * P2_z ** 3 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 3 * P2_z ** 2 + 3 * L ** 2 * P1_z ** 2 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z * P2_z ** 4 - 2 * P1_z ** 3 * P2_z ** 4) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
            T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                sqrt(L ** 2 - P1_z ** 2) * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * ((
                                                                                                                       1 - 2 * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                                               (
                                                                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                                                       L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 + 4 * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                                       (
                                                                                                                               L ** 2 - P1_z ** 2) * (
                                                                                                                               L ** 2 - P2_z ** 2) * (
                                                                                                                               L ** 4 - P1_z ** 2 * P2_z ** 2) ** 4))),
        (1 - 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                 (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (-8 * P1_z ** 2 * P2_z * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
            T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(
                                                                                               L ** 2 - P1_z ** 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 2 * P2_z * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(L ** 2 - P1_z ** 2) * (
                                                                                           L ** 2 - P2_z ** 2) ** (
                                                                                                   3 / 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * (
                                                                                           -L ** 3 * P1_z * t.sin_t0 + L * P1_z ** 3 * t.sin_t0 - P1_z * (
                                                                                           L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(
                                                                                               L ** 2 - P1_z ** 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * (
                                                                                           L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                                                                                           -L ** 3 * P1_z * t.sin_t0 - 2 * L ** 3 * P2_z * t.sin_t0 + 3 * L * P1_z * P2_z ** 2 * t.sin_t0 - P2_z * (
                                                                                           L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - sqrt(
                                                                                       -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                                                                                           sqrt(
                                                                                               L ** 2 - P1_z ** 2) * sqrt(
                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                   L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) / (
                (1 - 2 * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                         (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 + 4 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
                    T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P1_z ** 2) * (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 4)) + 2 * (
                -8 * P1_z ** 2 * P2_z * (L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 3) - 4 * P2_z * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                        (L ** 2 - P2_z ** 2) ** 2 * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2) - 2 * (
                        L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
                    T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                        -2 * L ** 3 * P1_z * t.sin_t0 - 4 * L ** 3 * P2_z * sin(
                    T_0) + 6 * L * P1_z * P2_z ** 2 * t.sin_t0 - 2 * P2_z * (
                                L ** 6 * P1_z * t.sin_t0_sq + 2 * L ** 6 * P2_z * t.sin_t0_sq - L ** 6 * P2_z - L ** 4 * P1_z ** 3 * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z * t.sin_t0_sq - 3 * L ** 4 * P1_z * P2_z ** 2 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z + 3 * L ** 2 * P1_z ** 3 * P2_z ** 2 * t.sin_t0_sq + 2 * L ** 2 * P1_z ** 2 * P2_z ** 3 - 2 * P1_z ** 4 * P2_z ** 3) / sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4) - 2 * sqrt(
                    -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                        (L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * sin(
            T_0) - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) * (
                L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * sin(
            T_0) - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
            -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) / (
                sqrt(L ** 2 - P1_z ** 2) * sqrt(L ** 2 - P2_z ** 2) * (L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2 * ((
                                                                                                                       1 - 2 * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                                               (
                                                                                                                                       L ** 2 - P2_z ** 2) * (
                                                                                                                                       L ** 4 - P1_z ** 2 * P2_z ** 2) ** 2)) ** 2 + 4 * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z ** 2 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 + L * P1_z ** 3 * P2_z * t.sin_t0 - P1_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 * (
                                                                                                                       L ** 5 * t.sin_t0 - L ** 3 * P1_z * P2_z * t.sin_t0 - L ** 3 * P2_z ** 2 * t.sin_t0 + L * P1_z * P2_z ** 3 * t.sin_t0 - P2_z * sqrt(
                                                                                                                   -2 * L ** 8 * t.sin_t0_sq + L ** 8 + 2 * L ** 6 * P1_z ** 2 * t.sin_t0_sq - L ** 6 * P1_z ** 2 + 2 * L ** 6 * P1_z * P2_z * t.sin_t0_sq + 2 * L ** 6 * P2_z ** 2 * t.sin_t0_sq - L ** 6 * P2_z ** 2 - 2 * L ** 4 * P1_z ** 3 * P2_z * t.sin_t0_sq - 2 * L ** 4 * P1_z ** 2 * P2_z ** 2 * t.sin_t0_sq - 2 * L ** 4 * P1_z * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 4 * P2_z ** 2 + 2 * L ** 2 * P1_z ** 3 * P2_z ** 3 * t.sin_t0_sq + L ** 2 * P1_z ** 2 * P2_z ** 4 - P1_z ** 4 * P2_z ** 4)) ** 2 / (
                                                                                                                       (
                                                                                                                               L ** 2 - P1_z ** 2) * (
                                                                                                                               L ** 2 - P2_z ** 2) * (
                                                                                                                               L ** 4 - P1_z ** 2 * P2_z ** 2) ** 4)))]
    return jac
