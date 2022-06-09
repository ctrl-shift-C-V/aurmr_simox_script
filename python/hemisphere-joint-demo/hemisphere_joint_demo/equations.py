import numpy as np

from numpy import pi, sin, cos, sqrt, arcsin, arctan2

from .terms import CommonTerms


def fk_pos(a1, a2, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=za3fZw9Rq5d9&line=1&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0, a1=a1, a2=a2)

    ex = 2 * L * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) * t.sin_t0 / (
                 sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq)
    ey = 2 * L * (
            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_03) * t.sin_t0 / (
                 sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq)
    ez = 2 * L * (L * t.l_p4_minus_a1_sq_mul_a2_sq * (a1 + a2) * t.sin_t0 + (t.l_p2 + a1 * a2) * sqrt(
        t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                 (t.l_p2 + a1 * a2) * t.l_p4_minus_a1_sq_mul_a2_sq)

    return np.array([ex, ey, ez])


def fk_ori(ex, ey, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=aNSv-3Ftv3ps&line=2&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0, ex=ex, ey=ey)

    # @title Orientation vectors
    exx = 1 - t.ex_p2 / t.two_l_p2_mul_sin_t0_sq
    exy = -t.ex_mul_ey / t.two_l_p2_mul_sin_t0_sq
    exz = -ex * t.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq_div_two_l_p2_mul_sin_t0_sq

    eyx = -t.ex_mul_ey / t.two_l_p2_mul_sin_t0_sq
    eyy = 1 - t.ey_p2 / t.two_l_p2_mul_sin_t0_sq
    eyz = -ey * t.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq_div_two_l_p2_mul_sin_t0_sq

    ezx = ex * t.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq_div_two_l_p2_mul_sin_t0_sq
    ezy = ey * t.sqrt_4_l_p2_mul_sin_t0_sq_mul_ex_sq_mul_ey_sq_div_two_l_p2_mul_sin_t0_sq
    ezz = (2 * t.l_p2 - t.ex_p2 / t.sin_t0_sq - t.ey_p2 / t.sin_t0_sq) / (2 * t.l_p2)

    # @markdown for base change from Wrist to Base generate matrix from base vectors
    # https://de.wikipedia.org/wiki/Basiswechsel_(Vektorraum)
    r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
    # print(f"r_wrist_to_base:\n{np.round(r_wrist_to_base, 3)}")

    # @markdown since the mechanism is symmetric to the middle just z axis is inverted for Base to wrist
    r_base_to_wrist = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [-exz, -eyz, -ezz]])
    # print(f"r_base_to_wrist:\n{np.round(r_base_to_wrist, 3)}")

    return r_wrist_to_base














































def jacobian(a1, a2, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=xx7_60I1sV9j&line=1&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0, a1=a1, a2=a2)
    
    # Jacobian of [ex, ey, ez, azimuth, declination]

    jac = np.zeros((5, 2), dtype=float)
    
    jac[0, 0] = (4 * L * a1 * a2 ** 2 * (L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (
                         sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) + 2 * L * (
                         -L ** 3 * a2 * sin(T_0) + L * a2 ** 3 * sin(T_0) - a2 * (
                             2 * L ** 6 * a1 * sin(T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                         T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                         T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                     -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (
                         sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2)))
    
    jac[0, 1] = (4 * L * a1 ** 2 * a2 * (
                         L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                     T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                     -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (
                         sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) + 2 * L * a2 * (
                         L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                     T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                     -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (
                         (L ** 2 - a2 ** 2) ** (3 / 2) * (L ** 4 - a1 ** 2 * a2 ** 2)) + 2 * L * (
                         -L ** 3 * a1 * sin(T_0) - 2 * L ** 3 * a2 * sin(T_0) + 3 * L * a1 * a2 ** 2 * sin(
                     T_0) - a2 * (L ** 6 * a1 * sin(T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                     T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                     T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                     T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                     T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                     -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - sqrt(
                     -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (
                         sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2)))

    jac[1] = [4 * L * a1 * a2 ** 2 * (
                L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
            T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (sqrt(
        L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) + 2 * L * a1 * (L ** 5 * sin(
        T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
        T_0) - a1 * sqrt(-2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / ((L ** 2 - a1 ** 2) ** (
                3 / 2) * (L ** 4 - a1 ** 2 * a2 ** 2)) + 2 * L * (-2 * L ** 3 * a1 * sin(
        T_0) - L ** 3 * a2 * sin(T_0) + 3 * L * a1 ** 2 * a2 * sin(T_0) - a1 * (2 * L ** 6 * a1 * sin(
        T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
        T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
        T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - sqrt(
        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (sqrt(
        L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2)),

 4 * L * a1 ** 2 * a2 * (L ** 5 * sin(
        T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
        T_0) - a1 * sqrt(-2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (sqrt(L ** 2 - a1 ** 2) * (
                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) + 2 * L * (-L ** 3 * a1 * sin(T_0) + L * a1 ** 3 * sin(
        T_0) - a1 * (L ** 6 * a1 * sin(T_0) ** 2 + 2 * L ** 6 * a2 * sin(
        T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
        T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
        T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(T_0) / (sqrt(
        L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2))]

    jac[2] = [4 * L * a1 * a2 ** 2 * (
                L * (L ** 4 - a1 ** 2 * a2 ** 2) * (a1 + a2) * sin(T_0) + (L ** 2 + a1 * a2) * sqrt(
            L ** 2 * (L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(T_0) ** 2 + (
                        -L ** 4 + a1 ** 2 * a2 ** 2) * (
                        -2 * L ** 4 * cos(T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                    T_0) ** 2 + L ** 2 * a2 ** 2 * cos(T_0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(T_0) / (
                                                                               (L ** 2 + a1 * a2) * (
                                                                                   L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L * a2 * (
                                                                               L * (L ** 4 - a1 ** 2 * a2 ** 2) * (
                                                                                   a1 + a2) * sin(T_0) + (
                                                                                           L ** 2 + a1 * a2) * sqrt(
                                                                           L ** 2 * (
                                                                                       L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(
                                                                               T_0) ** 2 + (
                                                                                       -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                       -2 * L ** 4 * cos(
                                                                                   T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                   T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                   T_0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(
        T_0) / ((L ** 2 + a1 * a2) ** 2 * (L ** 4 - a1 ** 2 * a2 ** 2)) + 2 * L * (-2 * L * a1 * a2 ** 2 * (
                a1 + a2) * sin(T_0) + L * (L ** 4 - a1 ** 2 * a2 ** 2) * sin(T_0) + a2 * sqrt(
        L ** 2 * (L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(T_0) ** 2 + (
                    -L ** 4 + a1 ** 2 * a2 ** 2) * (-2 * L ** 4 * cos(T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
            T_0) ** 2 + L ** 2 * a2 ** 2 * cos(T_0) ** 2 - a1 ** 2 * a2 ** 2)) + (L ** 2 + a1 * a2) * (
                                                                                                       L ** 2 * (
                                                                                                           2 * L ** 2 - 4 * a1 * a2 - 2 * a2 ** 2) * (
                                                                                                                   L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) * sin(
                                                                                                   T_0) ** 2 / 2 + a1 * a2 ** 2 * (
                                                                                                                   -2 * L ** 4 * cos(
                                                                                                               T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                                               T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                                               T_0) ** 2 - a1 ** 2 * a2 ** 2) + (
                                                                                                                   -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                                                   2 * L ** 2 * a1 * cos(
                                                                                                               T_0) ** 2 - 2 * a1 * a2 ** 2) / 2) / sqrt(
        L ** 2 * (L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(T_0) ** 2 + (
                    -L ** 4 + a1 ** 2 * a2 ** 2) * (-2 * L ** 4 * cos(T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
            T_0) ** 2 + L ** 2 * a2 ** 2 * cos(T_0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(T_0) / (
                                                                               (L ** 2 + a1 * a2) * (
                                                                                   L ** 4 - a1 ** 2 * a2 ** 2)),


                                                                   4 * L * a1 ** 2 * a2 * (
                                                                               L * (L ** 4 - a1 ** 2 * a2 ** 2) * (
                                                                                   a1 + a2) * sin(T_0) + (
                                                                                           L ** 2 + a1 * a2) * sqrt(
                                                                           L ** 2 * (
                                                                                       L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(
                                                                               T_0) ** 2 + (
                                                                                       -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                       -2 * L ** 4 * cos(
                                                                                   T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                   T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                   T_0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(
                                                                       T_0) / ((L ** 2 + a1 * a2) * (
                                                                               L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L * a1 * (
                                                                               L * (L ** 4 - a1 ** 2 * a2 ** 2) * (
                                                                                   a1 + a2) * sin(T_0) + (
                                                                                           L ** 2 + a1 * a2) * sqrt(
                                                                           L ** 2 * (
                                                                                       L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(
                                                                               T_0) ** 2 + (
                                                                                       -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                       -2 * L ** 4 * cos(
                                                                                   T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                   T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                   T_0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(
                                                                       T_0) / ((L ** 2 + a1 * a2) ** 2 * (
                                                                               L ** 4 - a1 ** 2 * a2 ** 2)) + 2 * L * (
                                                                               -2 * L * a1 ** 2 * a2 * (
                                                                                   a1 + a2) * sin(T_0) + L * (
                                                                                           L ** 4 - a1 ** 2 * a2 ** 2) * sin(
                                                                           T_0) + a1 * sqrt(L ** 2 * (
                                                                                   L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(
                                                                           T_0) ** 2 + (
                                                                                                          -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                                          -2 * L ** 4 * cos(
                                                                                                      T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                                      T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                                      T_0) ** 2 - a1 ** 2 * a2 ** 2)) + (
                                                                                           L ** 2 + a1 * a2) * (
                                                                                           L ** 2 * (
                                                                                               2 * L ** 2 - 2 * a1 ** 2 - 4 * a1 * a2) * (
                                                                                                       L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) * sin(
                                                                                       T_0) ** 2 / 2 + a1 ** 2 * a2 * (
                                                                                                       -2 * L ** 4 * cos(
                                                                                                   T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                                   T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                                   T_0) ** 2 - a1 ** 2 * a2 ** 2) + (
                                                                                                       -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                                       2 * L ** 2 * a2 * cos(
                                                                                                   T_0) ** 2 - 2 * a1 ** 2 * a2) / 2) / sqrt(
                                                                           L ** 2 * (
                                                                                       L ** 2 * a1 + L ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(
                                                                               T_0) ** 2 + (
                                                                                       -L ** 4 + a1 ** 2 * a2 ** 2) * (
                                                                                       -2 * L ** 4 * cos(
                                                                                   T_0) ** 2 + L ** 4 + L ** 2 * a1 ** 2 * cos(
                                                                                   T_0) ** 2 + L ** 2 * a2 ** 2 * cos(
                                                                                   T_0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(
                                                                       T_0) / ((L ** 2 + a1 * a2) * (
                                                                               L ** 4 - a1 ** 2 * a2 ** 2))]

    jac[3] = [
                -sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                            L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                    T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                  L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                              T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                              -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                  T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                  T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                  T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                  T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                  T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                  T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                  T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                    T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                            2 * a1 * a2 ** 2 * sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                            T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                                T_0) - L ** 3 * a1 * a2 * sin(
                                                                T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                                -2 * L ** 8 * sin(
                                                                    T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                        L * sqrt(L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(
                                    T_0)) + a1 * sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                            T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                                T_0) - L ** 3 * a1 * a2 * sin(
                                                                T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                                -2 * L ** 8 * sin(
                                                                    T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                        L * (L ** 2 - a1 ** 2) ** (3 / 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * sin(
                                    T_0)) + sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                            T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                             L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                         T_0) - L ** 3 * a1 * a2 * sin(
                                                         T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                         -2 * L ** 8 * sin(
                                                             T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                             T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                             T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                             T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                             T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                             T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                             T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                             T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                        -2 * L ** 3 * a1 * sin(T_0) - L ** 3 * a2 * sin(
                                    T_0) + 3 * L * a1 ** 2 * a2 * sin(T_0) - a1 * (2 * L ** 6 * a1 * sin(
                                    T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                    T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                    T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                    T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                    T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                        L * sqrt(L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * sin(T_0)) + (
                                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                        -8 * L ** 2 * a1 * a2 ** 2 * (
                                            L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                        T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                        T_0) - a2 * sqrt(
                                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                    T_0) ** 2 / ((L ** 2 - a2 ** 2) * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 8 * L ** 2 * a1 * a2 ** 2 * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                                T_0) - a1 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                    T_0) ** 2 / ((L ** 2 - a1 ** 2) * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 4 * L ** 2 * a1 * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                                T_0) - a1 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                    T_0) ** 2 / ((L ** 2 - a1 ** 2) ** 2 * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                    -2 * L ** 3 * a2 * sin(T_0) + 2 * L * a2 ** 3 * sin(
                                                T_0) - 2 * a2 * (2 * L ** 6 * a1 * sin(
                                                T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                                T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                                T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                                T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                                T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                                T_0) - a2 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                    T_0) ** 2 / ((L ** 2 - a2 ** 2) * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                                T_0) - a1 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                    -4 * L ** 3 * a1 * sin(T_0) - 2 * L ** 3 * a2 * sin(
                                                T_0) + 6 * L * a1 ** 2 * a2 * sin(T_0) - 2 * a1 * (
                                                                2 * L ** 6 * a1 * sin(
                                                            T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                                            T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                                            T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                                            T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                                            T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - 2 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                    T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) / (
                                        L * sqrt(L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * sqrt(
                                    4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                            T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                            T_0) - a2 * sqrt(
                                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (
                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                            T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                            T_0) - a1 * sqrt(
                                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                        T_0) ** 2 / (
                                                (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * sin(
                                    T_0))) * (
                            L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                            L * sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * ((4 * L ** 2 * sin(
                        T_0) ** 2 - 4 * L ** 2 * (L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                        T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * sin(
                                                                                                        T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                        T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                        T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                        T_0) - a1 * sqrt(
                                                                                                        -2 * L ** 8 * sin(
                                                                                                            T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
                        T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (L ** 2 * (
                                L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(T_0) ** 2) + (
                                                                                                           4 * L ** 2 * sin(
                                                                                                       T_0) ** 2 - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                                   T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) - a2 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a2 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) - a1 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a1 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                                                                                           L ** 5 * sin(
                                                                                                       T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                       T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                       T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                       T_0) - a1 * sqrt(
                                                                                                       -2 * L ** 8 * sin(
                                                                                                           T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                                           L ** 2 * (
                                                                                                               L ** 2 - a1 ** 2) * (
                                                                                                                       L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(
                                                                                                       T_0) ** 2)) * sin(
                        T_0)) + sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                            L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                    T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                 L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                             T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                             T_0) - a1 * sqrt(
                                             -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                 T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                    T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                            2 * a1 * a2 ** 2 * sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                            T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                                T_0) - L ** 3 * a1 * a2 * sin(
                                                                T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                                -2 * L ** 8 * sin(
                                                                    T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                        L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                                    T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                        L * sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(
                                    T_0)) + sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                            T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                             L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                         T_0) - L ** 3 * a1 * a2 * sin(
                                                         T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                         -2 * L ** 8 * sin(
                                                             T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                             T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                             T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                             T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                             T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                             T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                             T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                             T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                        -L ** 3 * a2 * sin(T_0) + L * a2 ** 3 * sin(T_0) - a2 * (
                                            2 * L ** 6 * a1 * sin(T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                        T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                        T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                        L * sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * sin(T_0)) + (
                                        L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                                    T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                        -8 * L ** 2 * a1 * a2 ** 2 * (
                                            L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                        T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                        T_0) - a2 * sqrt(
                                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                    T_0) ** 2 / ((L ** 2 - a2 ** 2) * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 8 * L ** 2 * a1 * a2 ** 2 * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                                T_0) - a1 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                    T_0) ** 2 / ((L ** 2 - a1 ** 2) * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 4 * L ** 2 * a1 * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                                T_0) - a1 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                    T_0) ** 2 / ((L ** 2 - a1 ** 2) ** 2 * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                    -2 * L ** 3 * a2 * sin(T_0) + 2 * L * a2 ** 3 * sin(
                                                T_0) - 2 * a2 * (2 * L ** 6 * a1 * sin(
                                                T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                                T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                                T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                                T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                                T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                                T_0) - a2 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                    T_0) ** 2 / ((L ** 2 - a2 ** 2) * (
                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                    L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                                T_0) - a1 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                    -4 * L ** 3 * a1 * sin(T_0) - 2 * L ** 3 * a2 * sin(
                                                T_0) + 6 * L * a1 ** 2 * a2 * sin(T_0) - 2 * a1 * (
                                                                2 * L ** 6 * a1 * sin(
                                                            T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                                            T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                                            T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                                            T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                                            T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - 2 * sqrt(
                                                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                    T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) / (
                                        L * sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * sqrt(
                                    4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                                                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                            T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                            T_0) - a2 * sqrt(
                                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (
                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                            T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                            T_0) - a1 * sqrt(
                                            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                        T_0) ** 2 / (
                                                (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * sin(
                                    T_0))) * (
                            L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                        T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                            L * sqrt(L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * ((4 * L ** 2 * sin(
                        T_0) ** 2 - 4 * L ** 2 * (L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                        T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * sin(
                                                                                                        T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                        T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                        T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                        T_0) - a1 * sqrt(
                                                                                                        -2 * L ** 8 * sin(
                                                                                                            T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
                        T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (L ** 2 * (
                                L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(T_0) ** 2) + (
                                                                                                           4 * L ** 2 * sin(
                                                                                                       T_0) ** 2 - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                                   T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) - a2 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a2 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) - a1 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a1 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                                                                                           L ** 5 * sin(
                                                                                                       T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                       T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                       T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                       T_0) - a1 * sqrt(
                                                                                                       -2 * L ** 8 * sin(
                                                                                                           T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                                           L ** 2 * (
                                                                                                               L ** 2 - a1 ** 2) * (
                                                                                                                       L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(
                                                                                                       T_0) ** 2)) * sin(
                        T_0)),

 -sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                    L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                                                 (L ** 2 - a2 ** 2) * (
                                                     L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                 L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                             T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                             T_0) - a1 * sqrt(
                                             -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                 T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
            T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (2 * a1 ** 2 * a2 * sqrt(
            4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                    T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
            T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
            T_0) - a1 * sqrt(-2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (L * sqrt(L ** 2 - a1 ** 2) * (
                    L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(T_0)) + sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                    L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                                                                                        (L ** 2 - a2 ** 2) * (
                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                        L ** 5 * sin(
                                                                                    T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                    T_0) - L ** 3 * a1 * a2 * sin(
                                                                                    T_0) + L * a1 ** 3 * a2 * sin(
                                                                                    T_0) - a1 * sqrt(
                                                                                    -2 * L ** 8 * sin(
                                                                                        T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
            T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (-L ** 3 * a1 * sin(
            T_0) + L * a1 ** 3 * sin(T_0) - a1 * (L ** 6 * a1 * sin(T_0) ** 2 + 2 * L ** 6 * a2 * sin(
            T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
            T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
            T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (L * sqrt(L ** 2 - a1 ** 2) * (
                    L ** 4 - a1 ** 2 * a2 ** 2) * sin(T_0)) + (L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
            T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                                                                       -8 * L ** 2 * a1 ** 2 * a2 * (
                                                                                                           L ** 5 * sin(
                                                                                                       T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                       T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                       T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                       T_0) - a2 * sqrt(
                                                                                                       -2 * L ** 8 * sin(
                                                                                                           T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a2 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 8 * L ** 2 * a1 ** 2 * a2 * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                               T_0) - a1 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a1 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 4 * L ** 2 * a2 * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                               T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                               T_0) - a2 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a2 ** 2) ** 2 * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                               T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                               T_0) - a2 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                                                                                   -2 * L ** 3 * a1 * sin(
                                                                                                               T_0) - 4 * L ** 3 * a2 * sin(
                                                                                                               T_0) + 6 * L * a1 * a2 ** 2 * sin(
                                                                                                               T_0) - 2 * a2 * (
                                                                                                                               L ** 6 * a1 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                                                                                                                           T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
                                                                                                                           T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - 2 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a2 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                                   -2 * L ** 3 * a1 * sin(
                                                                                                               T_0) + 2 * L * a1 ** 3 * sin(
                                                                                                               T_0) - 2 * a1 * (
                                                                                                                               L ** 6 * a1 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                                                                                                                           T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
                                                                                                                           T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                               T_0) - a1 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a1 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) / (
                                                                                                       L * sqrt(
                                                                                                   L ** 2 - a1 ** 2) * (
                                                                                                                   L ** 4 - a1 ** 2 * a2 ** 2) * sqrt(
                                                                                                   4 * L ** 2 * sin(
                                                                                                       T_0) ** 2 - 4 * L ** 2 * (
                                                                                                               L ** 5 * sin(
                                                                                                           T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                           T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                           T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                           T_0) - a2 * sqrt(
                                                                                                           -2 * L ** 8 * sin(
                                                                                                               T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                               T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a2 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                               L ** 5 * sin(
                                                                                                           T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                           T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                           T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                           T_0) - a1 * sqrt(
                                                                                                           -2 * L ** 8 * sin(
                                                                                                               T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                               T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a1 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * sin(
                                                                                                   T_0))) * (
                            L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                            L * sqrt(L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * ((4 * L ** 2 * sin(
                        T_0) ** 2 - 4 * L ** 2 * (L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                        T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * sin(
                                                                                                        T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                        T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                        T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                        T_0) - a1 * sqrt(
                                                                                                        -2 * L ** 8 * sin(
                                                                                                            T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
                        T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (L ** 2 * (
                                L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(T_0) ** 2) + (
                                                                                                           4 * L ** 2 * sin(
                                                                                                       T_0) ** 2 - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                                   T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) - a2 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a2 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) - a1 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a1 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                                                                                           L ** 5 * sin(
                                                                                                       T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                       T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                       T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                       T_0) - a1 * sqrt(
                                                                                                       -2 * L ** 8 * sin(
                                                                                                           T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                                           L ** 2 * (
                                                                                                               L ** 2 - a1 ** 2) * (
                                                                                                                       L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(
                                                                                                       T_0) ** 2)) * sin(
                        T_0)) + sqrt(4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                    L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                    T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                    T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                    T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                    T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                    T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                    T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                    T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                                                 (L ** 2 - a2 ** 2) * (
                                                     L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                 L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                             T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                             T_0) - a1 * sqrt(
                                             -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                 T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
            T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (2 * a1 ** 2 * a2 * sqrt(
            4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                    T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
            T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
            T_0) - a2 * sqrt(-2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (L * sqrt(L ** 2 - a2 ** 2) * (
                    L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(T_0)) + a2 * sqrt(
            4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                    T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
            T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
            T_0) - a2 * sqrt(-2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (L * (L ** 2 - a2 ** 2) ** (
                    3 / 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * sin(T_0)) + sqrt(
            4 * L ** 2 * sin(T_0) ** 2 - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                    T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                        L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                    T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                    -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                        T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                        T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                        T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                        T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(T_0) ** 2 / (
                        (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (-L ** 3 * a1 * sin(
            T_0) - 2 * L ** 3 * a2 * sin(T_0) + 3 * L * a1 * a2 ** 2 * sin(T_0) - a2 * (L ** 6 * a1 * sin(
            T_0) ** 2 + 2 * L ** 6 * a2 * sin(T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
            T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (L * sqrt(L ** 2 - a2 ** 2) * (
                    L ** 4 - a1 ** 2 * a2 ** 2) * sin(T_0)) + (L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
            T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                                                                       -8 * L ** 2 * a1 ** 2 * a2 * (
                                                                                                           L ** 5 * sin(
                                                                                                       T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                       T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                       T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                       T_0) - a2 * sqrt(
                                                                                                       -2 * L ** 8 * sin(
                                                                                                           T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a2 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 8 * L ** 2 * a1 ** 2 * a2 * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                               T_0) - a1 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a1 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 4 * L ** 2 * a2 * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                               T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                               T_0) - a2 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a2 ** 2) ** 2 * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                               T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                               T_0) - a2 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                                                                                   -2 * L ** 3 * a1 * sin(
                                                                                                               T_0) - 4 * L ** 3 * a2 * sin(
                                                                                                               T_0) + 6 * L * a1 * a2 ** 2 * sin(
                                                                                                               T_0) - 2 * a2 * (
                                                                                                                               L ** 6 * a1 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                                                                                                                           T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
                                                                                                                           T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - 2 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a2 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 2 * L ** 2 * (
                                                                                                                   -2 * L ** 3 * a1 * sin(
                                                                                                               T_0) + 2 * L * a1 ** 3 * sin(
                                                                                                               T_0) - 2 * a1 * (
                                                                                                                               L ** 6 * a1 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                                                                                                                           T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
                                                                                                                           T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                                                                                                                           T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                                                                                   L ** 5 * sin(
                                                                                                               T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                               T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                               T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                               T_0) - a1 * sqrt(
                                                                                                               -2 * L ** 8 * sin(
                                                                                                                   T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                   T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                   T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                   T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(
                                                                                                   T_0) ** 2 / ((
                                                                                                                            L ** 2 - a1 ** 2) * (
                                                                                                                            L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) / (
                                                                                                       L * sqrt(
                                                                                                   L ** 2 - a2 ** 2) * (
                                                                                                                   L ** 4 - a1 ** 2 * a2 ** 2) * sqrt(
                                                                                                   4 * L ** 2 * sin(
                                                                                                       T_0) ** 2 - 4 * L ** 2 * (
                                                                                                               L ** 5 * sin(
                                                                                                           T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                           T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                           T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                           T_0) - a2 * sqrt(
                                                                                                           -2 * L ** 8 * sin(
                                                                                                               T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                               T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a2 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                               L ** 5 * sin(
                                                                                                           T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                           T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                           T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                           T_0) - a1 * sqrt(
                                                                                                           -2 * L ** 8 * sin(
                                                                                                               T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                               T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                               T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                               T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a1 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * sin(
                                                                                                   T_0))) * (
                            L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                        T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                            L * sqrt(L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) * ((4 * L ** 2 * sin(
                        T_0) ** 2 - 4 * L ** 2 * (L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                        T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                            L ** 5 * sin(
                                                                                                        T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                        T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                        T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                        T_0) - a1 * sqrt(
                                                                                                        -2 * L ** 8 * sin(
                                                                                                            T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                        T_0) ** 2 / ((L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (L ** 5 * sin(
                        T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
                        T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
                        -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (L ** 2 * (
                                L ** 2 - a2 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(T_0) ** 2) + (
                                                                                                           4 * L ** 2 * sin(
                                                                                                       T_0) ** 2 - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) - L ** 3 * a2 ** 2 * sin(
                                                                                                                   T_0) + L * a1 * a2 ** 3 * sin(
                                                                                                                   T_0) - a2 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a2 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                                                       L ** 5 * sin(
                                                                                                                   T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                                   T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                                   T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                                   T_0) - a1 * sqrt(
                                                                                                                   -2 * L ** 8 * sin(
                                                                                                                       T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                                       T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                                       T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                                       T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 * sin(
                                                                                                       T_0) ** 2 / ((
                                                                                                                                L ** 2 - a1 ** 2) * (
                                                                                                                                L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) * (
                                                                                                           L ** 5 * sin(
                                                                                                       T_0) - L ** 3 * a1 ** 2 * sin(
                                                                                                       T_0) - L ** 3 * a1 * a2 * sin(
                                                                                                       T_0) + L * a1 ** 3 * a2 * sin(
                                                                                                       T_0) - a1 * sqrt(
                                                                                                       -2 * L ** 8 * sin(
                                                                                                           T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                                                           T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                                                           T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                                                           T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                                           L ** 2 * (
                                                                                                               L ** 2 - a1 ** 2) * (
                                                                                                                       L ** 4 - a1 ** 2 * a2 ** 2) ** 2 * sin(
                                                                                                       T_0) ** 2)) * sin(
                        T_0))]

    jac[4] = [(-16 * L ** 2 * a1 * a2 ** 2 * (
                L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(T_0) - L ** 3 * a2 ** 2 * sin(
            T_0) + L * a1 * a2 ** 3 * sin(T_0) - a2 * sqrt(
            -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / ((L ** 2 - a2 ** 2) * (
                L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 16 * L ** 2 * a1 * a2 ** 2 * (
                                              L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                          T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                          T_0) - a1 * sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                              (L ** 2 - a1 ** 2) * (
                                                  L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 8 * L ** 2 * a1 * (
                                              L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                          T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                          T_0) - a1 * sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                              (L ** 2 - a1 ** 2) ** 2 * (
                                                  L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                              -2 * L ** 3 * a2 * sin(T_0) + 2 * L * a2 ** 3 * sin(
                                          T_0) - 2 * a2 * (2 * L ** 6 * a1 * sin(
                                          T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                          T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                          T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                          T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                          T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                          T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                              L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                          T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                          T_0) - a2 * sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                              (L ** 2 - a2 ** 2) * (
                                                  L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                              L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                          T_0) - L ** 3 * a1 * a2 * sin(T_0) + L * a1 ** 3 * a2 * sin(
                                          T_0) - a1 * sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                              -4 * L ** 3 * a1 * sin(T_0) - 2 * L ** 3 * a2 * sin(
                                          T_0) + 6 * L * a1 ** 2 * a2 * sin(T_0) - 2 * a1 * (
                                                          2 * L ** 6 * a1 * sin(
                                                      T_0) ** 2 - L ** 6 * a1 + L ** 6 * a2 * sin(
                                                      T_0) ** 2 - 3 * L ** 4 * a1 ** 2 * a2 * sin(
                                                      T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 2 * sin(
                                                      T_0) ** 2 - L ** 4 * a2 ** 3 * sin(
                                                      T_0) ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 2 + 3 * L ** 2 * a1 ** 2 * a2 ** 3 * sin(
                                                      T_0) ** 2 + L ** 2 * a1 * a2 ** 4 - 2 * a1 ** 3 * a2 ** 4) / sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - 2 * sqrt(
                                          -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                              T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                              T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                              T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                              T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                              (L ** 2 - a1 ** 2) * (L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) / (
                                             2 * L ** 2 * sqrt(1 - (2 * L ** 2 - 4 * L ** 2 * (
                                                 L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                             T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                             T_0) - a2 * sqrt(
                                             -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                 T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                (L ** 2 - a2 ** 2) * (
                                                                                    L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                L ** 5 * sin(
                                                                            T_0) - L ** 3 * a1 ** 2 * sin(
                                                                            T_0) - L ** 3 * a1 * a2 * sin(
                                                                            T_0) + L * a1 ** 3 * a2 * sin(
                                                                            T_0) - a1 * sqrt(-2 * L ** 8 * sin(
                                                                            T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                (L ** 2 - a1 ** 2) * (
                                                                                    L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) ** 2 / (
                                                                           4 * L ** 4))),

 (
                                             -16 * L ** 2 * a1 ** 2 * a2 * (
                                                 L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                             T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                             T_0) - a2 * sqrt(
                                             -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                 T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                         (L ** 2 - a2 ** 2) * (
                                                             L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 16 * L ** 2 * a1 ** 2 * a2 * (
                                                         L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                     T_0) - L ** 3 * a1 * a2 * sin(
                                                     T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                     -2 * L ** 8 * sin(
                                                         T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                         (L ** 2 - a1 ** 2) * (
                                                             L ** 4 - a1 ** 2 * a2 ** 2) ** 3) - 8 * L ** 2 * a2 * (
                                                         L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                                     T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                                     T_0) - a2 * sqrt(-2 * L ** 8 * sin(
                                                     T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                     T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                     T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                     T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                     T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                     T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                     T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                     T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                         (L ** 2 - a2 ** 2) ** 2 * (
                                                             L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                         L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                                     T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                                     T_0) - a2 * sqrt(-2 * L ** 8 * sin(
                                                     T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                     T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                     T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                     T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                     T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                     T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                     T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                     T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                         -2 * L ** 3 * a1 * sin(T_0) - 4 * L ** 3 * a2 * sin(
                                                     T_0) + 6 * L * a1 * a2 ** 2 * sin(T_0) - 2 * a2 * (
                                                                     L ** 6 * a1 * sin(
                                                                 T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                                                                 T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
                                                                 T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                                                                 T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                                                     -2 * L ** 8 * sin(
                                                         T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4) - 2 * sqrt(
                                                     -2 * L ** 8 * sin(
                                                         T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                                         (L ** 2 - a2 ** 2) * (
                                                             L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                         -2 * L ** 3 * a1 * sin(T_0) + 2 * L * a1 ** 3 * sin(
                                                     T_0) - 2 * a1 * (L ** 6 * a1 * sin(
                                                     T_0) ** 2 + 2 * L ** 6 * a2 * sin(
                                                     T_0) ** 2 - L ** 6 * a2 - L ** 4 * a1 ** 3 * sin(
                                                     T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 * sin(
                                                     T_0) ** 2 - 3 * L ** 4 * a1 * a2 ** 2 * sin(
                                                     T_0) ** 2 + L ** 2 * a1 ** 4 * a2 + 3 * L ** 2 * a1 ** 3 * a2 ** 2 * sin(
                                                     T_0) ** 2 + 2 * L ** 2 * a1 ** 2 * a2 ** 3 - 2 * a1 ** 4 * a2 ** 3) / sqrt(
                                                     -2 * L ** 8 * sin(
                                                         T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * (
                                                         L ** 5 * sin(T_0) - L ** 3 * a1 ** 2 * sin(
                                                     T_0) - L ** 3 * a1 * a2 * sin(
                                                     T_0) + L * a1 ** 3 * a2 * sin(T_0) - a1 * sqrt(
                                                     -2 * L ** 8 * sin(
                                                         T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                         T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                         T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                         T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                         T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) / (
                                                         (L ** 2 - a1 ** 2) * (
                                                             L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) / (
                                             2 * L ** 2 * sqrt(1 - (2 * L ** 2 - 4 * L ** 2 * (
                                                 L ** 5 * sin(T_0) - L ** 3 * a1 * a2 * sin(
                                             T_0) - L ** 3 * a2 ** 2 * sin(T_0) + L * a1 * a2 ** 3 * sin(
                                             T_0) - a2 * sqrt(
                                             -2 * L ** 8 * sin(T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                 T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                 T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                 T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                 T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                (L ** 2 - a2 ** 2) * (
                                                                                    L ** 4 - a1 ** 2 * a2 ** 2) ** 2) - 4 * L ** 2 * (
                                                                                L ** 5 * sin(
                                                                            T_0) - L ** 3 * a1 ** 2 * sin(
                                                                            T_0) - L ** 3 * a1 * a2 * sin(
                                                                            T_0) + L * a1 ** 3 * a2 * sin(
                                                                            T_0) - a1 * sqrt(-2 * L ** 8 * sin(
                                                                            T_0) ** 2 + L ** 8 + 2 * L ** 6 * a1 ** 2 * sin(
                                                                            T_0) ** 2 - L ** 6 * a1 ** 2 + 2 * L ** 6 * a1 * a2 * sin(
                                                                            T_0) ** 2 + 2 * L ** 6 * a2 ** 2 * sin(
                                                                            T_0) ** 2 - L ** 6 * a2 ** 2 - 2 * L ** 4 * a1 ** 3 * a2 * sin(
                                                                            T_0) ** 2 - 2 * L ** 4 * a1 ** 2 * a2 ** 2 * sin(
                                                                            T_0) ** 2 - 2 * L ** 4 * a1 * a2 ** 3 * sin(
                                                                            T_0) ** 2 + L ** 2 * a1 ** 4 * a2 ** 2 + 2 * L ** 2 * a1 ** 3 * a2 ** 3 * sin(
                                                                            T_0) ** 2 + L ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) ** 2 / (
                                                                                (L ** 2 - a1 ** 2) * (
                                                                                    L ** 4 - a1 ** 2 * a2 ** 2) ** 2)) ** 2 / (
                                                                           4 * L ** 4)))]

    return jac




def jacobian_euler(a1, a2, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=xx7_60I1sV9j&line=1&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0, a1=a1, a2=a2)

    jac = np.zeros((6, 2), dtype=float)
    jac[0, 0] = (4 * L * a1 * t.a2_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) + 2 * L * (
                         -t.l_p3 * a2 * t.sin_t0 + L * t.a2_p3 * t.sin_t0 - a2 * (
                         2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / t.sqrt_05) * t.sin_t0 / (
                             sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq))
    jac[0, 1] = (4 * L * t.a1_p2 * a2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) + 2 * L * a2 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) * t.sin_t0 / (
                         (t.l_p2 - t.a2_p2) ** (3 / 2) * t.l_p4_minus_a1_sq_mul_a2_sq) + 2 * L * (
                         -t.l_p3 * a1 * t.sin_t0 - 2 * t.l_p3 * a2 * t.sin_t0 + 3 * L * a1 * t.a2_p2 * t.sin_t0 - a2 * (
                         t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / t.sqrt_05 - t.sqrt_05) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq))
    jac[1, 0] = (4 * L * a1 * t.a2_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_03) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) + 2 * L * a1 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_05) * t.sin_t0 / (
                         (t.l_p2 - t.a1_p2) ** (3 / 2) * t.l_p4_minus_a1_sq_mul_a2_sq) + 2 * L * (
                         -2 * t.l_p3 * a1 * t.sin_t0 - t.l_p3 * a2 * t.sin_t0 + 3 * L * t.a1_p2 * a2 * t.sin_t0 - a1 * (
                         2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / t.sqrt_05 - t.sqrt_05) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq))
    jac[1, 1] = (4 * L * t.a1_p2 * a2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_03) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) + 2 * L * (
                         -t.l_p3 * a1 * t.sin_t0 + L * t.a1_p3 * t.sin_t0 - a1 * (
                         t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / t.sqrt_05) * t.sin_t0 / (
                         sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq))

    jac[2, 0] = (4 * L * a1 * t.a2_p2 * (
            L * t.l_p4_minus_a1_sq_mul_a2_sq * (a1 + a2) * t.sin_t0 + (t.l_p2 + a1 * a2) * sqrt(
        t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                         (t.l_p2 + a1 * a2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * L * a2 * (
                         L * t.l_p4_minus_a1_sq_mul_a2_sq * (a1 + a2) * t.sin_t0 + (t.l_p2 + a1 * a2) * sqrt(
                     t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                             -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                             -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                         (t.l_p2 + a1 * a2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq) + 2 * L * (
                         -2 * L * a1 * t.a2_p2 * (a1 + a2) * t.sin_t0 + L * (
                         t.l_p4 - t.a1_p2 * t.a2_p2) * t.sin_t0 + a2 * sqrt(
                     t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                             -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                             -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2)) + (
                                 t.l_p2 + a1 * a2) * (t.l_p2 * (2 * t.l_p2 - 4 * a1 * a2 - 2 * t.a2_p2) * (
                         t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) * t.sin_t0_sq / 2 + a1 * t.a2_p2 * (
                                                              -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2) + (
                                                              -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                                                              2 * t.l_p2 * a1 * t.cos_t0_sq - 2 * a1 * t.a2_p2) / 2) / sqrt(
                     t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                             -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                             -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                         (t.l_p2 + a1 * a2) * t.l_p4_minus_a1_sq_mul_a2_sq))

    jac[2, 1] = (4 * L * t.a1_p2 * a2 * (
            L * t.l_p4_minus_a1_sq_mul_a2_sq * (a1 + a2) * t.sin_t0 + (t.l_p2 + a1 * a2) * sqrt(
        t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                         (t.l_p2 + a1 * a2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * L * a1 * (
                         L * t.l_p4_minus_a1_sq_mul_a2_sq * (a1 + a2) * t.sin_t0 + (t.l_p2 + a1 * a2) * sqrt(
                     t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                             -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                             -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                         (t.l_p2 + a1 * a2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq) + 2 * L * (
                         -2 * L * t.a1_p2 * a2 * (a1 + a2) * t.sin_t0 + L * (
                         t.l_p4 - t.a1_p2 * t.a2_p2) * t.sin_t0 + a1 * sqrt(
                     t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                             -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                             -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2)) + (
                                 t.l_p2 + a1 * a2) * (t.l_p2 * (2 * t.l_p2 - 2 * t.a1_p2 - 4 * a1 * a2) * (
                         t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) * t.sin_t0_sq / 2 + t.a1_p2 * a2 * (
                                                              -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2) + (
                                                              -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                                                              2 * t.l_p2 * a2 * t.cos_t0_sq - 2 * t.a1_p2 * a2) / 2) / sqrt(
                     t.l_p2 * (t.l_p2 * a1 + t.l_p2 * a2 - t.a1_p2 * a2 - a1 * t.a2_p2) ** 2 * t.sin_t0_sq + (
                             -t.l_p4 + t.a1_p2 * t.a2_p2) * (
                             -2 * t.l_p4 * t.cos_t0_sq + t.l_p4 + t.l_p2 * t.a1_p2 * t.cos_t0_sq + t.l_p2 * t.a2_p2 * t.cos_t0_sq - t.a1_p2 * t.a2_p2))) * t.sin_t0 / (
                         (t.l_p2 + a1 * a2) * t.l_p4_minus_a1_sq_mul_a2_sq))




    jac[3, 0] = (-(2 * t.l_p2 - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 / (
                           (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                           (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (2 * a1 * t.a2_p2 * sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.cos_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_04) / (
                                                                                               L * sqrt(
                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0) + a1 * sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.cos_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_04) / (
                                                                                               L * (
                                                                                               t.l_p2 - t.a1_p2) ** (
                                                                                                       3 / 2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) * t.sin_t0) + sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                                                                               -2 * t.l_p3 * a1 * t.sin_t0 - t.l_p3 * a2 * t.cos_t0 + 3 * L * t.a1_p2 * a2 * t.cos_t0 - a1 * (
                                                                                               2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / t.sqrt_04 - t.sqrt_04) / (
                                                                                               L * sqrt(
                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) * t.sin_t0) + (
                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.cos_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_04) * (
                                                                                               -8 * t.l_p2 * a1 * t.a2_p2 * (
                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_04) ** 2 * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a2_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 8 * t.l_p2 * a1 * t.a2_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a1_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 4 * t.l_p2 * a1 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a1_p2) ** 2 * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * t.l_p2 * (
                                                                                                       -2 * t.l_p3 * a2 * t.sin_t0 + 2 * L * t.a2_p3 * t.sin_t0 - 2 * a2 * (
                                                                                                       2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a2_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * t.l_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                                       -4 * t.l_p3 * a1 * t.sin_t0 - 2 * t.l_p3 * a2 * t.sin_t0 + 6 * L * t.a1_p2 * a2 * t.sin_t0 - 2 * a1 * (
                                                                                                       2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - 2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a1_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) / (
                                                                                               L * sqrt(
                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) * sqrt(
                                                                                           4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                                                                                                   t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                   (
                                                                                                           t.l_p2 - t.a2_p2) * (
                                                                                                           t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                   (
                                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                           t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) * t.sin_t0)) / (
                         2 * t.l_p2 * ((4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) ** 2 * t.sin_t0_sq / (
                                                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                                                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                               t.l_p2 * (t.l_p2 - t.a1_p2) * (
                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0_sq) + (
                                               2 * t.l_p2 - 4 * t.l_p2 * (
                                               t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                       (t.l_p2 - t.a2_p2) * (
                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                       (t.l_p2 - t.a1_p2) * (t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) ** 2 / (
                                               4 * t.l_p4))) + sqrt(4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 * t.sin_t0_sq / (
                                                                            (t.l_p2 - t.a2_p2) * (
                                                                            t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                        -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                            (t.l_p2 - t.a1_p2) * (
                                                                            t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_05) * (
                         -16 * t.l_p2 * a1 * t.a2_p2 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) ** 2 / (
                                 (t.l_p2 - t.a2_p2) * (
                                 t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 16 * t.l_p2 * a1 * t.a2_p2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_01) ** 2 / (
                                 (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 3) - 8 * t.l_p2 * a1 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_01) ** 2 / (
                                 (t.l_p2 - t.a1_p2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                                 -2 * t.l_p3 * a2 * t.sin_t0 + 2 * L * t.a2_p3 * t.sin_t0 - 2 * a2 * (
                                 2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / t.sqrt_01) * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_01) / (
                                 (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_01) * (
                                 -4 * t.l_p3 * a1 * t.sin_t0 - 2 * t.l_p3 * a2 * t.sin_t0 + 6 * L * t.a1_p2 * a2 * t.sin_t0 - 2 * a1 * (
                                 2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / t.sqrt_01 - 2 * t.sqrt_01) / (
                                 (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) / (
                         2 * t.l_p3 * sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq * ((
                                                                                                       4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a2_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a1_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                       t.l_p2 * (
                                                                                                       t.l_p2 - t.a1_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0_sq) + (
                                                                                                       2 * t.l_p2 - 4 * t.l_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a2_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a1_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) ** 2 / (
                                                                                                       4 * t.l_p4)) * t.sin_t0))





    jac[3, 1] = (-(2 * t.l_p2 - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 / (
                           (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                           (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (2 * t.a1_p2 * a2 * sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_04) / (
                                                                                               L * sqrt(
                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0) + sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                                                                               -t.l_p3 * a1 * t.sin_t0 + L * t.a1_p3 * t.sin_t0 - a1 * (
                                                                                               t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / t.sqrt_04) / (
                                                                                               L * sqrt(
                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) * t.sin_t0) + (
                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_04) * (
                                                                                               -8 * t.l_p2 * t.a1_p2 * a2 * (
                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_04) ** 2 * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a2_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 8 * t.l_p2 * t.a1_p2 * a2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a1_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 4 * t.l_p2 * a2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a2_p2) ** 2 * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * t.l_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                                       -2 * t.l_p3 * a1 * t.sin_t0 - 4 * t.l_p3 * a2 * t.sin_t0 + 6 * L * a1 * t.a2_p2 * t.sin_t0 - 2 * a2 * (
                                                                                                       t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - 2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a2_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * t.l_p2 * (
                                                                                                       -2 * t.l_p3 * a1 * t.sin_t0 + 2 * L * t.a1_p3 * t.sin_t0 - 2 * a1 * (
                                                                                                       t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                                                                                       (
                                                                                                               t.l_p2 - t.a1_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) / (
                                                                                               L * sqrt(
                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) * sqrt(
                                                                                           4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                                                                                                   t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                   (
                                                                                                           t.l_p2 - t.a2_p2) * (
                                                                                                           t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                   (
                                                                                                           t.l_p2 - t.a1_p2) * (
                                                                                                           t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) * t.sin_t0)) / (
                         2 * t.l_p2 * ((4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) ** 2 * t.sin_t0_sq / (
                                                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                                                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.cos_t0 - a1 * sqrt(
                                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.cos_t0 - a1 * sqrt(
                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                               t.l_p2 * (t.l_p2 - t.a1_p2) * (
                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0_sq) + (
                                               2 * t.l_p2 - 4 * t.l_p2 * (
                                               t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                       (t.l_p2 - t.a2_p2) * (
                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                       (t.l_p2 - t.a1_p2) * (t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) ** 2 / (
                                               4 * t.l_p4))) + sqrt(4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 * t.sin_t0_sq / (
                                                                            (t.l_p2 - t.a2_p2) * (
                                                                            t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                        -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                            (t.l_p2 - t.a1_p2) * (
                                                                            t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_05) * (
                         -16 * t.l_p2 * t.a1_p2 * a2 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) ** 2 / (
                                 (t.l_p2 - t.a2_p2) * (
                                 t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 16 * t.l_p2 * t.a1_p2 * a2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_01) ** 2 / (
                                 (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 3) - 8 * t.l_p2 * a2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_01) ** 2 / (
                                 (t.l_p2 - t.a2_p2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_01) * (
                                 -2 * t.l_p3 * a1 * t.sin_t0 - 4 * t.l_p3 * a2 * t.sin_t0 + 6 * L * a1 * t.a2_p2 * t.sin_t0 - 2 * a2 * (
                                 t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / t.sqrt_01 - 2 * t.sqrt_01) / (
                                 (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                                 -2 * t.l_p3 * a1 * t.sin_t0 + 2 * L * t.a1_p3 * t.sin_t0 - 2 * a1 * (
                                 t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / t.sqrt_01) * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_01) / (
                                 (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) / (
                         2 * t.l_p3 * sqrt(t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq * ((
                                                                                                       4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a2_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a1_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                       t.l_p2 * (
                                                                                                       t.l_p2 - t.a1_p2) * (
                                                                                                               t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0_sq) + (
                                                                                                       2 * t.l_p2 - 4 * t.l_p2 * (
                                                                                                       t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                   -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a2_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                                                                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                               (
                                                                                                                       t.l_p2 - t.a1_p2) * (
                                                                                                                       t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) ** 2 / (
                                                                                                       4 * t.l_p4)) * t.sin_t0)

                 )
    jac[4, 0] = (-(2 * a1 * t.a2_p2 * sqrt(4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 * t.sin_t0_sq / (
                                                   (t.l_p2 - t.a2_p2) * (
                                                   t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                   (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                           L * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2 * t.sin_t0) + sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                           -t.l_p3 * a2 * t.sin_t0 + L * t.a2_p3 * t.sin_t0 - a2 * (
                           2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                           L * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq * t.sin_t0) + (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                           -8 * t.l_p2 * a1 * t.a2_p2 * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                   (t.l_p2 - t.a2_p2) * (
                                   t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 8 * t.l_p2 * a1 * t.a2_p2 * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                   (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 3) - 4 * t.l_p2 * a1 * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                   (t.l_p2 - t.a1_p2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * t.l_p2 * (
                                   -2 * t.l_p3 * a2 * t.sin_t0 + 2 * L * t.a2_p3 * t.sin_t0 - 2 * a2 * (
                                   2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                   (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * t.l_p2 * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                   -4 * t.l_p3 * a1 * t.sin_t0 - 2 * t.l_p3 * a2 * t.sin_t0 + 6 * L * t.a1_p2 * a2 * t.sin_t0 - 2 * a1 * (
                                   2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - 2 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                   (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) / (
                           L * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq * sqrt(
                       4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                               t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                               (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                               (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * t.sin_t0)) / sqrt(1 - (
            4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 * t.sin_t0_sq / (
                    (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                    t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.cos_t0 - a1 * sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                    (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                                                                                                    t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                                    t.l_p2 * (
                                                                                                                    t.l_p2 - t.a2_p2) * (
                                                                                                                            t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0_sq)))
    jac[4, 1] = (-(2 * t.a1_p2 * a2 * sqrt(4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 * t.sin_t0_sq / (
                                                   (t.l_p2 - t.a2_p2) * (
                                                   t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 4 * t.l_p2 * (
                                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                                   (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                           L * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2 * t.sin_t0) + a2 * sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.cos_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                           L * (t.l_p2 - t.a2_p2) ** (3 / 2) * t.l_p4_minus_a1_sq_mul_a2_sq * t.sin_t0) + sqrt(
        4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_02) ** 2 * t.sin_t0_sq / (
                (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                           -t.l_p3 * a1 * t.sin_t0 - 2 * t.l_p3 * a2 * t.sin_t0 + 3 * L * a1 * t.a2_p2 * t.sin_t0 - a2 * (
                           t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                           L * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq * t.sin_t0) + (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                           -8 * t.l_p2 * t.a1_p2 * a2 * (
                           t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                       -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                   (t.l_p2 - t.a2_p2) * (
                                   t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 8 * t.l_p2 * t.a1_p2 * a2 * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                   (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 3) - 4 * t.l_p2 * a2 * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                                   (t.l_p2 - t.a2_p2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * t.l_p2 * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                   -2 * t.l_p3 * a1 * t.sin_t0 - 4 * t.l_p3 * a2 * t.sin_t0 + 6 * L * a1 * t.a2_p2 * t.sin_t0 - 2 * a2 * (
                                   t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - 2 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                   (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * t.l_p2 * (
                                   -2 * t.l_p3 * a1 * t.sin_t0 + 2 * L * t.a1_p3 * t.sin_t0 - 2 * a1 * (
                                   t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                   t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                               -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * t.sin_t0_sq / (
                                   (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) / (
                           L * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq * sqrt(
                       4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                               t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                               (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                               t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                           -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                               (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * t.sin_t0)) / sqrt(1 - (
            4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 * t.sin_t0_sq / (
                    (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 4 * t.l_p2 * (
                    t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.cos_t0 - a1 * sqrt(
                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * t.sin_t0_sq / (
                    (t.l_p2 - t.a1_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                                                                                                                    t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                                                -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                                                                                                    t.l_p2 * (
                                                                                                                    t.l_p2 - t.a2_p2) * (
                                                                                                                            t.l_p4 - t.a1_p2 * t.a2_p2) ** 2 * t.sin_t0_sq))

                 )
    jac[5, 0] = ((1 - 2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 / (
                          (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (-8 * a1 * t.a2_p2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_03) * (
                                                                                              t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                              sqrt(
                                                                                                  t.l_p2 - t.a1_p2) * sqrt(
                                                                                          t.l_p2 - t.a2_p2) * (
                                                                                                      t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 2 * a1 * (
                                                                                              t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                              t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                              (t.l_p2 - t.a1_p2) ** (
                                                                                              3 / 2) * sqrt(
                                                                                          t.l_p2 - t.a2_p2) * (
                                                                                                      t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * (
                                                                                              -t.l_p3 * a2 * t.sin_t0 + L * t.a2_p3 * t.sin_t0 - a2 * (
                                                                                              2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                              t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                              sqrt(
                                                                                                  t.l_p2 - t.a1_p2) * sqrt(
                                                                                          t.l_p2 - t.a2_p2) * (
                                                                                                      t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * (
                                                                                              t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                              -2 * t.l_p3 * a1 * t.sin_t0 - t.l_p3 * a2 * t.sin_t0 + 3 * L * t.a1_p2 * a2 * t.sin_t0 - a1 * (
                                                                                              2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - sqrt(
                                                                                          -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                              sqrt(
                                                                                                  t.l_p2 - t.a1_p2) * sqrt(
                                                                                          t.l_p2 - t.a2_p2) * (
                                                                                                      t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) / (
                         (1 - 2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_01) ** 2 / (
                                  (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) ** 2 + 4 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_01) ** 2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_01) ** 2 / (
                                 (t.l_p2 - t.a1_p2) * (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 4)) + 2 * (
                         -8 * a1 * t.a2_p2 * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) ** 2 / (
                                 (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 3) - 2 * (
                                 -2 * t.l_p3 * a2 * t.sin_t0 + 2 * L * t.a2_p3 * t.sin_t0 - 2 * a2 * (
                                 2 * t.l_p6 * a1 * t.sin_t0_sq - t.l_p6 * a1 + t.l_p6 * a2 * t.sin_t0_sq - 3 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq - t.l_p4 * t.a2_p3 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p3 * t.a2_p2 + 3 * t.l_p2 * t.a1_p2 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * a1 * t.a2_p4 - 2 * t.a1_p3 * t.a2_p4) / t.sqrt_01) * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_01) / (
                                 (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_05) * (
                         t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_05) / (
                         sqrt(t.l_p2 - t.a1_p2) * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2 * (
                         (1 - 2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                             -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                  (
                                          t.l_p2 - t.a2_p2) * (
                                          t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) ** 2 + 4 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                             -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * (
                                 t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                             -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                 (
                                         t.l_p2 - t.a1_p2) * (
                                         t.l_p2 - t.a2_p2) * (
                                         t.l_p4 - t.a1_p2 * t.a2_p2) ** 4))))

    jac[5, 1] = (1 - 2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * t.sqrt_03) ** 2 / (
                         (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (-8 * t.a1_p2 * a2 * (
            t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * t.sqrt_03) * (
                                                                                             t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                             sqrt(
                                                                                                 t.l_p2 - t.a1_p2) * sqrt(
                                                                                         t.l_p2 - t.a2_p2) * (
                                                                                                     t.l_p4 - t.a1_p2 * t.a2_p2) ** 3) - 2 * a2 * (
                                                                                             t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                             t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                             sqrt(t.l_p2 - t.a1_p2) * (
                                                                                             t.l_p2 - t.a2_p2) ** (
                                                                                                     3 / 2) * (
                                                                                                     t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * (
                                                                                             -t.l_p3 * a1 * t.sin_t0 + L * t.a1_p3 * t.sin_t0 - a1 * (
                                                                                             t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                             t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                             sqrt(
                                                                                                 t.l_p2 - t.a1_p2) * sqrt(
                                                                                         t.l_p2 - t.a2_p2) * (
                                                                                                     t.l_p4 - t.a1_p2 * t.a2_p2) ** 2) - 2 * (
                                                                                             t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                                                                             -t.l_p3 * a1 * t.sin_t0 - 2 * t.l_p3 * a2 * t.sin_t0 + 3 * L * a1 * t.a2_p2 * t.sin_t0 - a2 * (
                                                                                             t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - sqrt(
                                                                                         -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                                                                             sqrt(
                                                                                                 t.l_p2 - t.a1_p2) * sqrt(
                                                                                         t.l_p2 - t.a2_p2) * (
                                                                                                     t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) / (
                        (1 - 2 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                 (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) ** 2 + 4 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                (t.l_p2 - t.a1_p2) * (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 4)) + 2 * (
                        -8 * t.a1_p2 * a2 * (
                        t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                    -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 3) - 4 * a2 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                (t.l_p2 - t.a2_p2) ** 2 * t.l_p4_minus_a1_sq_mul_a2_sq ** 2) - 2 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                                -2 * t.l_p3 * a1 * t.sin_t0 - 4 * t.l_p3 * a2 * t.cos_t0 + 6 * L * a1 * t.a2_p2 * t.sin_t0 - 2 * a2 * (
                                t.l_p6 * a1 * t.sin_t0_sq + 2 * t.l_p6 * a2 * t.sin_t0_sq - t.l_p6 * a2 - t.l_p4 * t.a1_p3 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * a2 * t.sin_t0_sq - 3 * t.l_p4 * a1 * t.a2_p2 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * a2 + 3 * t.l_p2 * t.a1_p3 * t.a2_p2 * t.sin_t0_sq + 2 * t.l_p2 * t.a1_p2 * t.a2_p3 - 2 * t.a1_p4 * t.a2_p3) / sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4) - 2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                                (t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2)) * (
                        t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.cos_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                    -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) * (
                        t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.cos_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                    -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) / (
                        sqrt(t.l_p2 - t.a1_p2) * sqrt(t.l_p2 - t.a2_p2) * t.l_p4_minus_a1_sq_mul_a2_sq ** 2 * (
                        (1 - 2 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                 (
                                         t.l_p2 - t.a2_p2) * (
                                         t.l_p4 - t.a1_p2 * t.a2_p2) ** 2)) ** 2 + 4 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * t.a1_p2 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 + L * t.a1_p3 * a2 * t.sin_t0 - a1 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 * (
                                t.l_p5 * t.sin_t0 - t.l_p3 * a1 * a2 * t.sin_t0 - t.l_p3 * t.a2_p2 * t.sin_t0 + L * a1 * t.a2_p3 * t.sin_t0 - a2 * sqrt(
                            -2 * t.l_p8 * t.sin_t0_sq + t.l_p8 + 2 * t.l_p6 * t.a1_p2 * t.sin_t0_sq - t.l_p6 * t.a1_p2 + 2 * t.l_p6 * a1 * a2 * t.sin_t0_sq + 2 * t.l_p6 * t.a2_p2 * t.sin_t0_sq - t.l_p6 * t.a2_p2 - 2 * t.l_p4 * t.a1_p3 * a2 * t.sin_t0_sq - 2 * t.l_p4 * t.a1_p2 * t.a2_p2 * t.sin_t0_sq - 2 * t.l_p4 * a1 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p4 * t.a2_p2 + 2 * t.l_p2 * t.a1_p3 * t.a2_p3 * t.sin_t0_sq + t.l_p2 * t.a1_p2 * t.a2_p4 - t.a1_p4 * t.a2_p4)) ** 2 / (
                                (
                                        t.l_p2 - t.a1_p2) * (
                                        t.l_p2 - t.a2_p2) * (
                                        t.l_p4 - t.a1_p2 * t.a2_p2) ** 4)))
    return np.array(jac)



def f_zenith(a1, a2, L, T_0):
    """
    From: https://colab.research.google.com/drive/1Y7hfS1aD7iahi77h3xDdz3hMdDL553Dm#scrollTo=MXNYwnZwK2Xh&line=1&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0, a1=a1, a2=a2)

    return -arcsin((2 * t.l_p2 - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p5 * t.sin_t0 - L * sqrt(
        2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a2_div_l) ** 2 / (
                            (-t.l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p2) * (
                            -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) ** 2) - 4 * t.l_p2 * (
                            t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + t.l_p5 * t.cos_t0 - L * sqrt(
                        2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a1_div_l) ** 2 / (
                            (-t.l_p2 * t.sin_t0_plus_asin_a1_div_l_p2 + t.l_p2) * (
                            -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) ** 2)) / (
                           2 * t.l_p2)) + pi / 2


def f_azimuth(a1, a2, L, T_0):
    """
    From: https://colab.research.google.com/drive/1Y7hfS1aD7iahi77h3xDdz3hMdDL553Dm#scrollTo=MXNYwnZwK2Xh&line=1&uniqifier=1
    """
    t = CommonTerms(L=L, T_0=T_0, a1=a1, a2=a2)

    return arctan2(sqrt(4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
            t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p5 * t.cos_t0 - L * sqrt(
        2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a2_div_l) ** 2 * t.sin_t0_sq / (
                                (-t.l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p2) * (
                                -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) ** 2) - 4 * t.l_p2 * (
                                t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + t.l_p5 * t.cos_t0 - L * sqrt(
                            2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a1_div_l) ** 2 * t.sin_t0_sq / (
                                (-t.l_p2 * t.sin_t0_plus_asin_a1_div_l_p2 + t.l_p2) * (
                                -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) ** 2)) * (
                           t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.cos_t0 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p5 * t.sin_t0 - L * sqrt(
                       2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a2_div_l) / (
                           L * sqrt(-t.l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p2) * (
                           -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) * t.sin_t0),
                   sqrt(4 * t.l_p2 * t.sin_t0_sq - 4 * t.l_p2 * (
                           t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p5 * t.cos_t0 - L * sqrt(
                       2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a2_div_l) ** 2 * t.sin_t0_sq / (
                                (-t.l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p2) * (
                                -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) ** 2) - 4 * t.l_p2 * (
                                t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + t.l_p5 * t.sin_t0 - L * sqrt(
                            2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a1_div_l) ** 2 * t.sin_t0_sq / (
                                (-t.l_p2 * t.sin_t0_plus_asin_a1_div_l_p2 + t.l_p2) * (
                                -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) ** 2)) * (
                           t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p5 * t.sin_t0 * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + t.l_p5 * t.cos_t0 - L * sqrt(
                       2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l ** 3 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l ** 3 * t.sin_t0_plus_asin_a2_div_l - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l ** 3 + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a1_div_l * t.sin_t0_plus_asin_a2_div_l + 2 * t.l_p8 * t.sin_t0_sq * t.sin_t0_plus_asin_a2_div_l_p2 - 2 * t.l_p8 * t.sin_t0_sq - t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l ** 4 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l ** 4 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l ** 4 - t.l_p8 * t.sin_t0_plus_asin_a1_div_l_p2 - t.l_p8 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p8) * t.sin_t0_plus_asin_a1_div_l) / (
                           L * sqrt(-t.l_p2 * t.sin_t0_plus_asin_a1_div_l_p2 + t.l_p2) * (
                           -t.l_p4 * t.sin_t0_plus_asin_a1_div_l_p2 * t.sin_t0_plus_asin_a2_div_l_p2 + t.l_p4) * t.sin_t0))


# INVERSE KINEMATICS


def ik(ex, ey, L, T_0):
    """
    From:
    https://colab.research.google.com/drive/1a_asI3TbcC0TyEOPawE9-bi-Tq2IHzxT#scrollTo=txCaxKIJsXTz&line=1&uniqifier=1

    PE_x*(-PE_x/2 + x)/2 + PE_y*(-PE_y/2 + y)/2 + (z - sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2)/2)*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2)/2
    xline (x, -2*L**2*sin(T_0)**2 + PE_y*y + z*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2))

    P1_y_s
    (2*L**2*sin(T_0)**2 - (2*L**2*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) - L*PE_y*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2))*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2))/PE_y

    P3_y_s
    (2*L**2*sin(T_0)**2 - (2*L**2*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) + L*PE_y*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2))*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2))/PE_y

    a1_s
    2*L**2*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) - L*PE_y*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2)

    P3_z_s
    2*L**2*sqrt(4*L**2*sin(T_0)**2 - PE_x**2 - PE_y**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) + L*PE_y*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2)

    P2_x_s
    (2*L**2*sin(T_0)**2 - sqrt(4*L**2*sin(T_0)**2 - 2*PE_x**2)*(2*L**2*sqrt(4*L**2*sin(T_0)**2 - 2*PE_x**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) - L*PE_x*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2)))/PE_x

    a2_s
    2*L**2*sqrt(4*L**2*sin(T_0)**2 - 2*PE_x**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) - L*PE_x*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2)

    P4_x_s
    (2*L**2*sin(T_0)**2 - sqrt(4*L**2*sin(T_0)**2 - 2*PE_x**2)*(2*L**2*sqrt(4*L**2*sin(T_0)**2 - 2*PE_x**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) + L*PE_x*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2)))/PE_x

    P4_z_s
    2*L**2*sqrt(4*L**2*sin(T_0)**2 - 2*PE_x**2)*sin(T_0)**2/(4*L**2*sin(T_0)**2 - PE_x**2) + L*PE_x*sqrt(-4*L**2*sin(T_0)**4 + 4*L**2*sin(T_0)**2 - PE_x**2)/(4*L**2*sin(T_0)**2 - PE_x**2)
    2*t.l_p2*sqrt(4*t.l_p2*t.sin_t0_sq - t.ex_p2 - t.ey_p2)*t.sin_t0_sq/(4*t.l_p2*t.sin_t0_sq - t.ex_p2) - L*ey*sqrt(-4*t.l_p2*t.sin_t0**4 + 4*t.l_p2*t.sin_t0_sq - t.ex_p2)/(4*t.l_p2*t.sin_t0_sq - t.ex_p2)
    """

    t = CommonTerms(L=L, T_0=T_0, ex=ex, ey=ey)

    # a1_s
    a1 = 2 * t.l_p2 * sqrt(4 * t.l_p2 * t.sin_t0_sq - t.ex_p2 - t.ey_p2) * t.sin_t0_sq / (
            4 * t.l_p2 * t.sin_t0_sq - t.ex_p2) - L * ey * sqrt(
        -4 * t.l_p2 * t.sin_t0 ** 4 + 4 * t.l_p2 * t.sin_t0_sq - t.ex_p2) / (4 * t.l_p2 * t.sin_t0_sq - t.ex_p2)

    # a2_s
    a2 = 2 * t.l_p2 * sqrt(4 * t.l_p2 * t.sin_t0_sq - 2 * t.ex_p2) * t.sin_t0_sq / (
            4 * t.l_p2 * t.sin_t0_sq - t.ex_p2) - L * ex * sqrt(
        -4 * t.l_p2 * t.sin_t0 ** 4 + 4 * t.l_p2 * t.sin_t0_sq - t.ex_p2) / (4 * t.l_p2 * t.sin_t0_sq - t.ex_p2)

    return np.array([a1, a2])
