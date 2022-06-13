import os
import os.path

from sympy import symbols, sin, cos, sqrt, asin

from hemisphere_joint_demo.sympy_to_code import SympyToCpp

# Actuation (P1_z, P2_z)
a1, a2 = symbols('a1 a2')

# Constants defining geometry.
lever, theta0 = symbols('lever theta0')


def fk_pos():
    """
    # EEF position, for 90°
    (PE_x, PE_y, PE_z)
    From https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=za3fZw9Rq5d9&line=1&uniqifier=1
    """
    ex = 2 * lever * (lever ** 5 * sin(theta0) - lever ** 3 * a1 * a2 * sin(theta0) - lever ** 3 * a2 ** 2 * sin(theta0) + lever * a1 * a2 ** 3 * sin(theta0) - a2 * sqrt(-2 * lever ** 8 * sin(theta0) ** 2 + lever ** 8 + 2 * lever ** 6 * a1 ** 2 * sin(theta0) ** 2 - lever ** 6 * a1 ** 2 + 2 * lever ** 6 * a1 * a2 * sin(theta0) ** 2 + 2 * lever ** 6 * a2 ** 2 * sin(theta0) ** 2 - lever ** 6 * a2 ** 2 - 2 * lever ** 4 * a1 ** 3 * a2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 ** 2 * a2 ** 2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 4 * a2 ** 2 + 2 * lever ** 2 * a1 ** 3 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(theta0) / (sqrt(lever ** 2 - a2 ** 2) * (lever ** 4 - a1 ** 2 * a2 ** 2))
    ey = 2 * lever * (lever ** 5 * sin(theta0) - lever ** 3 * a1 ** 2 * sin(theta0) - lever ** 3 * a1 * a2 * sin(theta0) + lever * a1 ** 3 * a2 * sin(theta0) - a1 * sqrt(-2 * lever ** 8 * sin(theta0) ** 2 + lever ** 8 + 2 * lever ** 6 * a1 ** 2 * sin(theta0) ** 2 - lever ** 6 * a1 ** 2 + 2 * lever ** 6 * a1 * a2 * sin(theta0) ** 2 + 2 * lever ** 6 * a2 ** 2 * sin(theta0) ** 2 - lever ** 6 * a2 ** 2 - 2 * lever ** 4 * a1 ** 3 * a2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 ** 2 * a2 ** 2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 4 * a2 ** 2 + 2 * lever ** 2 * a1 ** 3 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(theta0) / (sqrt(lever ** 2 - a1 ** 2) * (lever ** 4 - a1 ** 2 * a2 ** 2))
    ez = (
            2
            * lever
            * (
                lever
                * (
                    lever ** 4
                    - a1 ** 2 * a2 ** 2
                )
                * (a1 + a2)
                * sin(theta0)
                + (  # _a1_m_a2_a_lever_p_2
                    lever ** 2  # _lever_p_2
                    + a1 * a2   # _a1_m_a2
                )
                * sqrt(
                    lever ** 2
                    * (
                        lever ** 2 * a1
                        + lever ** 2 * a2
                        - a1 ** 2 * a2
                        - a1 * a2 ** 2
                    ) ** 2
                    * sin(theta0) ** 2
                    + (-lever ** 4 + a1 ** 2 * a2 ** 2)
                    * (
                        -2
                        * lever ** 4
                        * cos(theta0) ** 2
                        + lever ** 4
                        + lever ** 2
                        * a1 ** 2
                        * cos(theta0) ** 2
                        + lever ** 2
                        * a2 ** 2
                        * cos(theta0) ** 2
                        - a1 ** 2
                        * a2 ** 2
                    )
                )
            )
            * sin(theta0)  # _sin_l_theta0_r_
            / (
                (  # _1_d__l_a1_m_a2_a_lever_p_2_r_
                    lever ** 2
                    + a1 * a2
                )
                * (  # _1_d__l__s_a1_p_2_m_a2_p_2_a_lever_p_4_r_
                    lever ** 4
                    - a1 ** 2
                    * a2 ** 2
                )
            )
    )
    return dict(ex=ex, ey=ey, ez=ez,)


def fk_ori(pos):
    """
    EEF orientation
    From https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=aNSv-3Ftv3ps&line=2&uniqifier=1
    """
    ex, ey, ez = pos["ex"], pos["ey"], pos["ez"]

    exx = 1 - ex ** 2 / (2 * lever ** 2 * sin(theta0) ** 2)
    exy = -ex * ey / (2 * lever ** 2 * sin(theta0) ** 2)
    exz = -ex * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)

    eyx = -ex * ey / (2 * lever ** 2 * sin(theta0) ** 2)
    eyy = 1 - ey ** 2 / (2 * lever ** 2 * sin(theta0) ** 2)
    eyz = -ey * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)

    ezx = ex * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)
    ezy = ey * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)
    ezz = (2 * lever ** 2 - ex ** 2 / sin(theta0) ** 2 - ey ** 2 / sin(theta0) ** 2) / (2 * lever ** 2)

    return dict(
        exx=exx, exy=exy, exz=exz,
        eyx=eyx, eyy=eyy, eyz=eyz,
        ezx=ezx, ezy=ezy, ezz=ezz,
    )


def jacobian_pos():
    """
    Jacobian
    From https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=xx7_60I1sV9j&line=1&uniqifier=1
    """
    jx1 = 4*lever*a1*a2**2*(lever**5*sin(theta0) - lever**3*a1*a2*sin(theta0) - lever**3*a2**2*sin(theta0) + lever*a1*a2**3*sin(theta0) - a2*sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a2**2)*(lever**4 - a1**2*a2**2)**2) + 2*lever*(-lever**3*a2*sin(theta0) + lever*a2**3*sin(theta0) - a2*(2*lever**6*a1*sin(theta0)**2 - lever**6*a1 + lever**6*a2*sin(theta0)**2 - 3*lever**4*a1**2*a2*sin(theta0)**2 - 2*lever**4*a1*a2**2*sin(theta0)**2 - lever**4*a2**3*sin(theta0)**2 + 2*lever**2*a1**3*a2**2 + 3*lever**2*a1**2*a2**3*sin(theta0)**2 + lever**2*a1*a2**4 - 2*a1**3*a2**4)/sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a2**2)*(lever**4 - a1**2*a2**2))
    jx2 = 4*lever*a1**2*a2*(lever**5*sin(theta0) - lever**3*a1*a2*sin(theta0) - lever**3*a2**2*sin(theta0) + lever*a1*a2**3*sin(theta0) - a2*sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a2**2)*(lever**4 - a1**2*a2**2)**2) + 2*lever*a2*(lever**5*sin(theta0) - lever**3*a1*a2*sin(theta0) - lever**3*a2**2*sin(theta0) + lever*a1*a2**3*sin(theta0) - a2*sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/((lever**2 - a2**2)**(3/2)*(lever**4 - a1**2*a2**2)) + 2*lever*(-lever**3*a1*sin(theta0) - 2*lever**3*a2*sin(theta0) + 3*lever*a1*a2**2*sin(theta0) - a2*(lever**6*a1*sin(theta0)**2 + 2*lever**6*a2*sin(theta0)**2 - lever**6*a2 - lever**4*a1**3*sin(theta0)**2 - 2*lever**4*a1**2*a2*sin(theta0)**2 - 3*lever**4*a1*a2**2*sin(theta0)**2 + lever**2*a1**4*a2 + 3*lever**2*a1**3*a2**2*sin(theta0)**2 + 2*lever**2*a1**2*a2**3 - 2*a1**4*a2**3)/sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4) - sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a2**2)*(lever**4 - a1**2*a2**2))

    jy1 = 4*lever*a1*a2**2*(lever**5*sin(theta0) - lever**3*a1**2*sin(theta0) - lever**3*a1*a2*sin(theta0) + lever*a1**3*a2*sin(theta0) - a1*sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a1**2)*(lever**4 - a1**2*a2**2)**2) + 2*lever*a1*(lever**5*sin(theta0) - lever**3*a1**2*sin(theta0) - lever**3*a1*a2*sin(theta0) + lever*a1**3*a2*sin(theta0) - a1*sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/((lever**2 - a1**2)**(3/2)*(lever**4 - a1**2*a2**2)) + 2*lever*(-2*lever**3*a1*sin(theta0) - lever**3*a2*sin(theta0) + 3*lever*a1**2*a2*sin(theta0) - a1*(2*lever**6*a1*sin(theta0)**2 - lever**6*a1 + lever**6*a2*sin(theta0)**2 - 3*lever**4*a1**2*a2*sin(theta0)**2 - 2*lever**4*a1*a2**2*sin(theta0)**2 - lever**4*a2**3*sin(theta0)**2 + 2*lever**2*a1**3*a2**2 + 3*lever**2*a1**2*a2**3*sin(theta0)**2 + lever**2*a1*a2**4 - 2*a1**3*a2**4)/sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4) - sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a1**2)*(lever**4 - a1**2*a2**2))
    jy2 = 4*lever*a1**2*a2*(lever**5*sin(theta0) - lever**3*a1**2*sin(theta0) - lever**3*a1*a2*sin(theta0) + lever*a1**3*a2*sin(theta0) - a1*sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a1**2)*(lever**4 - a1**2*a2**2)**2) + 2*lever*(-lever**3*a1*sin(theta0) + lever*a1**3*sin(theta0) - a1*(lever**6*a1*sin(theta0)**2 + 2*lever**6*a2*sin(theta0)**2 - lever**6*a2 - lever**4*a1**3*sin(theta0)**2 - 2*lever**4*a1**2*a2*sin(theta0)**2 - 3*lever**4*a1*a2**2*sin(theta0)**2 + lever**2*a1**4*a2 + 3*lever**2*a1**3*a2**2*sin(theta0)**2 + 2*lever**2*a1**2*a2**3 - 2*a1**4*a2**3)/sqrt(-2*lever**8*sin(theta0)**2 + lever**8 + 2*lever**6*a1**2*sin(theta0)**2 - lever**6*a1**2 + 2*lever**6*a1*a2*sin(theta0)**2 + 2*lever**6*a2**2*sin(theta0)**2 - lever**6*a2**2 - 2*lever**4*a1**3*a2*sin(theta0)**2 - 2*lever**4*a1**2*a2**2*sin(theta0)**2 - 2*lever**4*a1*a2**3*sin(theta0)**2 + lever**2*a1**4*a2**2 + 2*lever**2*a1**3*a2**3*sin(theta0)**2 + lever**2*a1**2*a2**4 - a1**4*a2**4))*sin(theta0)/(sqrt(lever**2 - a1**2)*(lever**4 - a1**2*a2**2))

    jz1 = 4*lever*a1*a2**2*(lever*(lever**4 - a1**2*a2**2)*(a1 + a2)*sin(theta0) + (lever**2 + a1*a2)*sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)))*sin(theta0)/((lever**2 + a1*a2)*(lever**4 - a1**2*a2**2)**2) - 2*lever*a2*(lever*(lever**4 - a1**2*a2**2)*(a1 + a2)*sin(theta0) + (lever**2 + a1*a2)*sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)))*sin(theta0)/((lever**2 + a1*a2)**2*(lever**4 - a1**2*a2**2)) + 2*lever*(-2*lever*a1*a2**2*(a1 + a2)*sin(theta0) + lever*(lever**4 - a1**2*a2**2)*sin(theta0) + a2*sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)) + (lever**2 + a1*a2)*(lever**2*(2*lever**2 - 4*a1*a2 - 2*a2**2)*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)*sin(theta0)**2/2 + a1*a2**2*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2) + (-lever**4 + a1**2*a2**2)*(2*lever**2*a1*cos(theta0)**2 - 2*a1*a2**2)/2)/sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)))*sin(theta0)/((lever**2 + a1*a2)*(lever**4 - a1**2*a2**2))
    jz2 = 4*lever*a1**2*a2*(lever*(lever**4 - a1**2*a2**2)*(a1 + a2)*sin(theta0) + (lever**2 + a1*a2)*sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)))*sin(theta0)/((lever**2 + a1*a2)*(lever**4 - a1**2*a2**2)**2) - 2*lever*a1*(lever*(lever**4 - a1**2*a2**2)*(a1 + a2)*sin(theta0) + (lever**2 + a1*a2)*sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)))*sin(theta0)/((lever**2 + a1*a2)**2*(lever**4 - a1**2*a2**2)) + 2*lever*(-2*lever*a1**2*a2*(a1 + a2)*sin(theta0) + lever*(lever**4 - a1**2*a2**2)*sin(theta0) + a1*sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)) + (lever**2 + a1*a2)*(lever**2*(2*lever**2 - 2*a1**2 - 4*a1*a2)*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)*sin(theta0)**2/2 + a1**2*a2*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2) + (-lever**4 + a1**2*a2**2)*(2*lever**2*a2*cos(theta0)**2 - 2*a1**2*a2)/2)/sqrt(lever**2*(lever**2*a1 + lever**2*a2 - a1**2*a2 - a1*a2**2)**2*sin(theta0)**2 + (-lever**4 + a1**2*a2**2)*(-2*lever**4*cos(theta0)**2 + lever**4 + lever**2*a1**2*cos(theta0)**2 + lever**2*a2**2*cos(theta0)**2 - a1**2*a2**2)))*sin(theta0)/((lever**2 + a1*a2)*(lever**4 - a1**2*a2**2))

    return dict(
        jx1=jx1, jx2=jx2,
        jy1=jy1, jy2=jy2,
        jz1=jz1, jz2=jz2,
    )


def jacobian_ori():
    """
    From https://colab.research.google.com/drive/16M2i_O4wQAMEVkaWrUnTU6smszJXCJpj
    """

    jrx1 = lever ** 2
    jrx2 = lever ** 2

    jry1 = lever ** 2
    jry2 = lever ** 2

    jrz1 = lever ** 2
    jrz2 = lever ** 2

    return dict(
        jrx1=jrx1, jrx2=jrx2,
        jry1=jry1, jry2=jry2,
        jrz1=jrz1, jrz2=jrz2,
    )


def radius():
    return dict(radius=2 * sin(theta0) * lever)


def actuator_offset():
    return dict(actuator_offset=asin(theta0))


test_mode = False

if not test_mode and __name__ == '__main__':

    output_dir = os.path.expandvars("$simox__PATH/VirtualRobot/Nodes/HemisphereJoint/")
    assert os.path.isdir(output_dir)
    name = "Expressions"

    pos = fk_pos()
    ori = fk_ori(pos)
    jac_pos = jacobian_pos()
    jac_ori = jacobian_ori()

    cpp = SympyToCpp(
        name=name,
        function_args=[a1, a2, lever, theta0],
        function_results=dict(
            # ez=pos["ez"],
            **pos,
            **ori,
            **jac_pos,
            **jac_ori,
            # No need to re-evaluate these each time.
            # **radius(),
            # **actuator_offset(),
        )
    )
    cpp.build()

    header_path = os.path.join(output_dir, cpp.name + ".h")
    source_path = os.path.join(output_dir, cpp.name + ".cpp")

    header_lines = cpp.make_header_lines()
    source_lines = cpp.make_source_lines()

    print("Declaration:")
    print(cpp.format_lines(header_lines, line_numbers=True))
    print("Implementation:")
    print(cpp.format_lines(source_lines, line_numbers=True))

    print("Writing files...")
    print(f"- {header_path}")
    print(cpp.write_lines(header_lines, header_path))
    print(f"- {source_path}")
    print(cpp.write_lines(source_lines, source_path))

    print("Done.")


if test_mode and __name__ == '__main__':
    import numpy as np
    from numpy import sin, cos, sqrt

    lever = 1
    theta0 = np.deg2rad(25)
    actuator_offset = np.arcsin(theta0)

    a = np.array([0., 0.])
    a = a + actuator_offset
    a1, a2 = a

    pos = fk_pos()
    print(pos)

