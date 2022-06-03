import os

import sympy as sp

from hemisphere_joint_demo.sympy_to_code import SympyToCpp


if __name__ == '__main__':
    from sympy import sin, cos, sqrt

    # Actuation
    a1, a2 = sp.symbols('a1 a2')

    # Constants defining geometry.
    lever, theta0 = sp.symbols('lever theta0')


    """
    # EEF position, for 90°
    From https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=za3fZw9Rq5d9&line=1&uniqifier=1
    """
    ex = 2 * lever * (lever ** 5 * sin(theta0) - lever ** 3 * a1 * a2 * sin(theta0) - lever ** 3 * a2 ** 2 * sin(theta0) + lever * a1 * a2 ** 3 * sin(theta0) - a2 * sqrt(-2 * lever ** 8 * sin(theta0) ** 2 + lever ** 8 + 2 * lever ** 6 * a1 ** 2 * sin(theta0) ** 2 - lever ** 6 * a1 ** 2 + 2 * lever ** 6 * a1 * a2 * sin(theta0) ** 2 + 2 * lever ** 6 * a2 ** 2 * sin(theta0) ** 2 - lever ** 6 * a2 ** 2 - 2 * lever ** 4 * a1 ** 3 * a2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 ** 2 * a2 ** 2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 4 * a2 ** 2 + 2 * lever ** 2 * a1 ** 3 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(theta0) / (sqrt(lever ** 2 - a2 ** 2) * (lever ** 4 - a1 ** 2 * a2 ** 2))
    ey = 2 * lever * (lever ** 5 * sin(theta0) - lever ** 3 * a1 ** 2 * sin(theta0) - lever ** 3 * a1 * a2 * sin(theta0) + lever * a1 ** 3 * a2 * sin(theta0) - a1 * sqrt(-2 * lever ** 8 * sin(theta0) ** 2 + lever ** 8 + 2 * lever ** 6 * a1 ** 2 * sin(theta0) ** 2 - lever ** 6 * a1 ** 2 + 2 * lever ** 6 * a1 * a2 * sin(theta0) ** 2 + 2 * lever ** 6 * a2 ** 2 * sin(theta0) ** 2 - lever ** 6 * a2 ** 2 - 2 * lever ** 4 * a1 ** 3 * a2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 ** 2 * a2 ** 2 * sin(theta0) ** 2 - 2 * lever ** 4 * a1 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 4 * a2 ** 2 + 2 * lever ** 2 * a1 ** 3 * a2 ** 3 * sin(theta0) ** 2 + lever ** 2 * a1 ** 2 * a2 ** 4 - a1 ** 4 * a2 ** 4)) * sin(theta0) / (sqrt(lever ** 2 - a1 ** 2) * (lever ** 4 - a1 ** 2 * a2 ** 2))
    ez = 2 * lever * (lever * (lever ** 4 - a1 ** 2 * a2 ** 2) * (a1 + a2) * sin(theta0) + (lever ** 2 + a1 * a2) * sqrt(lever ** 2 * (lever ** 2 * a1 + lever ** 2 * a2 - a1 ** 2 * a2 - a1 * a2 ** 2) ** 2 * sin(theta0) ** 2 + (-lever ** 4 + a1 ** 2 * a2 ** 2) * (-2 * lever ** 4 * cos(theta0) ** 2 + lever ** 4 + lever ** 2 * a1 ** 2 * cos(theta0) ** 2 + lever ** 2 * a2 ** 2 * cos(theta0) ** 2 - a1 ** 2 * a2 ** 2))) * sin(theta0) / ((lever ** 2 + a1 * a2) * (lever ** 4 - a1 ** 2 * a2 ** 2))

    """
    EEF orientation
    From https://colab.research.google.com/drive/11faUc8pS1yWxFrnmt05VqpDsqOwEi_dg#scrollTo=aNSv-3Ftv3ps&line=2&uniqifier=1
    """
    exx = 1 - ex ** 2 / (2 * lever ** 2 * sin(theta0) ** 2)
    exy = -ex * ey / (2 * lever ** 2 * sin(theta0) ** 2)
    exz = -ex * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)

    eyx = -ex * ey / (2 * lever ** 2 * sin(theta0) ** 2)
    eyy = 1 - ey ** 2 / (2 * lever ** 2 * sin(theta0) ** 2)
    eyz = -ey * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)

    ezx = ex * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)
    ezy = ey * sqrt(4 * lever ** 2 * sin(theta0) ** 2 - ex ** 2 - ey ** 2) / (2 * lever ** 2 * sin(theta0) ** 2)
    ezz = (2 * lever ** 2 - ex ** 2 / sin(theta0) ** 2 - ey ** 2 / sin(theta0) ** 2) / (2 * lever ** 2)

    cpp = SympyToCpp(
        function_args=[a1, a2, lever, theta0],
        function_results=dict(
            ex=ex, ey=ey, ez=ez,
            exx=exx, exy=exy, exz=exz,
            eyx=eyx, eyy=eyy, eyz=eyz,
            ezx=ezx, ezy=ezy, ezz=ezz,
        )
    )
    cpp.build()

    output_dir = "/home/rkartmann/code/simox/VirtualRobot/examples/HemisphereJoint/"
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
