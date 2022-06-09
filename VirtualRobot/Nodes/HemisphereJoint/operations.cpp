#include "operations.h"


namespace VirtualRobot
{

    Eigen::Vector3d hemisphere::getEndEffectorTranslation(const Expressions& expr)
    {
        return Eigen::Vector3d {
            expr.ex,
            expr.ey,
            expr.ez
        };
    }


    Eigen::Matrix3d hemisphere::getEndEffectorRotation(const Expressions& expr)
    {
        // r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
        Eigen::Matrix3d ori;
        ori << expr.exx, expr.eyx, expr.ezx,
                expr.exy, expr.eyy, expr.ezy,
                expr.exz, expr.eyz, expr.ezz;
        return ori;
    }


    Eigen::Matrix<double, 6, 2> hemisphere::getJacobian(const Expressions& expr)
    {
        Eigen::Matrix<double, 6, 2> jacobian;
        jacobian << expr.jx1, expr.jx2,
                expr.jy1, expr.jy2,
                expr.jz1, expr.jz2,
                expr.jrx1, expr.jrx2,
                expr.jry1, expr.jry2,
                expr.jrz1, expr.jrz2;
        return jacobian;
    }

}

