#pragma once

#include <Eigen/Core>

#include "Expressions.h"


namespace VirtualRobot::hemisphere
{

    Eigen::Vector3d getEndEffectorTranslation(const Expressions& expr);


    Eigen::Matrix3d getEndEffectorRotation(const Expressions& expr);


    Eigen::Matrix<double, 6, 2> getJacobian(const Expressions& expr);

}
