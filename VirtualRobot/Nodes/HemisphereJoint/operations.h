#pragma once

#include <Eigen/Core>

#include "Expressions.h"


namespace VirtualRobot::hemisphere
{

    Eigen::Vector3d getEndEffectorPosition(const Expressions& expr);


    Eigen::Matrix3d getEndEffectorOrientation(const Expressions& expr);


    Eigen::Matrix<double, 6, 2> getJacobian(const Expressions& expr);

}
