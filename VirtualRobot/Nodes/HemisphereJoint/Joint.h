#pragma once

#include <Eigen/Core>

#include "Expressions.h"


namespace VirtualRobot::hemisphere
{

    class Joint
    {
    public:

        using Jacobian = Eigen::Matrix<double, 6, 2>;

    public:

        Joint();
        Joint(double lever, double theta0);

        void computeFK(double a1, double a2);

        Eigen::Vector3d getEndEffectorTranslation() const;
        Eigen::Matrix3d getEndEffectorRotation() const;
        Eigen::Matrix4d getEndEffectorTransform() const;
        Jacobian getJacobian() const;

        void setConstants(double lever, double theta0);


    public:

        double lever;
        double theta0;
        double radius;
        double actuatorOffset;


        Expressions fk;

    };

}
