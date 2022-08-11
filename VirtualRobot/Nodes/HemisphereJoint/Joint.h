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


        void setConstants(double lever, double theta0);


        void computeFkOfAngle(const Eigen::Vector2d& alpha12);

        void computeFkOfPosition(const Eigen::Vector2d& p12);
        void computeFkOfPosition(double p1, double p2);


        Eigen::Vector3d getEndEffectorTranslation() const;
        Eigen::Matrix3d getEndEffectorRotation() const;
        Eigen::Matrix4d getEndEffectorTransform() const;
        Jacobian getJacobian() const;

        Eigen::Vector2d angleToPosition(const Eigen::Vector2d& alpha) const;


    public:

        double lever = 0;
        double theta0 = 0;
        double radius = 0;

        double limitLo = 0;
        double limitHi = 0;


        Expressions fk;

    };

}
