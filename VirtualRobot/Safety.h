
#include "VirtualRobot/Robot.h"

namespace VirtualRobot
{

    struct Circle
    {
        Eigen::Vector2f center;
        float radius;
    };

    Circle projectedBoundingCircle(const Robot& robot);
} // namespace VirtualRobot