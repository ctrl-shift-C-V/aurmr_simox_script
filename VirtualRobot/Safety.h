
#include <Eigen/Core>
#include <Eigen/Geometry>


#include "MathTools.h"
#include "VirtualRobot.h"
#include "VirtualRobot/Robot.h"

namespace VirtualRobot
{

    struct Circle
    {
        Eigen::Vector2f center;
        float radius;
    };

    inline Circle projectedBoundingCircle(const Robot& robot)
    {
        // MathTools::createConvexHull2D(const std::vector<Eigen::Vector2f> &points)
        std::vector<Eigen::Vector3f> nodePositions;

        const auto nodes = robot.getRobotNodes();

        std::transform(nodes.begin(),
                       nodes.end(),
                       std::back_inserter(nodePositions),
                       [](const RobotNodePtr& node)
                       { return node->getPositionInRootFrame(); });

        //const MathTools::ConvexHull2DPtr hull =
        //    MathTools::createConvexHull2D(nodePositions);

        std::vector<float> distances;

        std::transform(nodePositions.begin(),
                       nodePositions.end(),
                       std::back_inserter(distances),
                       [](const Eigen::Vector3f& pos) { return Eigen::Vector2f(pos.x(), pos.y()).norm(); });

        float maxDistance = -std::numeric_limits<float>::infinity();
        for (const auto distance : distances)
        {
            if (distance > maxDistance)
            {
                maxDistance = distance;
            }
        }

        //const float maxDistance = *std::max(distances.begin(), distances.end());
        return Circle{.center = Eigen::Vector2f::Zero(), .radius = maxDistance};
    }
} // namespace VirtualRobot
