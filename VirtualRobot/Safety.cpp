#include "Safety.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


#include "MathTools.h"
#include "VirtualRobot.h"

namespace VirtualRobot
{

    Circle projectedBoundingCircle(const Robot& robot)
    {
        // MathTools::createConvexHull2D(const std::vector<Eigen::Vector2f> &points)
        std::vector<Eigen::Vector2f> nodePositions;

        const auto nodes = robot.getRobotNodes();

        std::transform(nodes.begin(),
                       nodes.end(),
                       std::back_inserter(nodePositions),
                       [](const RobotNodePtr& node)
                       { return node->getPoseInRootFrame(); });

        const MathTools::ConvexHull2D hull =
            MathTools::createConvexHull2D(nodePositions);

        std::vector<float> distances;

        std::transform(hull.vertices.begin(),
                       hull.vertices.end(),
                       std::back_inserter(distances),
                       [](const Eigen::Vector2f& pos) { return pos.norm(); });

        const float maxDistance = *std::max(distances.begin(), distances.end());
        return Circle{.center = Eigen::Vector2f::Zero(), .radius = maxDistance};
    }
} // namespace VirtualRobot
