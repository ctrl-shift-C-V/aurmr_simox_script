/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2022 Rainer Kartmann
*/

#include "RobotNodeHemisphereFactory.h"
#include "RobotNode.h"
#include "RobotNodeHemisphere.h"
#include "../CollisionDetection/CollisionModel.h"


namespace VirtualRobot
{

    RobotNodeHemisphereFactory::RobotNodeHemisphereFactory()
    = default;


    RobotNodeHemisphereFactory::~RobotNodeHemisphereFactory()
    = default;


    RobotNodePtr RobotNodeHemisphereFactory::createRobotNode(
            RobotPtr robot,
            const std::string& nodeName,
            VisualizationNodePtr visualizationModel,
            CollisionModelPtr collisionModel,
            float limitLow,
            float limitHigh,
            float jointValueOffset,
            const Eigen::Matrix4f& preJointTransform,
            const Eigen::Vector3f& axis,
            const Eigen::Vector3f& /*translationDirection*/,
            const SceneObject::Physics& physics,
            RobotNode::RobotNodeType rntype) const
    {
        std::cout << "CREATE NEW HEMISPHERE JOINT" << std::endl;
        return std::make_shared<RobotNodeHemisphere>(
                    robot,
                    nodeName,
                    limitLow,
                    limitHigh,
                    preJointTransform,
                    axis,
                    visualizationModel,
                    collisionModel,
                    jointValueOffset,
                    physics,
                    (collisionModel ? collisionModel->getCollisionChecker() : CollisionCheckerPtr()),
                    rntype);
    }


    RobotNodePtr RobotNodeHemisphereFactory::createRobotNodeDH(
            RobotPtr robot,
            const std::string& nodeName,
            VisualizationNodePtr visualizationModel,
            CollisionModelPtr collisionModel,
            float limitLow,
            float limitHigh,
            float jointValueOffset,
            const DHParameter& dhParameters,
            const SceneObject::Physics& physics,
            RobotNode::RobotNodeType rntype) const
    {
        std::cout << "CREATE NEW HEMISPHERE JOINT DH" << std::endl;
        return std::make_shared<RobotNodeHemisphere>(
                    robot,
                    nodeName,
                    limitLow,
                    limitHigh,
                    dhParameters.aMM(),
                    dhParameters.dMM(),
                    dhParameters.alphaRadian(),
                    dhParameters.thetaRadian(),
                    visualizationModel,
                    collisionModel,
                    jointValueOffset,
                    physics,
                    CollisionCheckerPtr(),
                    rntype);
    }


    RobotNodeFactory::SubClassRegistry RobotNodeHemisphereFactory::registry(RobotNodeHemisphereFactory::getName(), &RobotNodeHemisphereFactory::createInstance);


    std::string RobotNodeHemisphereFactory::getName()
    {
        return "hemisphere";
    }


    std::shared_ptr<RobotNodeFactory> RobotNodeHemisphereFactory::createInstance(void*)
    {
        return std::make_shared<RobotNodeHemisphereFactory>();
    }

} // namespace VirtualRobot
