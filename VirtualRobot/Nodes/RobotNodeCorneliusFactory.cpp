/**
* @package    VirtualRobot
* @author     Rainer Kartmann
* @copyright  2022 Rainer Kartmann
*/

#include "RobotNodeCorneliusFactory.h"
#include "RobotNode.h"
#include "RobotNodeCornelius.h"
#include "../CollisionDetection/CollisionModel.h"


namespace VirtualRobot
{

    RobotNodeCorneliusFactory::RobotNodeCorneliusFactory()
    = default;


    RobotNodeCorneliusFactory::~RobotNodeCorneliusFactory()
    = default;


    /**
     * This method creates a VirtualRobot::RobotNodeCornelius.
     *
     * \return instance of VirtualRobot::RobotNodeCornelius.
     */
    RobotNodePtr RobotNodeCorneliusFactory::createRobotNode(
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
        std::cout << "CREATE NEW CORNELIUS JOINT" << std::endl;
        return std::make_shared<RobotNodeCornelius>(
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


    /**
     * This method creates a VirtualRobot::RobotNodeCornelius from DH parameters.
     *
     * \return instance of VirtualRobot::RobotNodeCornelius.
     */
    RobotNodePtr RobotNodeCorneliusFactory::createRobotNodeDH(
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
        std::cout << "CREATE NEW CORNELIUS JOINT DH" << std::endl;
        return std::make_shared<RobotNodeCornelius>(
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


    /**
     * register this class in the super class factory
     */
    RobotNodeFactory::SubClassRegistry RobotNodeCorneliusFactory::registry(RobotNodeCorneliusFactory::getName(), &RobotNodeCorneliusFactory::createInstance);


    std::string RobotNodeCorneliusFactory::getName()
    {
        return "cornelius";
    }


    /**
     * \return new instance of RobotNodeCorneliusFactory.
     */
    std::shared_ptr<RobotNodeFactory> RobotNodeCorneliusFactory::createInstance(void*)
    {
        return std::make_shared<RobotNodeCorneliusFactory>();
    }

} // namespace VirtualRobot
