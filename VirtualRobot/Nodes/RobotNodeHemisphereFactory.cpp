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


    /**
     * This method creates a VirtualRobot::RobotNodeHemisphere.
     *
     * \return instance of VirtualRobot::RobotNodeHemisphere.
     */
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


    /**
     * This method creates a VirtualRobot::RobotNodeHemisphere from DH parameters.
     *
     * \return instance of VirtualRobot::RobotNodeHemisphere.
     */
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


    /**
     * register this class in the super class factory
     */
    RobotNodeFactory::SubClassRegistry RobotNodeHemisphereFactory::registry(RobotNodeHemisphereFactory::getName(), &RobotNodeHemisphereFactory::createInstance);


    std::string RobotNodeHemisphereFactory::getName()
    {
        return "hemisphere";
    }


    /**
     * \return new instance of RobotNodeHemisphereFactory.
     */
    std::shared_ptr<RobotNodeFactory> RobotNodeHemisphereFactory::createInstance(void*)
    {
        return std::make_shared<RobotNodeHemisphereFactory>();
    }

} // namespace VirtualRobot
