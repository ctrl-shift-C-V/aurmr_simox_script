#include "RobotNodeHemisphere.h"
#include "Nodes/Sensor.h"
#include "Robot.h"
#include "VirtualRobotException.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include <SimoxUtility/math/pose/pose.h>


namespace VirtualRobot
{


    VirtualRobot::RobotNodeHemisphere::RobotNodeHemisphere()
    {
    }


    RobotNodeHemisphere::RobotNodeHemisphere(
            RobotWeakPtr rob,
            const std::string& name,
            float jointLimitLo,
            float jointLimitHi,
            const Eigen::Matrix4f& preJointTransform,
            const Eigen::Vector3f& axis,
            VisualizationNodePtr visualization,
            CollisionModelPtr collisionModel,
            float jointValueOffset,
            const SceneObject::Physics& p,
            CollisionCheckerPtr colChecker,
            RobotNodeType type,
            bool isSub
            ) :
        RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel,
                  jointValueOffset, p, colChecker, type),
        isSub(isSub)
    {
        initialized = false;
        optionalDHParameter.isSet = false;
        this->localTransformation = preJointTransform;
        this->jointRotationAxis = axis;
        this->jointRotationAxis.normalize();
        checkValidRobotNodeType();
    }


    RobotNodeHemisphere::RobotNodeHemisphere(
            RobotWeakPtr rob,
            const std::string& name,
            float jointLimitLo,
            float jointLimitHi,
            float a, float d, float alpha, float theta,
            VisualizationNodePtr visualization,
            CollisionModelPtr collisionModel,
            float jointValueOffset,
            const SceneObject::Physics& physics,
            CollisionCheckerPtr colChecker,
            RobotNodeType type
            ) :
        RobotNode(rob, name, jointLimitLo, jointLimitHi, visualization, collisionModel,
                  jointValueOffset, physics, colChecker, type)

    {
        initialized = false;
        optionalDHParameter.isSet = true;
        optionalDHParameter.setAInMM(a);
        optionalDHParameter.setDInMM(d);
        optionalDHParameter.setAlphaRadian(alpha, true);
        optionalDHParameter.setThetaRadian(theta, true);

        // compute DH transformation matrices
        Eigen::Matrix4f RotTheta = Eigen::Matrix4f::Identity();
        RotTheta(0, 0) = cos(theta);
        RotTheta(0, 1) = -sin(theta);
        RotTheta(1, 0) = sin(theta);
        RotTheta(1, 1) = cos(theta);
        Eigen::Matrix4f TransD = Eigen::Matrix4f::Identity();
        TransD(2, 3) = d;
        Eigen::Matrix4f TransA = Eigen::Matrix4f::Identity();
        TransA(0, 3) = a;
        Eigen::Matrix4f RotAlpha = Eigen::Matrix4f::Identity();
        RotAlpha(1, 1) = cos(alpha);
        RotAlpha(1, 2) = -sin(alpha);
        RotAlpha(2, 1) = sin(alpha);
        RotAlpha(2, 2) = cos(alpha);

        this->localTransformation = RotTheta * TransD * TransA * RotAlpha;
        this->jointRotationAxis = Eigen::Vector3f::UnitZ();  // rotation around z axis
        checkValidRobotNodeType();
    }


    RobotNodeHemisphere::~RobotNodeHemisphere()
    = default;


    bool RobotNodeHemisphere::initialize(
            SceneObjectPtr parent,
            const std::vector<SceneObjectPtr>& children)
    {
        // Create a sub joint as a child.
        if (not isSub)
        {
            const bool isSub = true;
            RobotNodeHemispherePtr sub = std::make_shared<RobotNodeHemisphere>(
                        robot,
                        name + "_sub",
                        jointLimitLo,
                        jointLimitHi,
                        localTransformation,  // const Eigen::Matrix4f& preJointTransform,
                        jointRotationAxis,  // const Eigen::Vector3f& axis,
                        nullptr,  // visualizationModel,
                        nullptr,  // collisionModel
                        jointValueOffset,
                        Physics{},  //physics,
                        nullptr,  // collisionChecker,
                        nodeType,
                        isSub
                        );
            sub->initialize(shared_from_this(), children);

            bool success = RobotNode::initialize(parent, {sub});
            return success;
        }
        else
        {
            return RobotNode::initialize(parent, children);
        }
    }


    void RobotNodeHemisphere::updateTransformationMatrices(
            const Eigen::Matrix4f& parentPose)
    {
        const Eigen::Matrix4f rot = simox::math::pose(
                    Eigen::AngleAxisf(jointValue + jointValueOffset, jointRotationAxis));
        globalPose = parentPose * localTransformation * rot;

        std::cout << __FUNCTION__ << "() with "
                  << "joint value = " << jointValue
                  << ", joint vaue offset = " << jointValueOffset
                  << ", joint rotation axis = " << jointRotationAxis.transpose()
                  << ", | joint rotation axis | = " << jointRotationAxis.norm()
                  << ", rot matrix: \n" << rot
                  << std::endl;
    }


    void RobotNodeHemisphere::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();

        if (printDecoration)
        {
            std::cout << "******** RobotNodeHemisphere ********" << std::endl;
        }

        RobotNode::print(false, false);

        std::cout << "* JointRotationAxis: " << jointRotationAxis.transpose() << std::endl;

        if (printDecoration)
        {
            std::cout << "******** End RobotNodeHemisphere ********" << std::endl;
        }

        if (printChildren)
        {
            for (const SceneObjectPtr& child : this->getChildren())
            {
                child->print(true, true);
            }
        }
    }


    RobotNodePtr RobotNodeHemisphere::_clone(
            const RobotPtr newRobot,
            const VisualizationNodePtr visualizationModel,
            const CollisionModelPtr collisionModel,
            CollisionCheckerPtr colChecker,
            float scaling)
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        RobotNodePtr result;

        Physics p = physics.scale(scaling);

        if (optionalDHParameter.isSet)
        {
            result.reset(new RobotNodeHemisphere(newRobot, name, jointLimitLo, jointLimitHi, optionalDHParameter.aMM()*scaling, optionalDHParameter.dMM()*scaling, optionalDHParameter.alphaRadian(), optionalDHParameter.thetaRadian(), visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }
        else
        {
            Eigen::Matrix4f lt = getLocalTransformation();
            simox::math::position(lt) *= scaling;
            result.reset(new RobotNodeHemisphere(newRobot, name, jointLimitLo, jointLimitHi, lt, jointRotationAxis, visualizationModel, collisionModel, jointValueOffset, p, colChecker, nodeType));
        }

        return result;
    }


    bool RobotNodeHemisphere::isRotationalJoint() const
    {
        return true;
    }


    Eigen::Vector3f RobotNodeHemisphere::getJointRotationAxisInJointCoordSystem() const
    {
        return jointRotationAxis;
    }


    void RobotNodeHemisphere::setJointRotationAxis(const Eigen::Vector3f& newAxis)
    {
        this->jointRotationAxis = newAxis;
    }


    Eigen::Vector3f RobotNodeHemisphere::getJointRotationAxis(const SceneObjectPtr coordSystem) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        Eigen::Vector4f result4f = Eigen::Vector4f::Zero();
        result4f.segment(0, 3) = jointRotationAxis;
        result4f = globalPose * result4f;

        if (coordSystem)
        {
            result4f = coordSystem->getGlobalPose().inverse() * result4f;
        }

        return result4f.head<3>();
    }

    void RobotNodeHemisphere::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform, "RobotNodeHemisphere must be a JointNode or a GenericNode");
    }


    std::string RobotNodeHemisphere::_toXML(const std::string& /*modelPath*/)
    {
        std::stringstream ss;
        ss << "\t\t<Joint type='Hemisphere'>" << std::endl;
        ss << "\t\t\t<axis x='" << jointRotationAxis[0] << "' y='" << jointRotationAxis[1] << "' z='" << jointRotationAxis[2] << "'/>" << std::endl;
        ss << "\t\t\t<limits lo='" << jointLimitLo << "' hi='" << jointLimitHi << "' units='radian'/>" << std::endl;
        ss << "\t\t\t<MaxAcceleration value='" << maxAcceleration << "'/>" << std::endl;
        ss << "\t\t\t<MaxVelocity value='" << maxVelocity << "'/>" << std::endl;
        ss << "\t\t\t<MaxTorque value='" << maxTorque << "'/>" << std::endl;
        std::map< std::string, float >::iterator propIt = propagatedJointValues.begin();

        while (propIt != propagatedJointValues.end())
        {
            ss << "\t\t\t<PropagateJointValue name='" << propIt->first << "' factor='" << propIt->second << "'/>" << std::endl;
            propIt++;
        }

        ss << "\t\t</Joint>" << std::endl;
        return ss.str();
    }


    float RobotNodeHemisphere::getLMTC(float angle)
    {
        return std::sqrt(2 - 2 * std::cos(angle));
    }


    float RobotNodeHemisphere::getLMomentArm(float angle)
    {
        float b = getLMTC(angle);
        return std::sqrt(4 * std::pow(b, 2) - std::pow(b, 4)) / (2 * b);
    }

}
