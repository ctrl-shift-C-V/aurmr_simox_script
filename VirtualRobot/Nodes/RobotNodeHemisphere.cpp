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

    static const float limit = 0.6;


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
            const SceneObject::Physics& physics,
            CollisionCheckerPtr colChecker,
            RobotNodeType type,
            bool isTail
            ) :
        RobotNode(rob, name, -limit, limit, visualization, collisionModel,
                  jointValueOffset, physics, colChecker, type)
    {
        (void) axis;
        (void) jointLimitLo, (void) jointLimitHi;

        if (isTail)
        {
            tail.emplace(Tail{});
        }
        else
        {
            head.emplace(Head{});
        }

        initialized = false;
        optionalDHParameter.isSet = false;
        localTransformation = preJointTransform;
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
                  jointValueOffset, physics, colChecker, type),
        tail(Tail{})
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

        localTransformation = RotTheta * TransD * TransA * RotAlpha;
        checkValidRobotNodeType();
    }


    RobotNodeHemispherePtr RobotNodeHemisphere::MakeHead(
            RobotWeakPtr robot, const std::string& name, RobotNodeType type)
    {
        bool isTail = false;
        return std::make_shared<RobotNodeHemisphere>(
                    robot, name, -limit, limit,
                    Eigen::Matrix4f::Identity(), Eigen::Vector3f::Zero(),
                    nullptr, nullptr, 0.0, Physics{}, nullptr, type,
                    isTail);
    }


    RobotNodeHemisphere::~RobotNodeHemisphere()
    = default;


    bool RobotNodeHemisphere::initialize(
            SceneObjectPtr parent,
            const std::vector<SceneObjectPtr>& _children)
    {
        (void) _children;
        VR_ASSERT_MESSAGE(head xor sub, head.has_value() << " / " sub.has_value());

        if (tail)
        {
            if (not tail->head)
            {
                // Create a head joint as a child of parent.
                RobotNodeHemispherePtr headNode = RobotNodeHemisphere::MakeHead(
                            robot, name + "_head", nodeType
                            );

                headNode->setLocalTransformation(this->localTransformation);
                this->localTransformation.setIdentity();

                // Start:  parent -> tail
                // Goal:   parent -> head -> tail
                SceneObjectPtr tailNode = shared_from_this();
                parent->detachChild(tailNode);
                headNode->attachChild(tailNode);
                parent->attachChild(headNode);

                tail->head = headNode;

                headNode->initialize(parent, {tailNode});
                // Stop here, recurse when the head node initializes this node (its child).
                return true;
            }
            else
            {
                // Recurse.
                return RobotNode::initialize(parent, children);
            }
        }
        else
        {
            return RobotNode::initialize(parent, children);
        }
    }


    void RobotNodeHemisphere::updateTransformationMatrices(
            const Eigen::Matrix4f& parentPose)
    {
        VR_ASSERT_MESSAGE(head xor sub, head.has_value() << " / " sub.has_value());

        const double maxNorm = 1;

        if (head)
        {
            globalPose = parentPose * localTransformation;
        }
        else if (tail)
        {
            Eigen::Vector2d a(tail->head->getJointValue(), this->getJointValue());
            double norm = a.norm();
            if (norm > maxNorm)
            {
                a = a / norm * maxNorm;
            }
            tail->joint.computeFK(a(0), a(1));

            Eigen::Vector3d translation = tail->joint.getEndEffectorTranslation();
            translation = translation.normalized() * tail->joint.radius;
            const Eigen::Matrix3d rotation = tail->joint.getEndEffectorRotation();
            const Eigen::Matrix4d transform = simox::math::pose(translation, rotation);

            globalPose = parentPose * localTransformation * transform.cast<float>();

            Eigen::IOFormat iof(5, 0, " ", "\n", "    [", "]");
            std::cout << __FUNCTION__ << "() with "
                      << "\n  lever = " << tail->joint.lever
                      << "\n  theta0 = " << tail->joint.theta0
                      << "\n  radius = " << tail->joint.radius
                      << "\n  actuator offset = " << tail->joint.actuatorOffset
                      << "\n  joint value = " << jointValue
                      << "\n  joint vaue offset = " << jointValueOffset
                      << "\n  actuator = \n" << a.transpose().format(iof)
                      << "\n  local transform = \n" << localTransformation.format(iof)
                      << "\n  transform = \n" << transform.format(iof)
                      << std::endl;
        }
    }


    void RobotNodeHemisphere::print(bool printChildren, bool printDecoration) const
    {
        ReadLockPtr lock = getRobot()->getReadLock();
        VR_ASSERT_MESSAGE(head xor sub, head.has_value() << " / " sub.has_value());

        if (printDecoration)
        {
            std::cout << "******** RobotNodeHemisphere ********" << std::endl;
        }

        RobotNode::print(false, false);

        if (head)
        {
            std::cout << "* Hemisphere joint head node";
        }
        else if (tail)
        {
            std::cout << "* Hemisphere joint tail node";
            std::cout << "* Transform: \n" << tail->joint.getEndEffectorTransform() << std::endl;
        }

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
            Eigen::Matrix4f localTransform = getLocalTransformation();
            simox::math::position(localTransform) *= scaling;
            result.reset(new RobotNodeHemisphere(
                             newRobot, name,
                             jointLimitLo, jointLimitHi,
                             localTransform, Eigen::Vector3f::Zero(),
                             visualizationModel, collisionModel,
                             jointValueOffset, p, colChecker, nodeType));
        }

        return result;
    }


    bool RobotNodeHemisphere::isHemisphereJoint() const
    {
        return true;
    }

    void RobotNodeHemisphere::setConstants(double lever, double theta0)
    {
        VR_ASSERT(tail);
        tail->joint.setConstants(lever, theta0);
    }

    void RobotNodeHemisphere::checkValidRobotNodeType()
    {
        RobotNode::checkValidRobotNodeType();
        THROW_VR_EXCEPTION_IF(nodeType == Body || nodeType == Transform, "RobotNodeHemisphere must be a JointNode or a GenericNode");
    }


    std::string RobotNodeHemisphere::_toXML(const std::string& /*modelPath*/)
    {
        VR_ASSERT_MESSAGE(head xor sub, head.has_value() << " / " sub.has_value());

        if (head)
        {
            // Hidden, not part of xml.
            return "";
        }
        else
        {
            std::stringstream ss;
            ss << "\t\t<Joint type='Hemisphere'>" << std::endl;
            ss << "\t\t\t<hemisphere lever='" << tail->joint.lever << "' theta0='" << tail->joint.theta0 << "' />" << std::endl;
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
