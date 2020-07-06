
#include "Sensor.h"
#include "../VirtualRobot.h"
#include "../VirtualRobotException.h"
#include "../XML/BaseIO.h"
#include "../Visualization/TriMeshModel.h"
#include "RobotNode.h"
#include <VirtualRobot/Visualization/VisualizationNode.h>

#include <Eigen/Core>

#include <iomanip>

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    Sensor::Sensor(RobotNodeWeakPtr robotNode,
                   const std::string& name,
                   VisualizationNodePtr visualization,
                   const Eigen::Matrix4f& rnTrafo
                  ) : SceneObject(name, visualization)
    {
        this->robotNode = robotNode;
        rnTransformation = rnTrafo;
    }


    Sensor::~Sensor()
    = default;


    bool Sensor::initialize(SceneObjectPtr parent, const std::vector<SceneObjectPtr>& children)
    {
        RobotNodePtr rn = robotNode.lock();
        THROW_VR_EXCEPTION_IF(!rn, "Could not init Sensor without robotnode");

        // robotnode
        if (!rn->hasSensor(name))
        {
            rn->registerSensor(std::static_pointer_cast<Sensor>(shared_from_this()));
        }

        return SceneObject::initialize(parent, children);
    }

    RobotNodePtr Sensor::getRobotNode() const
    {
        RobotNodePtr result(robotNode);
        return result;
    }

    void Sensor::setGlobalPose(const Eigen::Matrix4f& /*pose*/)
    {
        THROW_VR_EXCEPTION("Use setJointValues of parent RobotNode to control the position of a Sensor");
    }

    void Sensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            std::cout << "******** Sensor ********" << std::endl;
        }

        std::cout << "* Name: " << name << std::endl;
        std::cout << "* Parent: ";
        SceneObjectPtr p = this->getParent();

        if (p)
        {
            std::cout << p->getName() << std::endl;
        }
        else
        {
            std::cout << " -- " << std::endl;
        }

        std::cout << "* visualization model: " << std::endl;

        if (visualizationModel)
        {
            visualizationModel->print();
        }
        else
        {
            std::cout << "  No visualization model" << std::endl;
        }

        if (initialized)
        {
            std::cout << "* initialized: true" << std::endl;
        }
        else
        {
            std::cout << "* initialized: false" << std::endl;
        }

        {
            // scope1
            std::ostringstream sos;
            sos << std::setiosflags(std::ios::fixed);
            sos << "* RobotNode to sensor transformation:" << endl << rnTransformation << std::endl;
            sos << "* globalPose:" << endl << getGlobalPose() << std::endl;
            std::cout << sos.str();
        } // scope1

        if (printDecoration)
        {
            std::cout << "******** End Sensor ********" << std::endl;
        }

        if (printChildren)
        {
            std::vector< SceneObjectPtr > children = this->getChildren();

            for (auto & i : children)
            {
                i->print(true, true);
            }
        }
    }

    SensorPtr Sensor::clone(RobotNodePtr newRobotNode, float scaling)
    {
        if (!newRobotNode)
        {
            VR_ERROR << "Attempting to clone Sensor for invalid robot node";
            return SensorPtr();
        }


        VisualizationNodePtr clonedVisualizationNode;

        if (visualizationModel)
        {
            clonedVisualizationNode = visualizationModel->clone(true, scaling);
        }


        SensorPtr result = _clone(newRobotNode, clonedVisualizationNode, scaling);

        if (!result)
        {
            VR_ERROR << "Cloning failed.." << std::endl;
            return result;
        }

        newRobotNode->registerSensor(result);

        result->initialize(newRobotNode);

        return result;
    }

    void Sensor::setRobotNodeToSensorTransformation(const Eigen::Matrix4f& t)
    {
        this->rnTransformation = t;
        updatePose();
    }

    void Sensor::updatePose(bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

        SceneObjectPtr p = getParent();

        if (p)
        {
            this->globalPose = p->getGlobalPose() * rnTransformation;
        } else
            this->globalPose = rnTransformation;

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    void Sensor::updatePose(const Eigen::Matrix4f& globalPose, bool updateChildren)
    {
        THROW_VR_EXCEPTION_IF(!initialized, "Not initialized");

        this->globalPose = globalPose * rnTransformation;

        // update collision and visualization model and children
        SceneObject::updatePose(updateChildren);
    }

    std::string Sensor::toXML(const std::string& modelPath, int tabs)
    {
        // this will not work, since no type is available, just a reference implementation of a sensor XML tag
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor name='" << name << "'>" << std::endl;
        std::string pre2 = pre + t;
        ss << pre << "<Transform>" << std::endl;
        ss << BaseIO::toXML(rnTransformation, pre2);
        ss << pre << "</Transform>" << std::endl;

        if (visualizationModel && visualizationModel->getTriMeshModel() && visualizationModel->getTriMeshModel()->faces.size() > 0)
        {
            ss << visualizationModel->toXML(modelPath, tabs + 1);
        }

        ss << pre << "</Sensor>" << std::endl;
        return ss.str();
    }


} // namespace VirtualRobot
