#include "MujocoIO.h"

#include <filesystem>

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobotChecks.h>

#include <VirtualRobot/MJCF/visitors/Collector.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "body_sanitizer/DummyMassBodySanitizer.h"
#include "body_sanitizer/MergingBodySanitizer.h"


namespace fs = std::filesystem;


namespace VirtualRobot::mujoco
{

template <typename EnumT>
static EnumT stringToEnum(const std::map<std::string, EnumT>& map, const std::string& string)
{
    std::string lower = string;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    try
    {
        return map.at(lower);
    }
    catch (const std::out_of_range&)
    {
        std::stringstream msg;
        msg << "Invalid key '" << string << "'. Valid keys are: ";
        for (const auto& item : map)
        {
            msg << item.first << " ";
        }
        throw std::out_of_range(msg.str());
    }
}


BodySanitizeMode toBodySanitizeMode(const std::string& string)
{
    return stringToEnum<BodySanitizeMode>(
    {
        { "none",       BodySanitizeMode::NONE, },
        { "dummy_mass", BodySanitizeMode::DUMMY_MASS },
        { "dummymass",  BodySanitizeMode::DUMMY_MASS },
        { "merge",      BodySanitizeMode::MERGE },
    }, string);
}


MujocoIO::MujocoIO(RobotPtr robot) : robot(robot), mjcf(robot)
{
    VR_CHECK_HINT(robot, "RobotPtr must not be null.");
}

std::string MujocoIO::saveMJCF(
        const std::string& filename, const std::string& basePath, const std::string& meshRelDir)
{
    VR_CHECK_HINT(!filename.empty(), "Filename must not be empty.");
    
    // Reset MJCF.
    mjcf.reset();
    
    // Set paths.
    mjcf.setOutputPaths(fs::path(basePath) / filename, meshRelDir);
    
    // Add meta elements.
    mjcf.addCompiler();
    
    mjcf.addRobotDefaults();
    mjcf.getDocument().setNewElementClass(robot->getName(), true);
    
    mjcf.addSkybox();
    
    mjcf::Body mocapBody;
    switch (worldMountMode)
    {
    case WorldMountMode::FIXED:
        break;
    
    case WorldMountMode::FREE:
        mjcf.getRobotBody().addFreeJoint();
        break;
        
    case WorldMountMode::MOCAP:
        std::cout << "Adding mocap body ..." << std::endl;
        mjcf.getRobotBody().addFreeJoint();
        mocapBody = mjcf.addMocapBodyWeldRobot();
        break;
    }
    
    std::cout << "Creating bodies structure ..." << std::endl;
    mjcf.addNodeBodies();
    
    std::cout << "Adding meshes and geoms ..." << std::endl;
    mjcf.addNodeBodyMeshes();
    
    if (verbose)
    {
        std::cout << "===========================" << std::endl
                  << "Current model: "             << std::endl
                  << "--------------"              << std::endl;
        std::cout << mjcf.getDocument();
        std::cout << "===========================" << std::endl;
    }
    
    
    std::cout << "Sanitizing massless bodies ..." << std::endl;
    switch (bodySanitizeMode)
    {
    case BodySanitizeMode::NONE:
        std::cout << "Doing nothing." << std::endl;
        break;
        
    case BodySanitizeMode::DUMMY_MASS:
    {
        std::cout << t << "Adding dummy masses ..." << std::endl;
        bodySanitizer.reset(new DummyMassBodySanitizer());
    }
        break;
        
    case BodySanitizeMode::MERGE:
    {
        std::cout << t << "Merging massless bodies ..." << std::endl;
        
        std::unique_ptr<MergingBodySanitizer> sanitizer(new MergingBodySanitizer(robot));
        sanitizer->setLengthScale(mjcf.getLengthScale());
        
        bodySanitizer = std::move(sanitizer);
    }    
        break;
    }
    bodySanitizer->sanitize(mjcf.getDocument(), mjcf.getRobotBody());
    
    
    std::cout << "Adding contact excludes ..." << std::endl;
    mjcf.addContactExcludes();

    if (worldMountMode == WorldMountMode::MOCAP)
    {
        std::cout << "Adding mocap body contact excludes ..." << std::endl;
        VR_CHECK(mocapBody);
        mjcf.addMocapContactExcludes(mocapBody);
    }
    
    std::cout << "Adding actuators ..." << std::endl;
    mjcf.addActuators(actuatorType);
    
    std::cout << "Done." << std::endl;
    
    if (verbose)
    {
        std::cout << std::endl;
        std::cout << "===========================" << std::endl
                  << "Output file: "             << std::endl
                  << "------------"              << std::endl;
        std::cout << mjcf.getDocument();
        std::cout << "===========================" << std::endl;
    }
    
    
    const fs::path outputFilePath = mjcf.getOutputFile();
    
    VR_CHECK(!outputFilePath.empty());
    
    std::cout << "Writing to " << outputFilePath << std::endl;
    mjcf.getDocument().saveFile(outputFilePath);
    
    return outputFilePath;
}

RobotMjcf& MujocoIO::getMjcf()
{
    return mjcf;
}

const RobotMjcf& MujocoIO::getMjcf() const
{
    return mjcf;
}

void MujocoIO::setUseRelativePaths(bool useRelative)
{
    mjcf.setUseRelativePaths(useRelative);
}

void MujocoIO::scaleLengths(mjcf::AnyElement element)
{
    VR_CHECK(element);
    if (verbose)
    {
        std::cout << "Traversing element " << element.actualTag() << std::endl;
    }
    
    if (element.is<mjcf::Joint>())
    {
        mjcf::Joint joint = element.as<mjcf::Joint>();
        
        if (joint.type == "slide" && joint.range.isSet())
        {
            std::cout << t << "Scaling range of slide joint '" << joint.name << "'" << std::endl;
            joint.range = joint.range.get() * mjcf.getLengthScale();
        }
    }
    else if (element.is<mjcf::ActuatorPosition>())
    {
        mjcf::ActuatorPosition act = element.as<mjcf::ActuatorPosition>();
        if (act.ctrlrange.isSet())
        {
            std::cout << t << "Scaling ctrlrange of position actuator '" << act.name << "'" << std::endl;
            act.ctrlrange = act.ctrlrange.get() * mjcf.getLengthScale();
        }
    }
    else if (element.isAttributeSet("pos"))
    {
        std::cout << t << "Scaling pos of " << element.actualTag() << " ";
        if (element.isAttributeSet("name"))
        {
            std::cout << "'" << element.getAttribute("name") << "'";
        }
        else
        {
            std::cout << "element";
        }
        std::cout << std::endl;
        
        Eigen::Vector3f pos = element.getAttribute<Eigen::Vector3f>("pos");
        pos *= mjcf.getLengthScale();
        element.setAttribute("pos", pos);
    }
    
    for (mjcf::AnyElement child = element.firstChild<mjcf::AnyElement>();
         child; child = child.nextSiblingElement<mjcf::AnyElement>())
    {
        scaleLengths(child);
    }
}

void MujocoIO::setLengthScale(float value)
{
    mjcf.setLengthScale(value);
}

void MujocoIO::setMeshScale(float value)
{
    mjcf.setMeshScale(value);
}

void MujocoIO::setMassScale(float value)
{
    mjcf.setMassScale(value);
}

void MujocoIO::setActuatorType(ActuatorType value)
{
    this->actuatorType = value;
}

void MujocoIO::setBodySanitizeMode(BodySanitizeMode mode)
{
    this->bodySanitizeMode = mode;
}

WorldMountMode MujocoIO::getWorldMountMode() const
{
    return worldMountMode;
}

void MujocoIO::setWorldMountMode(const WorldMountMode& value)
{
    worldMountMode = value;
}

void MujocoIO::setVerbose(bool value)
{
    this->verbose = value;
}


}
