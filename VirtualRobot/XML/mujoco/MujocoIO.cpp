#include "MujocoIO.h"

#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/VirtualRobotException.h>

#include <VirtualRobot/MJCF/visitors/Collector.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/RobotIO.h>

#include "DummyMassBodySanitizer.h"
#include "MergingBodySanitizer.h"


namespace fs = std::filesystem;


namespace
{
    namespace fs = std::filesystem;
    inline fs::path removeTrailingSeparator(fs::path p)
    {
        p /= "dummy";
        return p.parent_path();
    }
}



namespace VirtualRobot::mujoco
{

template <typename EnumT>
EnumT stringToEnum(const std::map<std::string, EnumT>& map, const std::string& string)
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

ActuatorType toActuatorType(const std::string& string)
{
    return stringToEnum<ActuatorType>(
    {
        { "motor",    ActuatorType::MOTOR },
        { "position", ActuatorType::POSITION },
        { "velocity", ActuatorType::VELOCITY }
    }, string);
}

BodySanitizeMode toBodySanitizeMode(const std::string& string)
{
    return stringToEnum<BodySanitizeMode>(
    {
        { "dummy_mass", BodySanitizeMode::DUMMY_MASS },
        { "dummymass",  BodySanitizeMode::DUMMY_MASS },
        { "merge",      BodySanitizeMode::MERGE },
    }, string);
}

WorldMountMode toWorldMountMode(const std::string& string)
{
    return stringToEnum<WorldMountMode>(
    {
        { "fixed", WorldMountMode::FIXED },
        { "free",  WorldMountMode::FREE },
        { "mocap", WorldMountMode::MOCAP },
    }, string);
}



MujocoIO::MujocoIO(RobotPtr robot) : robot(robot) 
{
    THROW_VR_EXCEPTION_IF(!robot, "Given RobotPtr robot is null.");
}

std::string MujocoIO::saveMJCF(
        const std::string& filename, const std::string& basePath, const std::string& meshRelDir)
{
    THROW_VR_EXCEPTION_IF(filename.empty(), "Given filename is empty.");
    
    setPaths(filename, basePath, meshRelDir);
    
    document.reset(new mjcf::Document());
    document->setModelName(robot->getName());
    
    makeCompiler();
    
    makeDefaultsClass();
    document->setNewElementClass(robot->getName(), true);
    
    addSkybox();
    
    mjcf::Body mocapBody;
    if (worldMountMode == WorldMountMode::MOCAP)
    {
        std::cout << "Adding mocap body ..." << std::endl;
        mocapBody = addMocapBody();
    }
    
    std::cout << "Creating bodies structure ..." << std::endl;
    makeNodeBodies();
    
    std::cout << "Adding meshes and geoms ..." << std::endl;
    addNodeBodyMeshes();
    
    if (verbose)
    {
        std::cout << "===========================" << std::endl
                  << "Current model: "             << std::endl
                  << "--------------"              << std::endl;
        std::cout << *document;
        std::cout << "===========================" << std::endl;
    }
    
    
    std::cout << "Sanitizing massless bodies ..." << std::endl;
    switch (bodySanitizeMode)
    {
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
        sanitizer->setLengthScale(lengthScale);
        
        bodySanitizer = std::move(sanitizer);
    }    
        break;
    }
    bodySanitizer->sanitize(*document, robotRoot);
    
    
    std::cout << "Adding contact excludes ..." << std::endl;
    addContactExcludes();

    if (worldMountMode == WorldMountMode::MOCAP)
    {
        std::cout << "Adding mocap body contact excludes ..." << std::endl;
        addMocapContactExcludes(mocapBody);
    }
    
    std::cout << "Adding actuators ..." << std::endl;
    addActuators();
    
    std::cout << "Done." << std::endl;
    
    if (verbose)
    {
        std::cout << std::endl;
        std::cout << "===========================" << std::endl
                  << "Output file: "             << std::endl
                  << "------------"              << std::endl;
        std::cout << *document;
        std::cout << "===========================" << std::endl;
    }
    
    VR_ASSERT(!outputFileName.empty());
    
    const fs::path outputFilePath = outputDirectory / outputFileName;
    
    std::cout << "Writing to " << outputFilePath << std::endl;
    document->saveFile(outputFilePath);
    
    return outputFilePath;
}

void MujocoIO::setUseRelativePaths(bool useRelative)
{
    this->useRelativePaths = useRelative;
}

void MujocoIO::setPaths(
        const std::string& filename,
        const std::string& basePath,
        const std::string& meshRelDir)
{
    outputDirectory = basePath;
    outputFileName = filename;
    outputMeshRelDirectory = meshRelDir;
    
    ensureDirectoriesExist();
}

void MujocoIO::ensureDirectoriesExist()
{
    auto ensureDirExists = [](const fs::path& dir, const std::string& errMsgName)
    {
        if (!fs::is_directory(dir))
        {
            std::cout << "Creating directory: " << dir << std::endl;
            bool success = fs::create_directories(removeTrailingSeparator(dir));
            THROW_VR_EXCEPTION_IF(!success, "Could not create " << errMsgName << ": " << dir);
        }
    };

    ensureDirExists(outputDirectory, "output directory");
    ensureDirExists(outputMeshDirectory(), "output mesh directory");
}

void MujocoIO::makeCompiler()
{
    mjcf.addCompiler();
}

void MujocoIO::makeDefaultsClass()
{
    mjcf.addDefaultsClass(meshScale);
}

void MujocoIO::addSkybox()
{
    mjcf.addSkybox();
}


mjcf::Body MujocoIO::addMocapBody()
{
    const std::string className = "mocap";
    const float geomSize = 0.01f;
    
    std::string bodyName;
    {
        std::stringstream ss;
        ss << robot->getName() << "_Mocap";
        bodyName = ss.str();
    }
    
    // add defaults class
    mjcf::DefaultClass defaultClass = document->default_().getClass(className);
    
    mjcf::Geom geom = defaultClass.getElement<mjcf::Geom>();
    geom.rgba = Eigen::Vector4f(.9f, .5f, .5f, .5f);
    
    {
        mjcf::EqualityDefaults equality = defaultClass.getElement<mjcf::EqualityDefaults>();
        Eigen::Vector5f solimp = equality.solimp;
        solimp(1) = 0.99f;
        equality.solimp = solimp;
        //equality.solref = Eigen::Vector2f(.02f, 1.f);
    }
    
    // add body
    mjcf::Body mocap = document->worldbody().addMocapBody(bodyName, geomSize);
    mocap.childclass = className;
    
    // add equality weld constraint
    mjcf::EqualityWeld weld = document->equality().addWeld(bodyName, robot->getName(), bodyName);
    weld.class_ = className;
    
    document->contact().addExclude(mocap.name, robot->getName());
    
    return mocap;
}


void MujocoIO::makeNodeBodies()
{
    nodeBodies.clear();
    
    RobotNodePtr rootNode = robot->getRootNode();
    VR_ASSERT(rootNode);
    
    // add root
    robotRoot = document->worldbody().addBody(robot->getName(), robot->getName());
    
    switch (worldMountMode)
    {
    case WorldMountMode::FREE:
    case WorldMountMode::MOCAP:
    {
        if (!robotRoot.hasMass())
        {
            robotRoot.addDummyInertial();
        }
        mjcf::FreeJoint joint = robotRoot.addFreeJoint();
        joint.name = robot->getName();
    }        
        break;
    default:
        break;
    }
    
    mjcf::Body root = addNodeBody(robotRoot, rootNode);
    nodeBodies[rootNode->getName()] = root;
    VR_ASSERT(root);
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        addNodeBody(node);
    }
}

mjcf::Body MujocoIO::addNodeBody(mjcf::Body parent, RobotNodePtr node)
{
    mjcf.addNodeBody(node, parent);
}

mjcf::Joint MujocoIO::addNodeJoint(mjcf::Body body, RobotNodePtr node)
{
    return mjcf.addNodeJoint(node, body);
}

mjcf::Inertial MujocoIO::addNodeInertial(mjcf::Body body, RobotNodePtr node)
{
    return mjcf.addNodeInertial(node, body);
}

void MujocoIO::addNodeBodyMeshes()
{
    const bool meshlabserverAviable = true; // system("which meshlabserver > /dev/null 2>&1") == 0;
    bool notAvailableReported = false;
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        VisualizationNodePtr visualization = node->getVisualization(SceneObject::VisualizationType::Full);
        
        if (!visualization)
        {
            continue;
        }
        
        std::cout << t << "Node '" << node->getName() << "':\t";
        
        const fs::path srcMeshPath = visualization->getFilename();
        
        fs::path dstMeshFileName = srcMeshPath.filename();
        dstMeshFileName.replace_extension("stl");
        const fs::path dstMeshPath = outputMeshDirectory() / dstMeshFileName;
        
        if (!fs::exists(dstMeshPath))
        {
            if (srcMeshPath.extension() != ".stl")
            {
                std::cout << "Converting to .stl: " << srcMeshPath << std::endl;
                
                if (!meshlabserverAviable)
                {
                    if (!notAvailableReported)
                    {
                        std::cerr << std::endl 
                                  << "Command 'meshlabserver' not available, cannot convert meshes."
                                  << " (This error is only reported once.)"
                                  << std::endl;
                        notAvailableReported = true;
                    }
                    continue;
                }
                
                // meshlabserver available
                std::stringstream convertCommand;
                convertCommand << "meshlabserver"
                               << " -i " << srcMeshPath 
                               << " -o " << dstMeshPath;
                
                // run command
                std::cout << "----------------------------------------------------------" << std::endl;
                std::cout << "Running command: " << convertCommand.str() << std::endl;
                int r = system(convertCommand.str().c_str());
                std::cout << "----------------------------------------------------------" << std::endl;
                if (r != 0)
                {
                    std::cout << "Command returned with error: " << r << "\n"
                              << "Command was: " << convertCommand.str() << std::endl;
                }
            }
            else
            {
                std::cout << "Copying: " << srcMeshPath << "\n"
                          << "     to: " << dstMeshPath;
                fs::copy_file(srcMeshPath, dstMeshPath);
            }
        }
        else
        {
            std::cout << "skipping (" << outputMeshRelDirectory / dstMeshFileName 
                      << " already exists)";
        }
        std::cout << std::endl;
        
        
        
        // add asset
        const std::string meshName = node->getName();
        const fs::path meshPath = useRelativePaths 
                ? outputMeshRelDirectory / dstMeshFileName
                : fs::absolute(dstMeshPath);
        
        document->asset().addMesh(meshName, meshPath.string());
        
        // add geom to body
        mjcf::Body& body = nodeBodies.at(node->getName());
        mjcf::Geom geom = body.addGeomMesh(meshName);
        geom.name = node->getName();
    }
}



mjcf::Body MujocoIO::addNodeBody(RobotNodePtr node)
{
    mjcf.addNodeBody(node);
}

struct ParentChildContactExcludeVisitor : public mjcf::Visitor
{
    ParentChildContactExcludeVisitor(mjcf::Document& document) : mjcf::Visitor (document)
    {}
    virtual ~ParentChildContactExcludeVisitor() override = default;

    //bool VisitEnter(const tinyxml2::XMLElement&, const tinyxml2::XMLAttribute*) override;
    bool visitEnter(const mjcf::AnyElement& element) override;
    
    std::vector<std::pair<std::string, std::string>> excludePairs;
    bool firstSkipped = false;  ///< Used to skip the root element.
};

bool ParentChildContactExcludeVisitor::visitEnter(const mjcf::AnyElement& element)
{
    if (!element.is<mjcf::Body>())
    {
        return true;
    }
    
    const mjcf::Body body = element.as<mjcf::Body>();
    
    if (!firstSkipped)
    {
        firstSkipped = true;
        return true;
    }
    
    const mjcf::Body parent = body.parent<mjcf::Body>();
    VR_ASSERT(parent);
    excludePairs.emplace_back(parent.name.get(), body.name.get());
    
    return true;
}


void MujocoIO::addContactExcludes()
{
    RobotNodePtr rootNode = robot->getRootNode();
    
    std::vector<std::pair<std::string, std::string>> excludePairs;
    
    for (RobotNodePtr node : robot->getRobotNodes())
    {
        for (std::string& ignore : node->getPhysics().ignoreCollisions)
        {
            // I found an <IgnoreCollision> element referring to a non-existing node.
            // => check node existence here
            if (robot->hasRobotNode(ignore))
            {
                excludePairs.push_back({node->getName(), ignore});
            }
        }
    }
    
    // resolve body names and add exludes
    for (const auto& excludePair : excludePairs)
    {
        std::string body1 = excludePair.first;
        std::string body2 = excludePair.second;
        
        MergingBodySanitizer* mergingSanitizer = 
                dynamic_cast<MergingBodySanitizer*>(bodySanitizer.get());
        if (mergingSanitizer)
        {
            body1 = mergingSanitizer->getMergedBodyName(body1);
            body2 = mergingSanitizer->getMergedBodyName(body2);
        }
        
        // assert !body1.empty()
        // assert !body2.empty()
        document->contact().addExclude(body1, body2);
    }
    
    // Add excludes between parent and child elemenets. 
    // This should actually not be necessary?
    ParentChildContactExcludeVisitor visitor(*document);
    robotRoot.accept(visitor);
    for (const auto& excludePair : visitor.excludePairs)
    {
        document->contact().addExclude(excludePair.first, excludePair.second);
    }
}

void MujocoIO::addMocapContactExcludes(mjcf::Body mocap)
{
    if (!mocap)
    {
        throw std::invalid_argument("Passed uninitialized mocap body to " + std::string(__FUNCTION__) + "()");
    }
    for (const auto& nodeBody : nodeBodies)
    {
        document->contact().addExclude(mocap, nodeBody.second);
    }
}

void MujocoIO::addActuators()
{
    const std::vector<mjcf::Joint> jointElements = mjcf::Collector<mjcf::Joint>::collect(
                *document, document->worldbody());
    
    for (const auto& joint : jointElements)
    {
        mjcf::AnyElement actuator;
        
        const std::string jointName = joint.name;
        switch (actuatorType)
        {
            case ActuatorType::MOTOR:
            {
                mjcf::ActuatorMotor act = document->actuator().addMotor(jointName);
                actuator = act;
                break;
            }
                
            case ActuatorType::POSITION:
            {
                mjcf::ActuatorPosition act = document->actuator().addPosition(jointName);
                actuator = act;
                
                if (joint.limited)
                {
                    act.ctrllimited = joint.limited;
                    act.ctrlrange = joint.range;
                }
            }
                break;
                
            case ActuatorType::VELOCITY:
                actuator = document->actuator().addVelocity(jointName);
                break;
        }
        
        std::stringstream actuatorName;
        actuatorName << joint.name;
        if (addActuatorTypeSuffix)
        {
            actuatorName << actuatorTypeSuffixes.at(actuatorType);
        }
        actuator.setAttribute("name", actuatorName.str());
    }
}

void MujocoIO::scaleLengths(mjcf::AnyElement element)
{
    VR_ASSERT(element);
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
            joint.range = joint.range.get() * lengthScale;
        }
    }
    else if (element.is<mjcf::ActuatorPosition>())
    {
        mjcf::ActuatorPosition act = element.as<mjcf::ActuatorPosition>();
        if (act.ctrlrange.isSet())
        {
            std::cout << t << "Scaling ctrlrange of position actuator '" << act.name << "'" << std::endl;
            act.ctrlrange = act.ctrlrange.get() * lengthScale;
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
        pos *= lengthScale;
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
    this->lengthScale = value;
}

void MujocoIO::setMeshScale(float value)
{
    this->meshScale = value;
}

void MujocoIO::setMassScale(float value)
{
    this->massScale = value;
}

void MujocoIO::setActuatorType(ActuatorType value)
{
    this->actuatorType = value;
}

void MujocoIO::setAddActuatorTypeSuffix(bool enable)
{
    this->addActuatorTypeSuffix = enable;
}

void MujocoIO::setActuatorTypeSuffixes(const std::map<ActuatorType, std::string>& suffixes)
{
    this->actuatorTypeSuffixes = suffixes;
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
