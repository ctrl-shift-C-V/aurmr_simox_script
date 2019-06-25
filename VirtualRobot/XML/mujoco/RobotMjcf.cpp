#include "RobotMjcf.h"

#include <boost/shared_ptr.hpp>

#include <VirtualRobot/RobotNodeSet.h>

#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>


namespace fs = std::filesystem;


namespace VirtualRobot::mujoco
{

RobotMjcf::RobotMjcf(RobotPtr robot) : robot(robot)
{
    document->setModelName(robot->getName());
    
    // Add robot root.
    robotBody = document->worldbody().addBody(robot->getName(), robot->getName());
    nodeBodies[robot->getName()] = robotBody;
}

mjcf::Document& RobotMjcf::getDocument()
{
    return *document;
}

const mjcf::Document& RobotMjcf::getDocument() const
{
    return *document;
}

RobotPtr RobotMjcf::getRobot() const
{
    return robot;
}

mjcf::Body RobotMjcf::getRobotNodeBody(const std::string& nodeName) const
{
    return nodeBodies.at(nodeName);
}

void RobotMjcf::setOutputFile(const std::filesystem::path& filePath)
{
    this->outputFile = filePath;
}

void RobotMjcf::addCompiler(const std::string& angle, bool balanceIneratia)
{
    document->compiler().angle = angle;
    document->compiler().balanceinertia = balanceIneratia;
}

void RobotMjcf::addDefaultsClass(float meshScale)
{
    mjcf::DefaultClass defaultsClass = document->default_().addClass(robot->getName());
    
    defaultsClass.insertComment("Add default values for " + robot->getName() + " here.", true);
    
    mjcf::Joint joint = defaultsClass.getElement<mjcf::Joint>();
    joint.frictionloss = 1;
    joint.damping = 0;
    
    mjcf::Mesh mesh = defaultsClass.getElement<mjcf::Mesh>();
    mesh.scale = Eigen::Vector3f::Constant(meshScale);
    
    mjcf::Geom geom = defaultsClass.getElement<mjcf::Geom>();
    geom.condim = 4;
    
    mjcf::ActuatorPosition actPos = defaultsClass.getElement<mjcf::ActuatorPosition>();
    actPos.kp = 1;
    
    mjcf::ActuatorVelocity actVel = defaultsClass.getElement<mjcf::ActuatorVelocity >();
    actVel.kv = 1;
}

void RobotMjcf::addSkybox(const Eigen::Vector3f& rgb1, const Eigen::Vector3f& rgb2)
{
    document->asset().addSkyboxTexture(rgb1, rgb2);
}

mjcf::Body RobotMjcf::addNodeBody(RobotNodePtr node, mjcf::Body parent,
                                  bool addJoint, bool addInertial)
{
    mjcf::Body body = parent.addBody(node->getName());

    if (node->hasParent())
    {
        Eigen::Matrix4f pose = node->getTransformationFrom(node->getParent());
        math::Helpers::ScaleTranslation(pose, lengthScale);
        body.setPose(pose);
    }
    
    if (addJoint && (node->isRotationalJoint() || node->isTranslationalJoint()))
    {
        addNodeJoint(node, body);
    }
    
    if (addInertial)
    {
        addNodeInertial(node, body);
    }
    
    return body;
}

mjcf::Joint RobotMjcf::addNodeJoint(RobotNodePtr node, mjcf::Body body)
{
    VR_ASSERT(node->isRotationalJoint() xor node->isTranslationalJoint());
    
    mjcf::Joint joint = body.addJoint();
    joint.name = node->getName();
    
    // get the axis
    Eigen::Vector3f axis;
    if (node->isRotationalJoint())
    {
        RobotNodeRevolutePtr revolute = boost::dynamic_pointer_cast<RobotNodeRevolute>(node);
        VR_ASSERT(revolute);
        axis = revolute->getJointRotationAxisInJointCoordSystem();
    }
    else if (node->isTranslationalJoint())
    {
        RobotNodePrismaticPtr prismatic = boost::dynamic_pointer_cast<RobotNodePrismatic>(node);
        VR_ASSERT(prismatic);
        axis = prismatic->getJointTranslationDirectionJointCoordSystem();
    }
    
    joint.type = node->isRotationalJoint() ? "hinge" : "slide";
    joint.axis = axis;
    joint.limited = !node->isLimitless();
    
    if (!node->isLimitless())
    {
        Eigen::Vector2f range = { node->getJointLimitLow(), node->getJointLimitHigh() };
        if (node->isTranslationalJoint())
        {
            range *= lengthScale;
        }
        
        // Mujoco does not like ranges where min >= max.
        if (std::abs(range(0) - range(1)) < 1e-6f)
        {
            range(1) = range(0) + 1e-6f;
        }
        joint.range = range;
    }
    
    return joint;
}

mjcf::Inertial RobotMjcf::addNodeInertial(RobotNodePtr node, mjcf::Body body)
{
    const Eigen::Matrix3f matrix = node->getInertiaMatrix();
    if (matrix.isIdentity(document->getFloatCompPrecision()) 
        && node->getMass() < document->getFloatCompPrecision())
    {
        // Dont set an inertial element and let it be derived automatically.
        return { };
    }
    
    mjcf::Inertial inertial = body.addInertial();
    inertial.pos = node->getCoMLocal() * lengthScale;
    inertial.mass = node->getMass() * massScale;
    inertial.inertiaFromMatrix(matrix);
    
    return inertial;
}

mjcf::Body RobotMjcf::addNodeBody(RobotNodePtr node, bool addJoint, bool addInertial)
{
    // See whether body for the node already exists.
    auto find = nodeBodies.find(node->getName());
    if (find != nodeBodies.end())
    {
        // Exists => break recursion.
        return find->second;
    }
    
    // Check whether body exists for parent node.
    mjcf::Body parent;
    
    if (node->getName() == robot->getRootNode()->getName())
    {
        parent = robotBody;
    }
    else
    {
        find = nodeBodies.find(node->getParent()->getName());
        if (find != nodeBodies.end())
        {
            // Parent exists.
            parent = find->second;
        }
        else
        {
            // Parent does not exist => create it first.
            parent = addNodeBody(robot->getRobotNode(node->getParent()->getName()),
                                 addJoint, addInertial);
        }
    }
    
    // Add body as child of parent.
    mjcf::Body body = addNodeBody(node, parent, addJoint, addInertial);
    nodeBodies[node->getName()] = body;

    return body;
}

void RobotMjcf::addNodeBodies()
{
    addNodeBodies(robot->getRobotNodeNames());
}

void RobotMjcf::addNodeBodies(RobotNodeSetPtr nodeSet)
{
    addNodeBodies(nodeSet->getNodeNames());
}


void RobotMjcf::addNodeBodies(const std::vector<std::string>& nodeNames)
{
    for (const std::string& nodeName : nodeNames)
    {
        addNodeBody(robot->getRobotNode(nodeName));
    }
}

void RobotMjcf::addNodeBodyMesh(RobotNodePtr node)
{
    mjcf::Body body = getRobotNodeBody(node->getName());
    
    const bool meshlabserverAviable = true; // system("which meshlabserver > /dev/null 2>&1") == 0;
    bool notAvailableReported = false;
    
    
}

void RobotMjcf::convertNodeMeshToSTL(RobotNodePtr node)
{
    
}

void RobotMjcf::convertNodeMeshToSTL(
        const std::filesystem::path& sourceFile,
        const std::filesystem::path& _targetPath,
        bool skipIfExists)
{
    fs::path targetFile = _targetPath;
    
    VR_ASSERT();
    
    // Add a file name if none was passed.
    if (!targetFile.has_filename())
    {
        fs::path filename = sourceFile.filename();
        filename.replace_extension("stl");
        targetFile = targetFile / filename;
    }
    
    // Make sure parent directory exists.
    if (!fs::exists(targetFile.parent_path()))
    {
        fs::create_directories(targetFile.parent_path());
    }
    
    // Check if file already exists.
    if (skipIfExists && fs::exists(targetFile))
    {
        std::cout << "skipping (" << targetFile << " already exists)";
        return;
    }
    
    // Check if file has to be converted.
    if (sourceFile.extension() != ".stl")
    {
        std::cout << "Copying: " << sourceFile << "\n"
                  << "     to: " << targetFile;
        fs::copy_file(sourceFile, targetFile);
        
        return;
    }
    
    
    std::cout << "Converting to .stl: " << sourceFile << std::endl;
    
    const bool meshlabserverAvailable = system("which meshlabserver > /dev/null 2>&1") == 0;
    bool notAvailableReported = false;
    
    if (!meshlabserverAvailable)
    {
        if (!notAvailableReported)
        {
            std::cerr << std::endl 
                      << "Command 'meshlabserver' not available, cannot convert meshes."
                      << " (This error is reported only once.)"
                      << std::endl;
            notAvailableReported = true;
        }
        
        return;
    }
    
    // meshlabserver available
    std::stringstream convertCommand;
    convertCommand << "meshlabserver"
                   << " -i " << sourceFile
                   << " -o " << targetFile;
    
    // run command
    std::cout << "----------------------------------------------------------" << std::endl;
    std::cout << "Running command: " << convertCommand.str() << std::endl;
    const int r = system(convertCommand.str().c_str());
    std::cout << "----------------------------------------------------------" << std::endl;
    if (r != 0)
    {
        std::cout << "Command returned with error: " << r << "\n"
                  << "Command was: " << convertCommand.str() << std::endl;
    }
}

void RobotMjcf::addNodeBodyMeshes()
{
    addNodeBodyMeshes(robot->getRobotNodeNames());
}

void RobotMjcf::addNodeBodyMeshes(RobotNodeSetPtr nodeSet)
{
    addNodeBodyMeshes(nodeSet->getNodeNames());
}

void RobotMjcf::addNodeBodyMeshes(const std::vector<std::string>& nodeNames)
{
    for (const std::string& nodeName : nodeNames)
    {
        addNodeBody(robot->getRobotNode(nodeName));
    }    
}

}
