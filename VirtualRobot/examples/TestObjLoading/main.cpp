#include <filesystem>

#include <VirtualRobot/RuntimeEnvironment.h>

#include <VirtualRobot/XML/ObjectIO.h>

#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSeparator.h>

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/SoOutput.h>
#include <Inventor/SoInteraction.h>

#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/actions/SoWriteAction.h>

#include <Inventor/VRMLnodes/SoVRMLGroup.h>

#include <VirtualRobot/Import/MeshImport/AssimpReader.h>


using namespace VirtualRobot;
using VirtualRobot::RuntimeEnvironment;

namespace fs = std::filesystem;


/**
 * Loads a Simox robot and converts it to Mujoco's XML format (MJCF).
 * The converted file is stored in a directory mjcf/ placed in the directory
 * of the input file.
 */
int main(int argc, char* argv[])
{
    SoDB::init();
    SoInteraction::init();

    RuntimeEnvironment::setCaption("Test .obj mesh loading with textures");

    RuntimeEnvironment::considerKey(
                "input", ".obj file containing the (textured) mesh");

    RuntimeEnvironment::processCommandLine(argc, argv);


    if (RuntimeEnvironment::hasHelpFlag())
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }

    std::string input;
    if (RuntimeEnvironment::hasValue("input"))
    {
        input = RuntimeEnvironment::getValue("input");
    }
    else
    {
        //input = "/home/paus/code/h2t/PriorKnowledgeData/data/PriorKnowledgeData/objects/KIT/Amicelli/Amicelli.obj";
        input = "/home/paus/code/h2t/PriorKnowledgeData/data/PriorKnowledgeData/objects/Maintenance/workbench/workbench.x3d";
    }

    fs::path inputFilename;
    if (RuntimeEnvironment::getDataFileAbsolute(input))
    {
        inputFilename = input;
    }
    else
    {
        std::cout << "Something is wrong with " << input;
    }

    SoNode* node = AssimpReader::readFileAsSoNode(inputFilename.string());
    if (node)
    {
        std::cout << "Successfully loaded file\n";
        node->unref();
    }
    else
    {
        std::cout << "Could not load file: " << inputFilename << "\n";
    }

    return 0;
}
