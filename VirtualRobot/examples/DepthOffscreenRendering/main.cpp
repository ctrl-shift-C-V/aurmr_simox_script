#include <VirtualRobot/RuntimeEnvironment.h>

#include "DepthOffscreenRendering.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "DepthOffscreenRendering");
    DepthOffscreenRenderingExample w;
    w.main();
    return 0;
}
