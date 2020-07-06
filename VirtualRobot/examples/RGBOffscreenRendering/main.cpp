#include <VirtualRobot/RuntimeEnvironment.h>

#include "RGBOffscreenRendering.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "RGBOffscreenRendering");
    RGBOffscreenRenderingExample w;
    w.main();
    return 0;
}
