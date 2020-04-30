#include <VirtualRobot/RuntimeEnvironment.h>

#include "CoinViewer.h"

int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "CoinViewer");
    CoinViewerExample w;
    w.main();
    return 0;
}
