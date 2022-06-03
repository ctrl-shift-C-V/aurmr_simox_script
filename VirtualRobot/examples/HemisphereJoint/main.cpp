#include "Expressions.h"

#include <iostream>

#include <SimoxUtility/math/convert/deg_to_rad.h>

#include <VirtualRobot/RuntimeEnvironment.h>


using VirtualRobot::RuntimeEnvironment;


/**
 *
 */
int main(int argc, char* argv[])
{
    RuntimeEnvironment::setCaption("Convert .iv to .wrl files");

#if 0
    RuntimeEnvironment::considerKey(
                "input", ".iv file containing the (textured) mesh");
#endif

    RuntimeEnvironment::processCommandLine(argc, argv);

    if (RuntimeEnvironment::hasHelpFlag()
            // || !RuntimeEnvironment::hasValue("input")
            //|| !RuntimeEnvironment::hasValue("output"))
            )
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }

    Expressions expr;
    double lever = 1, theta0 = simox::math::deg_to_rad(25.);
    double a1 = 0.0, a2 = 0.0;
    expr.compute(a1, a2, lever, theta0);

    Eigen::Vector3d pos {expr.ex, expr.ey, expr.ez};


    std::cout << "(lever, theta0) = (" << lever << ", " << theta0 << ") " << std::endl;
    std::cout << "(a1, a2) = (" << a1 << ", " << a2 << ") " << std::endl;

    std::cout << "pos = (" << pos.transpose() << ")" << std::endl;

    return 0;
}
