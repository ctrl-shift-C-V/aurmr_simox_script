#include "Expressions.h"

#include <chrono>
#include <iostream>

#include <SimoxUtility/math/convert/deg_to_rad.h>
#include <SimoxUtility/math/rescale.h>
#include <SimoxUtility/math/statistics/measures.h>

#include <VirtualRobot/RuntimeEnvironment.h>


using VirtualRobot::RuntimeEnvironment;


/**
 *
 */
int main(int argc, char* argv[])
{
    RuntimeEnvironment::setCaption("Convert .iv to .wrl files");

    RuntimeEnvironment::considerFlag(
                "verbose", "Enable verbose output.");

    RuntimeEnvironment::processCommandLine(argc, argv);

    if (RuntimeEnvironment::hasHelpFlag()
            // || !RuntimeEnvironment::hasValue("input")
            //|| !RuntimeEnvironment::hasValue("output"))
            )
    {
        RuntimeEnvironment::printOptions();
        return 0;
    }

    const bool verbose = RuntimeEnvironment::hasFlag("verbose");

    const double lever = 1, theta0 = simox::math::deg_to_rad(25.);
    std::cout << "(lever, theta0) = (" << lever << ", " << theta0 << ") " << std::endl;

    std::vector<double> a1s, a2s;
    int num = 100;
    double aMin = -0.7, aMax=0.7;
    for (int i = 0; i < num; ++i)
    {
        double value = simox::math::rescale(double(i), double(0), double(num), aMin, aMax);
        a1s.push_back(value);
        a2s.push_back(value);
    }


    using time_point = std::chrono::system_clock::time_point;

    std::vector<double> durationsUs;
    durationsUs.reserve(a1s.size() * a2s.size());

    for (double a1 : a1s)
    {
        for (double a2 : a2s)
        {
            const time_point start = std::chrono::system_clock::now();

            Expressions expr;
            expr.compute(a1, a2, lever, theta0);

            Eigen::Vector3d pos {expr.ex, expr.ey, expr.ez};

            // r_wrist_to_base = np.array([[exx, eyx, ezx], [exy, eyy, ezy], [exz, eyz, ezz]])
            Eigen::Matrix3d ori;
            ori << expr.exx, expr.eyx, expr.ezx,
                   expr.exy, expr.eyy, expr.ezy,
                   expr.exz, expr.eyz, expr.ezz;

            Eigen::Matrix<double, 6, 2> jacobian;
            jacobian << expr.jx1, expr.jx2,
                        expr.jy1, expr.jy2,
                        expr.jz1, expr.jz2,
                        expr.jrx1, expr.jrx2,
                        expr.jry1, expr.jry2,
                        expr.jrz1, expr.jrz2;

            const time_point end = std::chrono::system_clock::now();
            using duration = std::chrono::nanoseconds;
            durationsUs.push_back(std::chrono::duration_cast<duration>(end - start).count() / 1000.f);

            if (verbose)
            {
                std::cout << "(a1, a2) = (" << a1 << ", " << a2 << ") \t ->  "
                          << "pos = (" << pos.transpose() << ")" << std::endl;
            }
        }
    }

    double mean = simox::math::mean(durationsUs);
    double stddev = simox::math::stddev(durationsUs, mean);
    double min = simox::math::min(durationsUs);
    double max = simox::math::max(durationsUs);

    const std::string unit = " us";
    std::cout << "Durations:"
              << " " << mean << " +- " << stddev << unit
              << ", range: [" << min << unit << " to " << max << unit <<"]"
              << std::endl;

    return 0;
}
