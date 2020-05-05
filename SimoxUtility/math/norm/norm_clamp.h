#pragma once


// Eigen
#include <Eigen/Core>

// Simox
#include <SimoxUtility/math/norm/norm_max.h>
#include <SimoxUtility/math/norm/norm_min.h>


namespace simox::math
{

    template <int rows>
    Eigen::Matrix<float, rows, 1>
    norm_clamp(const Eigen::Matrix<float, rows, 1>& v, const float min, const float max)
    {
        if (v.norm() < min)
        {
            return v.normalized() * min;
        }
        else if (v.norm() > max)
        {
            return v.normalized() * max;
        }

        return v;
    }

}
