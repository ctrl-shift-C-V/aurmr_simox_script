#pragma once

#include <boost/math/constants/constants.hpp>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template<class FloatT>
    FloatT deg_to_rad(FloatT rad)
    {
        return rad / 180.f * boost::math::constants::pi<FloatT>();
    }

    template<class FloatT>
    Eigen::Matrix<FloatT, 3, 1> deg_to_rad(Eigen::Ref<const Eigen::Matrix<FloatT, 3, 1>> rad)
    {
        return {deg_to_rad(rad(0)), deg_to_rad(rad(1)), deg_to_rad(rad(2))};
    }
}
