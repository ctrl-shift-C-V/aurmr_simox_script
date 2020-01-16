#pragma once

#include <boost/math/constants/constants.hpp>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
    template<class FloatT>
    std::enable_if_t<std::is_floating_point_v<FloatT>, FloatT>
    deg_to_rad(FloatT deg)
    {
        return deg / 180.f * boost::math::constants::pi<FloatT>();
    }

    template<class FloatT>
    std::enable_if_t<std::is_floating_point_v<FloatT>, Eigen::Matrix<FloatT, 3, 1>>
    deg_to_rad(Eigen::Ref<const Eigen::Matrix<FloatT, 3, 1>> deg)
    {
        return {deg_to_rad(deg(0)), deg_to_rad(deg(1)), deg_to_rad(deg(2))};
    }
}
