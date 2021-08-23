#pragma once

#include <cmath>

namespace simox::math
{
    template<class FloatT>
    std::enable_if_t<std::is_floating_point_v<FloatT>, FloatT>
    rad_to_deg(FloatT rad)
    {
        return rad * 180.f / static_cast<FloatT>(M_PI);
    }

    template<class FloatT>
    std::enable_if_t<std::is_floating_point_v<FloatT>, Eigen::Matrix<FloatT, 3, 1> >
    rad_to_deg(Eigen::Ref<const Eigen::Matrix<FloatT, 3, 1>> rad)
    {
        return {rad_to_deg(rad(0)), rad_to_deg(rad(1)), rad_to_deg(rad(2))};
    }
}
