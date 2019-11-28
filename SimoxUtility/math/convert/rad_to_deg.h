#pragma once

#include <boost/math/constants/constants.hpp>

namespace simox::math
{
    template<class FloatT>
    FloatT rad_to_deg(FloatT rad)
    {
        return rad * 180.f / boost::math::constants::pi<FloatT>();
    }

    template<class FloatT>
    Eigen::Matrix<FloatT, 3, 1> rad_to_deg(Eigen::Ref<const Eigen::Matrix<FloatT, 3, 1>> rad)
    {
        return {rad_to_deg(rad(0)), rad_to_deg(rad(1)), rad_to_deg(rad(2))};
    }
}
