#pragma once

#include <boost/math/constants/constants.hpp>

namespace simox
{
    template<class FloatT>
    FloatT deg2rad(FloatT rad)
    {
        return rad / 180.f * boost::math::constants::pi<FloatT>();
    }
}
