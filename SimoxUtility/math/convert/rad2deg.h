#pragma once

#include <boost/math/constants/constants.hpp>

namespace simox::math
{
    template<class FloatT>
    FloatT rad2deg(FloatT rad)
    {
        return rad * 180.f / boost::math::constants::pi<FloatT>();
    }
}
