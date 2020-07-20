#pragma once

namespace simox::math
{
    template <class T>
    inline constexpr T
    scale_value_from_to(T val, T from_lo, T from_hi, T to_lo, T to_hi)
    {
        return to_lo + (val - from_lo) / (from_hi - from_lo) * (to_hi - to_lo);
    }

}
