#pragma once

namespace simox::math
{
    inline constexpr auto
    scale_value_from_to(auto val,
                        auto from_lo, auto from_hi,
                        auto to_lo, auto to_hi)
    {
        return to_lo + (val - from_lo) / (from_hi - from_lo) * (to_hi - to_lo);
    }

}
