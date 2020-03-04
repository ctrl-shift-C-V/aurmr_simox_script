#pragma once

#include <type_traits>

#include <Eigen/Core>


namespace simox::color
{

    /// RGBA as bytes (0 - 255).
    using Color = Eigen::Matrix<uint8_t, 4, 1>;
    /// RGBA as floats (0.0 - 1.0).
    using Colorf = Eigen::Matrix<float, 4, 1>;


    template <typename T, std::enable_if_t<std::is_integral_v<T>, int> = 0>
    uint8_t to_byte(T value)
    {
        return static_cast<uint8_t>(std::clamp(value, 0, 255));
    }

    template <typename T, std::enable_if_t<std::is_floating_point_v<T>, int> = 0>
    uint8_t to_byte(T value)
    {
        return static_cast<uint8_t>(255 * std::clamp(value, static_cast<T>(0.), static_cast<T>(1.)));
    }

    Color to_byte(Colorf c);


    float to_float(uint8_t value);
    Colorf to_float(Color color);

}
