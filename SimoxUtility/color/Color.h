#pragma once

#include <Eigen/Core>

#include <type_traits>


namespace simox::color
{

    template <typename Int, std::enable_if_t<std::is_integral_v<Int>, int> = 0>
    uint8_t to_byte(Int value)
    {
        return static_cast<uint8_t>(std::clamp(value, static_cast<Int>(0), static_cast<Int>(255)));
    }

    template <typename Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    uint8_t to_byte(Float value)
    {
        return static_cast<uint8_t>(255 * std::clamp(value, static_cast<Float>(0.), static_cast<Float>(1.)));
    }


    template <typename Int, std::enable_if_t<std::is_integral_v<Int>, int> = 0>
    float to_float(Int value)
    {
        return to_byte(value) / 255.f;
    }

    template <typename Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
    float to_float(Float value)
    {
        return std::clamp(value, static_cast<Float>(0.), static_cast<Float>(1.));
    }


    struct Color
    {
        // Attributes.

        /// Red in [0, 255].
        uint8_t r = 0;
        /// Green in [0, 255].
        uint8_t g = 0;
        /// Blue in [0, 255].
        uint8_t b = 0;
        /// Alpha in [0, 255], i.e. opacity (0: transparent, 255: visible).
        uint8_t a = 255;


        // Constructors.

        /// Construct a black color.
        Color() = default;

        /// Construct a color from RGBA.
        Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255);

        /// Construct a color from integer RGBA.
        template <typename Int, std::enable_if_t<std::is_integral_v<Int>, int> = 0>
        Color(Int r, Int g, Int b, Int a = 255) : Color(to_byte(r), to_byte(g), to_byte(b), to_byte(a))
        {}
        /// Construct a color from float RGBA.
        template <typename Float, std::enable_if_t<std::is_floating_point_v<Float>, int> = 0>
        Color(Float r, Float g, Float b, Float a = 1.0) : Color(to_byte(r), to_byte(g), to_byte(b), to_byte(a))
        {}

        /// Construct a color from an RGB Eigen vector.
        template <typename T>
        Color(Eigen::Matrix<T, 3, 1> vector3) : Color(vector3(0), vector3(1), vector3(2))
        {}
        /// Construct a color from an RGBA Eigen vector.
        template <typename T>
        Color(Eigen::Matrix<T, 4, 1> vector4) : Color(vector4(0), vector4(1), vector4(2), vector4(3))
        {}


        // Converters.

        Eigen::Matrix<uint8_t, 3, 1> toVector3b() const;
        Eigen::Matrix<uint8_t, 4, 1> toVector4b() const;
        Eigen::Vector3f toVector3f() const;
        Eigen::Vector4f toVector4f() const;

    };

}
