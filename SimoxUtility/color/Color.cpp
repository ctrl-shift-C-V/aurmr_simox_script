#include "Color.h"


namespace simox::color
{

    Color::Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a)
    {}

    Eigen::Vector3i Color::to_vector3i() const
    {
        return { r, g, b };
    }

    Eigen::Vector4i Color::to_vector4i() const
    {
        return { r, g, b, a };
    }

    Eigen::Vector3f Color::to_vector3f() const
    {
        return { to_float(r), to_float(g), to_float(b) };
    }

    Eigen::Vector4f Color::to_vector4f() const
    {
        return { to_float(r), to_float(g), to_float(b), to_float(a) };
    }

}
