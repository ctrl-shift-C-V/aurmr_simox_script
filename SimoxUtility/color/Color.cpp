#include "Color.h"


namespace simox::color
{

    Color::Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a)
    {}

    Eigen::Matrix<uint8_t, 3, 1> Color::to_vector3b() const
    {
        return { r, g, b };
    }

    Eigen::Matrix<uint8_t, 4, 1> Color::to_vector4b() const
    {
        return { r, g, b, a};
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
