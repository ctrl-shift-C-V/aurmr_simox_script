#include "color.h"


namespace simox
{

    color::Color color::to_byte(Colorf c)
    {
        return (c.cwiseMin(1.0).cwiseMax(0.0) * 255).cast<uint8_t>();
    }

    float color::to_float(uint8_t value)
    {
        return value / 255.f;
    }

    color::Colorf color::to_float(Color color)
    {
        return color.cast<float>() / 255.f;
    }

}






