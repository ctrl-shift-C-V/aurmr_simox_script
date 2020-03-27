#include "interpolation.h"


namespace simox::color
{

    Color interpol::linear(float t, const Color& lhs, const Color& rhs)
    {
        return Color(((1 - t) * lhs.to_vector4f() + t * rhs.to_vector4f()).eval());
    }

}
