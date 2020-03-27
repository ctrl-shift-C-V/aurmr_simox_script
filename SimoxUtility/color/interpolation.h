#pragma once

#include "Color.h"


namespace simox::color::interpol
{

    Color linear(float t, const Color& lhs, const Color& rhs);

}


