#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Werror"

#warning "This header is deprecated. Use <SimoxUtility/json/converters.h> instead."

#pragma GCC diagnostic pop

// Legacy overloads redirecting to Simox/math/json
// Do not add functions here, but add them to <SimoxUtility/json/converters.h>.


#include <SimoxUtility/json/converters.h>

namespace VirtualRobot
{
    namespace json = simox::json;
}
