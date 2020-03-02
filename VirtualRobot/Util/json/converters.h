#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Werror"

#warning "This header is deprecated. Use <SimoxUtility/json/converters.h> instead."

#pragma GCC diagnostic pop

#include <SimoxUtility/json/converters.h>


namespace VirtualRobot
{
    // Redirect namespace.
    namespace json = simox::json;
}
