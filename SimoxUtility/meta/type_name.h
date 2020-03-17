#pragma once

#include <string>
#include <typeinfo>

#include <boost/core/demangle.hpp>


namespace simox::meta
{

    /// Return the demangled type name in `typeInfo`.
    inline std::string get_type_name(const std::type_info& typeInfo)
    {
        return boost::core::demangle(typeInfo.name());
    }

    /// Return the demangled (static) type name of `t`.
    template <typename T>
    std::string get_type_name()
    {
        return get_type_name(typeid (T));
    }

    /// Return the demangled (dynamic) type name of `t`.
    template <typename T>
    std::string get_type_name(const T& t)
    {
        return get_type_name(typeid (t));
    }

}

