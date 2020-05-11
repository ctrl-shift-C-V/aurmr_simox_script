#pragma once

#include <boost/preprocessor/variadic/to_seq.hpp>
#include <boost/preprocessor/seq/for_each.hpp>

#define SIMOX_VARIADIC_FOR_EACH(macro, data, ...) \
    BOOST_PP_SEQ_FOR_EACH(macro, data, BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))
