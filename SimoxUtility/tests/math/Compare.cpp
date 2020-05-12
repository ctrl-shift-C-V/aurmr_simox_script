/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2020 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility/math/Compare

#include <random>
#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/compare.h>


BOOST_AUTO_TEST_CASE(test_normal_orth)
{
    using namespace simox::math;

    std::cout << "is_equal\n";
    BOOST_CHECK(!is_equal(-1, 5u));
    BOOST_CHECK(!is_equal(5u, -1));
    BOOST_CHECK(is_equal(5u, 5u));
    BOOST_CHECK(is_equal(-1, -1));
    BOOST_CHECK(is_equal(-1.f, -1));
    BOOST_CHECK(is_equal(-1, -1.));

    std::cout << "is_greater\n";
    BOOST_CHECK(!is_greater(-1, 5u));
    BOOST_CHECK(is_greater(5u, -1));
    BOOST_CHECK(!is_greater(5u, 5u));
    BOOST_CHECK(!is_greater(-1, -1));
    BOOST_CHECK(!is_greater(-1.f, -1));
    BOOST_CHECK(!is_greater(-1, -1.));

    std::cout << "is_greater_equal\n";
    BOOST_CHECK(!is_greater_equal(-1, 5u));
    BOOST_CHECK(is_greater_equal(5u, -1));
    BOOST_CHECK(is_greater_equal(5u, 5u));
    BOOST_CHECK(is_greater_equal(-1, -1));

    std::cout << "is_inequal\n";
    BOOST_CHECK(is_inequal(-1, 5u));
    BOOST_CHECK(is_inequal(5u, -1));
    BOOST_CHECK(!is_inequal(5u, 5u));
    BOOST_CHECK(!is_inequal(-1, -1));
    BOOST_CHECK(!is_inequal(-1.f, -1));
    BOOST_CHECK(!is_inequal(-1, -1.));

    std::cout << "is_less\n";
    BOOST_CHECK(is_less(-1, 5u));
    BOOST_CHECK(!is_less(5u, -1));
    BOOST_CHECK(!is_less(5u, 5u));
    BOOST_CHECK(!is_less(-1, -1));
    BOOST_CHECK(!is_less(-1.f, -1));
    BOOST_CHECK(!is_less(-1, -1.));

    std::cout << "is_less_equal\n";
    BOOST_CHECK(is_less_equal(-1, 5u));
    BOOST_CHECK(!is_less_equal(5u, -1));
    BOOST_CHECK(is_less_equal(5u, 5u));
    BOOST_CHECK(is_less_equal(-1, -1));
    BOOST_CHECK(is_less_equal(-1.f, -1));
    BOOST_CHECK(is_less_equal(-1, -1.));

    std::cout << "value_in_limits_of_type\n";
    BOOST_CHECK(value_in_limits_of_type<int>(-1));
    BOOST_CHECK(value_in_limits_of_type<int>(5u));
    BOOST_CHECK(value_in_limits_of_type<int>(555u));

    BOOST_CHECK(!value_in_limits_of_type<std::uint8_t>(-1));
    BOOST_CHECK(value_in_limits_of_type<std::uint8_t>(5u));
    BOOST_CHECK(!value_in_limits_of_type<std::uint8_t>(555u));

    BOOST_CHECK(value_in_limits_of_type<float>(-1));
    BOOST_CHECK(value_in_limits_of_type<float>(5u));
    BOOST_CHECK(value_in_limits_of_type<float>(555u));
}
