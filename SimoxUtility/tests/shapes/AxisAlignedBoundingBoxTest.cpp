/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility/shapes/OrientedBoxTest

#include <random>
#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/shapes/AxisAlignedBoundingBox.h>


namespace
{
    struct PointT
    {
        float x = 0, y = 0, z = 0;
    };
}


BOOST_AUTO_TEST_CASE(test_AABB_from_points_eigen)
{
    std::vector<Eigen::Vector3f> points{ { 0, 0, 0 } , { -2, 0, 1 }, { 2, 0, -1 } };

    simox::AxisAlignedBoundingBox aabb = simox::aabb::from_points(points);

    BOOST_CHECK_EQUAL(aabb.min(), Eigen::Vector3f(-2, 0, -1));
    BOOST_CHECK_EQUAL(aabb.max(), Eigen::Vector3f( 2, 0,  1));
}

BOOST_AUTO_TEST_CASE(test_AABB_from_points_custom)
{
    std::vector<PointT> points{ { 0, 0, 0 } , { -2, 0, 1 }, { 2, 0, -1 } };

    simox::AxisAlignedBoundingBox aabb = simox::aabb::from_points(points);

    BOOST_CHECK_EQUAL(aabb.min(), Eigen::Vector3f(-2, 0, -1));
    BOOST_CHECK_EQUAL(aabb.max(), Eigen::Vector3f( 2, 0,  1));
}
