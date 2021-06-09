/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility/shapes/OrientedBoxTest

#include <random>
#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <filesystem>

#include <SimoxUtility/shapes/json_conversions.h>
#include <SimoxUtility/shapes/OrientedBox.h>
#include <SimoxUtility/shapes/AxisAlignedBoundingBox.h>
#include <SimoxUtility/json/io.h>


#include <SimoxUtility/json/util.h>
#include <SimoxUtility/json/eigen_conversion.h>


namespace
{
    static const std::vector<std::string> JSON_AABB_FILENAMES = {
        "AABB_center_extents.json",
        "AABB_min_max.json",
    };
    static const std::vector<std::string> JSON_ORIENTED_BOX_FILENAMES = {
        "OrientedBox_position_orientation_extents.json",
        "OrientedBox_pos_ori_dimensions.json",
    };

    const float PRECF = 1e-3f;
    const double PRECD = 1e-3;
}

#define BOOST_CHECK_EQUAL_VECTOR(lhs, rhs, prec) \
    BOOST_TEST_INFO((lhs).transpose() << " == " << (rhs).transpose()); \
    BOOST_CHECK_LE(((lhs) - (rhs)).norm(), prec);


namespace
{
    template <typename Float>
    void test_read_OrientedBox_type(const Float prec)
    {
        namespace fs = std::filesystem;

        for (const std::string& filename : JSON_ORIENTED_BOX_FILENAMES)
        {
            BOOST_TEST_CONTEXT("File " << filename)
            {
                BOOST_REQUIRE(fs::is_regular_file(filename));

                const nlohmann::json j = nlohmann::read_json(filename);

                const simox::OrientedBox<float> ob = j.get<simox::OrientedBox<float>>();
                BOOST_CHECK_EQUAL_VECTOR(ob.center(), Eigen::Vector3f(10, -20, 0), prec);
                BOOST_CHECK_EQUAL_VECTOR(ob.rotation(), Eigen::Quaternionf(0.707f, 0, 0.707f, 0).toRotationMatrix(), prec);
                BOOST_CHECK_EQUAL_VECTOR(ob.dimensions(), Eigen::Vector3f(100, 200, 300), prec * Float(1e2));
            }
        }
    }
}


BOOST_AUTO_TEST_CASE(test_read_AABB)
{
    namespace fs = std::filesystem;
    float prec = 1e-4f;


    for (const std::string& filename : JSON_AABB_FILENAMES)
    {
        BOOST_TEST_CONTEXT("File " << filename)
        {
            BOOST_REQUIRE(fs::is_regular_file(filename));

            const nlohmann::json j = nlohmann::read_json(filename);

            const simox::AxisAlignedBoundingBox aabb = j.get<simox::AxisAlignedBoundingBox>();
            BOOST_CHECK_EQUAL_VECTOR(aabb.center(), Eigen::Vector3f(45, 90, 135), prec);
            BOOST_CHECK_EQUAL_VECTOR(aabb.extents(), Eigen::Vector3f(110, 220, 330), prec);
            BOOST_CHECK_EQUAL_VECTOR(aabb.min(), Eigen::Vector3f(-10, -20, -30), prec);
            BOOST_CHECK_EQUAL_VECTOR(aabb.max(), Eigen::Vector3f(100, 200, 300), prec);
        }
    }
}


BOOST_AUTO_TEST_CASE(test_read_OrientedBox_float)
{
    test_read_OrientedBox_type<float>(PRECF);
}

BOOST_AUTO_TEST_CASE(test_read_OrientedBox_double)
{
    test_read_OrientedBox_type<double>(PRECD);
}
