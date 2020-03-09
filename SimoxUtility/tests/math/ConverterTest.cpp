/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_ConverterTest

#include <random>
#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/math/convert.h>
#include <SimoxUtility/math/distance/delta_angle.h>

BOOST_AUTO_TEST_CASE(test_convert_oris)
{
    static constexpr double eps = 1e-5;

    for (int i = 0; i < 50; ++i)
    {
        const Eigen::Quaternionf in = Eigen::Quaternionf::UnitRandom();

        const auto check = [&](const auto & o)
        {
            BOOST_CHECK_LT(simox::math::delta_angle(in, o), eps);
        };

        //aa
        {
            check(simox::math::quat_to_aa(in));

            Eigen::AngleAxisf v;
            simox::math::quat_to_aa(in, v);
            check(v);

            check(simox::math::aa_to_quat(v));
            check(simox::math::aa_to_mat3f(v));
            check(simox::math::aa_to_mat4f(v));
            check(simox::math::rpy_to_aa(simox::math::aa_to_rpy(v)));
        }

        //mat3f
        {
            check(simox::math::quat_to_mat3f(in));

            Eigen::Matrix3f v;
            simox::math::quat_to_mat3f(in, v);
            check(v);

            check(simox::math::mat3f_to_aa(v));
            check(simox::math::mat3f_to_quat(v));
            check(simox::math::mat3f_to_mat4f(v));
            check(simox::math::rpy_to_mat3f(simox::math::mat3f_to_rpy(v)));
        }

        //mat4f
        {
            check(simox::math::quat_to_mat4f(in));

            Eigen::Matrix4f v;
            simox::math::quat_to_mat4f(in, v);
            check(v);

            check(simox::math::mat4f_to_aa(v));
            check(simox::math::mat4f_to_quat(v));
            check(simox::math::mat4f_to_mat3f(v));
            check(simox::math::rpy_to_mat4f(simox::math::mat4f_to_rpy(v)));
        }

        //rpy
        {
            check(simox::math::rpy_to_aa(simox::math::quat_to_rpy(in)));

            Eigen::Vector3f v;
            simox::math::quat_to_rpy(in, v);

            check(simox::math::rpy_to_aa(v));
            check(simox::math::rpy_to_quat(v));
            check(simox::math::rpy_to_mat3f(v));
            check(simox::math::rpy_to_mat4f(v));
        }
    }
}
