/**
* @package    SimoxUtility
* @author     Raphael Grimm
* @copyright  2019 Raphael Grimm
*/

#define BOOST_TEST_MODULE SimoxUtility_OrientedBoxTest

#include <random>
#include <iostream>

#include <boost/test/included/unit_test.hpp>

#include <SimoxUtility/shapes/OrientedBox.h>

BOOST_AUTO_TEST_CASE(test_OrientedBox)
{
    static constexpr double eps = 1e-5;

    std::mt19937 gen{std::random_device{}()};
    std::uniform_real_distribution<double> d{1e-4, 10};

    const auto dif_pos = [](const auto & a, const auto & b)
    {
        return (a - b).norm();
    };
    const auto dif_rot = [](const auto & a, const auto & b)
    {
        return Eigen::AngleAxisd{a.transpose()* b}.angle();
    };

    const auto check = [&](
                           const Eigen::Quaterniond & q,
                           const Eigen::Vector3d & corner,
                           const Eigen::Vector3d & dim)
    {
        const auto vol = std::abs(dim(0) * dim(1) * dim(2));
        const Eigen::Matrix3d rot = q.toRotationMatrix();
        const Eigen::Vector3d cent = corner + rot * 0.5 * dim;
        std::cout << "####################\n" << rot << '\n';
        const auto subt = [&](auto v1, auto v2, auto v3)
        {
            simox::OrientedBox<double> b{corner, v1, v2, v3};
            std::cout << "----\n" << b.transformation_centered().matrix() << '\n';
            BOOST_CHECK_LT(std::abs(b.volume() - vol), eps);
            BOOST_CHECK_LT(dif_rot(b.rotation(), rot), eps);
            BOOST_CHECK_LT(dif_pos(b.translation(), corner), eps);
            BOOST_CHECK_LT(dif_pos(b.center(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.dimensions(), dim), eps);
            BOOST_CHECK(b.contains(cent));
            BOOST_CHECK(b.contains_by(corner, -eps));

            const auto sc = d(gen);
            b.scale_centered(sc);
            std::cout << "----\n" << b.transformation_centered().matrix() << '\n';
            BOOST_CHECK_LT(std::abs(b.volume() - vol * sc * sc * sc), eps);
            BOOST_CHECK_LT(dif_rot(b.rotation(), rot), eps);
            BOOST_CHECK_LT(dif_pos(b.center(), cent), eps);
            BOOST_CHECK(b.contains(cent));
            BOOST_CHECK_LT(dif_pos(b.dimensions(), dim * sc), eps);

            b.scale_centered(0.0);
            std::cout << "----\n" << b.transformation_centered().matrix() << '\n';
            BOOST_CHECK_LT(std::abs(b.volume()), eps);
            BOOST_CHECK_LT(dif_rot(b.rotation(), rot), eps);
            BOOST_CHECK_LT(dif_pos(b.translation(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.center(), cent), eps);
            BOOST_CHECK_LT(dif_pos(b.dimensions(), Eigen::Vector3d::Zero()), eps);
        };
        subt(rot.col(0)* dim(0),
             rot.col(2)* dim(2),
             rot.col(1)* dim(1));
        subt(rot.col(0)* dim(0),
             rot.col(1)* dim(1),
             rot.col(2)* dim(2));
    };

    for (int i = 0; i < 100; ++i)
    {
        check(Eigen::Quaterniond::UnitRandom(),
              Eigen::Vector3d::Random() * 100,
              Eigen::Vector3d::Random().cwiseAbs() * 10);
    }
}
