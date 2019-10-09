/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Raphael Grimm (raphael dot grimm at kit dot edu)
 * @copyright  2019 Raphael Grimm
 *             GNU Lesser General Public License
 */

#pragma once

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VirtualRobot
{
    template<class FloatT>
    class OrientedBox
    {
    public:
        using float_t = FloatT;
        using vector_t = Eigen::Matrix<float_t, 3, 1>;
        using transform_t = Eigen::Matrix<float_t, 4, 4>;
        using rotation_t = Eigen::Matrix<float_t, 3, 3>;

    public:
        static constexpr float_t eps = static_cast<float_t>(1e8);
        static constexpr float_t pi  = static_cast<float_t>(M_PI);

    private:
        static auto translation(transform_t& t)
        {
            return t.template topRightCorner<3, 1>();
        }
        static auto translation(const transform_t& t)
        {
            return t.template topRightCorner<3, 1>();
        }

        static auto rotation(transform_t& t)
        {
            return t.template topLeftCorner<3, 3>();
        }
        static auto rotation(const transform_t& t)
        {
            return t.template topLeftCorner<3, 3>();
        }

        static transform_t transformation_identity()
        {
            return transform_t::Identity();
        }

        static transform_t transformation(const rotation_t& rot, const vector_t& trans)
        {
            transform_t t = transformation_identity();
            rotation(t) = rot;
            translation(t) = trans;
            return t;
        }
    public:
        OrientedBox(OrientedBox&&) = default;
        OrientedBox(const OrientedBox&) = default;
        OrientedBox& operator=(OrientedBox&&) = default;
        OrientedBox& operator=(const OrientedBox&) = default;

        OrientedBox(
            const vector_t& corner,
            const vector_t& extend0,
            const vector_t& extend1,
            const vector_t& extend2
        )
        {
            const float_t len0 = extend0.norm();
            const float_t len1 = extend1.norm();
            const float_t len2 = extend2.norm();
            const vector_t normalized0 = extend0 / len0;
            const vector_t normalized1 = extend1 / len1;
            const vector_t normalized2 = extend2 / len2;

            const float_t dot01 = normalized0.dot(normalized1);
            const float_t dot02 = normalized0.dot(normalized2);
            const float_t dot12 = normalized0.dot(normalized1);

            const float_t angle01 = std::acos(dot01) * 180 / pi;
            const float_t angle02 = std::acos(dot02) * 180 / pi;
            const float_t angle12 = std::acos(dot12) * 180 / pi;

            //checks
            if (std::abs(angle01) > eps)
            {
                throw std::invalid_argument
                {
                    "extend0 and extend1 are not perpendicular! (angle = " +
                    std::to_string(angle01) + "°)"
                };
            }
            if (std::abs(angle02) > eps)
            {
                throw std::invalid_argument
                {
                    "extend0 and extend2 are not perpendicular! (angle = " +
                    std::to_string(angle02) + "°)"
                };
            }
            if (std::abs(angle12) > eps)
            {
                throw std::invalid_argument
                {
                    "extend1 and extend2 are not perpendicular! (angle = " +
                    std::to_string(angle12) + "°)"
                };
            }

            //build transform
            _t = transform_t::Identity();
            _t.template block<3, 1>(0, 0) = normalized0;
            _d(0) = len0;

            const vector_t cross01 = normalized0.cross(normalized1);
            const float_t direction_match = cross01.dot(normalized2);

            if (direction_match > 0)
            {
                _t.template block<3, 1>(0, 1) = normalized1;
                _t.template block<3, 1>(0, 2) = normalized2;
                _d(1) = len1;
                _d(2) = len2;
            }
            else
            {
                _t.template block<3, 1>(0, 1) = normalized2;
                _t.template block<3, 1>(0, 2) = normalized1;
                _d(1) = len2;
                _d(2) = len1;
            }
            _t.template block<3, 1>(0, 3) = corner;
        }



        const vector_t& dimensions() const
        {
            return _d;
        }
        const transform_t& transformation() const
        {
            return _t;
        }
        transform_t transformation_centered() const
        {
            return transformation(rotation(), center());
        }

        auto translation() const
        {
            return translation(_t);
        }
        auto rotation() const
        {
            return rotation(_t);
        }

        auto axis_x() const
        {
            return _t.template block<3, 1>(0, 0);
        }
        auto axis_y() const
        {
            return _t.template block<3, 1>(0, 1);
        }
        auto axis_z() const
        {
            return _t.template block<3, 1>(0, 2);
        }

        float_t volume() const
        {
            return _d(0) * _d(1) * _d(2);
        }

        void scale(float_t factor)
        {
            _d *= factor;
        }
        void scale_centered(float_t factor)
        {
            translation(_t) += rotation() * _d * (1 - factor) / 2;
            _d *= factor;
        }

        vector_t from_box_frame(const vector_t& p) const
        {
            return translation() + rotation() * p;
        }

        vector_t to_box_frame(const vector_t& p) const
        {
            return -rotation().transpose() * translation() + rotation().transpose() * p;
        }

        bool contains(const vector_t& p)
        {
            const vector_t b = to_box_frame(p);
            return (_d(0) < 0 ? b(0) >= _d(0) : b(0) <= _d(0)) &&
                   (_d(1) < 0 ? b(1) >= _d(1) : b(1) <= _d(1)) &&
                   (_d(2) < 0 ? b(2) >= _d(2) : b(2) <= _d(2));
        }

        vector_t center() const
        {
            return from_box_frame(_d / 2);
        }

    private:
        transform_t _t;
        vector_t _d;
    };
}

