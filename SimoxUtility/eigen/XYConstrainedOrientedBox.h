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

namespace simox
{
    template<class FloatT>
    class XYConstrainedOrientedBox
    {
    public:
        template<class T> using vector2_casted   = Eigen::Matrix<T, 2, 1>;
        template<class T> using vector_casted    = Eigen::Matrix<T, 3, 1>;
        template<class T> using transform_casted = Eigen::Matrix<T, 4, 4>;
        template<class T> using rotation_casted  = Eigen::Matrix<T, 3, 3>;

        using float_t = FloatT;
        using vector2_t   = vector2_casted<float_t>;
        using vector_t    = vector_casted<float_t>;
        using transform_t = transform_casted<float_t>;
        using rotation_t  = rotation_casted<float_t>;

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
        XYConstrainedOrientedBox(XYConstrainedOrientedBox&&) = default;
        XYConstrainedOrientedBox(const XYConstrainedOrientedBox&) = default;
        XYConstrainedOrientedBox& operator=(XYConstrainedOrientedBox&&) = default;
        XYConstrainedOrientedBox& operator=(const XYConstrainedOrientedBox&) = default;


        XYConstrainedOrientedBox(
            const vector_t& corner = {0, 0, 0},
            const float_t yaw = 0,
            const vector_t& dimensions = {0, 0, 0}
        ) :
            _t{transformation(
                   Eigen::AngleAxis<float_t>{yaw, vector_t::UnitZ()}.toRotationMatrix(),
        corner
           )},
           _d{dimensions},
           _yaw{yaw}
        {}

    private:

        static std::tuple<vector_t, float_t, vector_t> calc_params(
            const vector_t& corner,
            const vector2_t& extend0,
            const vector2_t& extend1,
            const float_t height
        )
        {
            const float_t len0 = extend0.norm();
            const float_t len1 = extend1.norm();

            const vector2_t normalized2d0 = extend0 / len0;
            const vector2_t normalized2d1 = extend1 / len1;

            const float_t dot01 = normalized2d0.dot(normalized2d1);
            const float_t angle01 = std::acos(dot01) * 180 / pi;

            //checks
            if (std::abs(angle01) > eps)
            {
                throw std::invalid_argument
                {
                    "extend0 and extend1 are not perpendicular! (angle = " +
                    std::to_string(angle01) + "°)"
                };
            }

            //make sure the system is right handed + calculate yaw
            const vector_t normalized0{normalized2d0(0), normalized2d0(1), 0};
            const vector_t normalized1{normalized2d1(0), normalized2d1(1), 0};

            const vector_t cross01 = normalized0.cross(normalized1);
            if (cross01(2) >= 0)
            {
                // x=0, y=1
                const float_t yaw = -std::atan2(normalized1(0), normalized1(1));
                return {corner, yaw, vector_t{len0, len1, height}};
            }

            // x=1, y=0
            const float_t yaw = -std::atan2(normalized0(0), normalized0(1));
            return {corner, yaw, vector_t{len1, len0, height}};
        }

        XYConstrainedOrientedBox(std::tuple<vector_t, float_t, vector_t> params) :
            XYConstrainedOrientedBox(std::get<0>(params), std::get<1>(params), std::get<2>(params))
        {}
    public:

        XYConstrainedOrientedBox(
            const vector_t& corner,
            const vector2_t& extend0,
            const vector2_t& extend1,
            const float_t height
        ) :
            XYConstrainedOrientedBox(calc_params(corner, extend0, extend1, height))
        {}

        template<class T>
        XYConstrainedOrientedBox<T> cast() const
        {
            return {translation<T>(), yaw<T>(0), dimensions<T>()};
        }

        const vector_t& dimensions() const
        {
            return _d;
        }
        template<class T>
        vector_casted<T> dimensions() const
        {
            return dimensions().template cast<T>();
        }

        float_t dimension(int i) const
        {
            return _d(i);
        }
        template<class T>
        T dimension(int i) const
        {
            return static_cast<T>(dimension(i));
        }

        const transform_t& transformation() const
        {
            return _t;
        }
        template<class T>
        transform_casted<T> transformation() const
        {
            return transformation().template cast<T>();
        }

        transform_t transformation_centered() const
        {
            return transformation(rotation(), center());
        }
        template<class T>
        transform_casted<T> transformation_centered() const
        {
            return transformation_centered().template cast<T>();
        }

        auto translation() const
        {
            return translation(_t);
        }
        template<class T>
        vector_casted<T> translation() const
        {
            return translation().template cast<T>();
        }
        auto rotation() const
        {
            return rotation(_t);
        }
        template<class T>
        rotation_casted<T> rotation() const
        {
            return rotation().template cast<T>();
        }

        auto axis_x() const
        {
            return _t.template block<3, 1>(0, 0);
        }
        template<class T>
        vector_casted<T> axis_x() const
        {
            return axis_x().template cast<T>();
        }

        auto axis_y() const
        {
            return _t.template block<3, 1>(0, 1);
        }
        template<class T>
        vector_casted<T> axis_y() const
        {
            return axis_y().template cast<T>();
        }

        auto axis_z() const
        {
            return _t.template block<3, 1>(0, 2);
        }
        template<class T>
        vector_casted<T> axis_z() const
        {
            return axis_z().template cast<T>();
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
        template<class T>
        vector_casted<T> from_box_frame(const vector_t& p) const
        {
            return from_box_frame(p).template cast<T>();
        }

        vector_t to_box_frame(const vector_t& p) const
        {
            return -rotation().transpose() * translation() + rotation().transpose() * p;
        }
        template<class T>
        vector_casted<T> to_box_frame(const vector_t& p) const
        {
            return from_box_frame(p).template cast<T>();
        }

        bool contains(const vector_t& p)
        {
            const vector_t b = to_box_frame(p);
            const auto check_dim = [&](int i)
            {
                return _d(i) < 0 ?
                       (b(i) <= 0 && b(i) >= _d(i)) :
                       (b(i) >= 0 && b(i) <= _d(i));
            };
            return check_dim(0) && check_dim(1) && check_dim(2);
        }

        bool contains_by(const vector_t& p, float_t thresh = 0)
        {
            const vector_t b = to_box_frame(p);
            const auto check_dim = [&](int i)
            {
                return _d(i) < 0 ?
                       (b(i) <= -thresh && b(i) >= _d(i) + thresh) :
                       (b(i) >= +thresh && b(i) <= _d(i) - thresh);
            };
            return check_dim(0) && check_dim(1) && check_dim(2);
        }

        template<class T, class...Other>
        std::vector<Eigen::Matrix<T, 3, 1>, Other...> contained_points(const std::vector<Eigen::Matrix<T, 3, 1>, Other...>& ps)
        {
            std::vector<Eigen::Matrix<T, 3, 1>, Other...> filtered;
            for (const auto& p : ps)
            {
                if (contains(p))
                {
                    filtered.emplace_back(p);
                }
            }
            return filtered;
        }

        vector_t center() const
        {
            return from_box_frame(_d / 2);
        }
        template<class T>
        vector_casted<T> center() const
        {
            return center().template cast<T>();
        }

        float_t yaw() const
        {
            return _yaw;
        }
        template<class T>
        T yaw() const
        {
            return _yaw;
        }
    private:
        transform_t _t;
        vector_t _d;
        float_t _yaw;
    };
}
