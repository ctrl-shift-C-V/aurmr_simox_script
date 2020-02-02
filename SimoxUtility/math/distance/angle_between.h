#pragma once

#include <cmath>

#include <Eigen/Geometry>

#include "../../meta/eigen/enable_if_compile_time_size.h"

namespace simox::math
{
//    template<class D1, class D2> inline
//    meta::enable_if_vec3_vec3<D1, D2, float>
//    angle_between_vec3f_vec3f(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
//    {
//        return std::acos(l.normalized().dot(r.normalized()));
//    }

    template<class D1, class D2> inline
    meta::enable_if_vec3_vec3<D1, D2, float>
    angle_between_vec3f_vec3f_abs(const Eigen::MatrixBase<D1>& l, const Eigen::MatrixBase<D2>& r)
    {
        return std::acos(l.normalized().dot(r.normalized()));
    }
}

