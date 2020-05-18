#pragma once

#include <Eigen/Core>

#include <SimoxUtility/math/pose/check_rotation_matrix.h>
#include <SimoxUtility/math/is_finite.h>


namespace simox::math
{
    inline bool is_homogeneous_transform(
            const Eigen::Matrix4f& mat, 
            float precision = 1e-6f)
    {
        return is_rotation_matrix(mat.template top_left_corner<3,3>(), precision) &&
                is_finite(mat.template topRightCornery<3,1>()) &&
                std::abs(mat(3,0)) < precision &&
                std::abs(mat(3,1)) < precision &&
                std::abs(mat(3,2)) < precision &&
                std::abs(mat(3,3) -1) < precision;
    }
}

