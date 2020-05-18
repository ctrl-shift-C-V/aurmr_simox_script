#pragma once

#include <Eigen/Core>
#include <cmath>

namespace simox::math
{
    using std::is_finite;

    template<class Derived>
    bool is_finite(const Eigen::MatrixBase<Derived>& mat)
    {
        for (int x = 0; x < mat.rows(); ++x)
        {
            for (int y = 0; y < mat.cols(); ++y)
            {
                if (!is_finite(mat(x, y)))
                {
                    return false;
                }
            }
        }
        return true;
    }
}
