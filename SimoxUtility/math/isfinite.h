#pragma once

#include <Eigen/Core>
#include <cmath>


namespace simox::math
{
    using std::isfinite;

    template <class Derived>
    bool isfinite(const Eigen::MatrixBase<Derived>& mat)
    {
        for (int x = 0; x < mat.rows(); ++x)
        {
            for (int y = 0; y < mat.cols(); ++y)
            {
                if (!std::isfinite(mat(x, y)))
                {
                    return false;
                }
            }
        }
        return true;
    }
}
