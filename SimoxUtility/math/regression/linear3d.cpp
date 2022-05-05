#include "linear3d.h"

#include <Eigen/Dense>


namespace simox::math
{

    LinearRegression3D
    LinearRegression3D::Fit(
            const std::vector<double>& xs,
            const std::vector<Eigen::Vector3d>& ys,
            bool offsetInput)
    {
        if (offsetInput and xs.at(0) != 0)
        {
            double offset = - xs.at(0);  // Move x_0 to 0.
            std::vector<double> virtualXs = xs;
            for (double& x : virtualXs)
            {
                x = x + offset;
            }
            LinearRegression3D r = LinearRegression3D::Fit(virtualXs, ys, false);
            r.inputOffset = offset;
            return r;
        }

        Eigen::Matrix3Xd ysMatrix(3, ys.size());
        for (long col = 0; col < ysMatrix.cols(); ++col)
        {
            ysMatrix.col(col) = ys.at(col);
        }

        // The matrix of the predictor functions evaluated at the corresponding xs.
        // Since this is a linear regression, the constant function a(t) = 1 and identity
        // b(t) = t are used.
        Eigen::MatrixX2d linFuncMatrix(xs.size(), 2);
        linFuncMatrix.col(0) = Eigen::RowVectorXd::Ones(xs.size());
        linFuncMatrix.col(1) = Eigen::Map<const Eigen::VectorXd>(xs.data(), xs.size());

        // `linFuncMatrix` is poorly conditioned for xs that are close together
        // (e.g. time stamps), so the normal equation would loose a lot of precision.
        auto qrDecomp = linFuncMatrix.colPivHouseholderQr();

        // Each coordinate can be treated individually (general multivariate regression).
        // `coeffs` contains a_i and b_i in a_0 + b_0 * t = x, a_1 + b_1 * t = y, etc.
        Eigen::Matrix<double, 3, 2> coeffs;
        for (int dim = 0; dim < 3; ++dim)
        {
            Eigen::VectorXd coords = ysMatrix.row(dim).transpose();
            coeffs.row(dim) = qrDecomp.solve(coords);
        }

        return LinearRegression3D { .coefficients = coeffs };
    }


    Eigen::Vector3d
    LinearRegression3D::predict(double x) const
    {
        Eigen::Vector2d input;
        input << 1.0, x + inputOffset;
        return coefficients * input;
    }

}


std::ostream&
simox::math::operator<<(std::ostream& os, const LinearRegression3D& r)
{
    os << "<LinearRegression3D a + b * x = y [ ";
    for (Eigen::Index row = 0; row < r.coefficients.rows(); ++row)
    {
        if (row != 0)
        {
            os << ", ";
        }
        os << r.coefficients(row, 0)
           << " + " << r.coefficients(row, 1) << " * x_" << row
           << " = y_" << row;
    }
    os << " ]";
    return os;
}
