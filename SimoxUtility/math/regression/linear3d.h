#pragma once

#include <Eigen/Core>


namespace simox::math
{

    /**
     * @brief A linear regression model of the form a + b * x = y,
     * or per dimension, a_i + b_i * x_i = y_i.
     *
     * - x is the scalar input variable (e.g. time)
     * - y is 3D vector output variable (e.g. position)
     * - a is a 3D bias vector
     * - b is a 3D coefficient vector
     *
     * In matrix notation, the underlying equation system is:
     *
     * [[ a_0  b_0 ]    [  1    1    1  ]   [ y_0 ]
     *  [ a_1  b_1 ]  * [ x_0  x_1  x_2 ] = [ y_1 ]
     *  [ a_2  b_2 ]]                       [ y_2 ]
     */
    class LinearRegression3D
    {
    public:

        /**
         * The coefficients of the bias term and input variable x
         * [[ a_0  b_0 ]
         *  [ a_1  b_1 ]
         *  [ a_2  b_2 ]]
         */
        Eigen::Matrix<double, 3, 2> coefficients;


        /**
         * @brief Fit a linear regression model to the given data.
         * @param xs The input variables.
         * @param ys The output variables.
         * @return The regression model.
         */
        static LinearRegression3D
        Fit(const std::vector<double>& xs, const std::vector<Eigen::Vector3d>& ys);

        /**
         * @brief Predict the output variable of the given input variable.
         * @param x The input variable.
         * @return The predicted output variable.
         */
        Eigen::Vector3d predict(double x) const;

    };


    std::ostream& operator<<(std::ostream& os, const LinearRegression3D& r);

}
