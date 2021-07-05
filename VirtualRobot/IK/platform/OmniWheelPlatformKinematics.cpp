/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Fabian Reister ( fabian dot reister at kit dot edu )
 * @date       2021
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include "OmniWheelPlatformKinematics.h"

namespace VirtualRobot
{
    // OmniWheelPlatformKinematicsParams

    Eigen::Matrix3f OmniWheelPlatformKinematicsParams::B() const
    {
        Eigen::Matrix3f b;

        // clang-format off
            b << 1.F, -std::sin(delta), -std::sin(delta), 
                 0.F,  std::cos(delta), -std::cos(delta), 
                 L, L, L;
        // clang-format on

        return b;
    }

    Eigen::Matrix3f OmniWheelPlatformKinematicsParams::C() const
    {
        return B().transpose().inverse() * R / n;
    }

    // OmniWheelPlatformKinematics

    OmniWheelPlatformKinematics::OmniWheelPlatformKinematics(const Params& params) :
        params(params), C(params.C()), C_inv(C.inverse())
    {
    }

    OmniWheelPlatformKinematics::WheelVelocities
    OmniWheelPlatformKinematics::calcWheelVelocity(const CartesianVelocity& v)
    {
        return C * v;
    }

    OmniWheelPlatformKinematics::CartesianVelocity
    OmniWheelPlatformKinematics::calcCartesianVelocity(const WheelVelocities& w)
    {
        return C_inv * w;
    }

    const OmniWheelPlatformKinematics::Params& OmniWheelPlatformKinematics::getParams() const
    {
        return params;
    }

} // namespace VirtualRobot
