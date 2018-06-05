/*
 * This file is part of ArmarX.
 * 
 * Copyright (C) 2012-2016, High Performance Humanoid Technologies (H2T),
 * Karlsruhe Institute of Technology (KIT), all rights reserved.
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
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include "Helpers.h"
#include "LinearInterpolatedOrientation.h"

using namespace math;

LinearInterpolatedOrientation::LinearInterpolatedOrientation(const Eigen::Quaternionf &startOri, const Eigen::Quaternionf &endOri, float startT, float endT, bool clamp)
    :startOri(startOri), endOri(endOri), startT(startT), endT(endT), clamp(clamp)
{
    Eigen::AngleAxisf aa(endOri * startOri.inverse());
    float angle = Helpers::AngleModPI(aa.angle());
    Eigen::Vector3f oriDelta = fabs(angle) < 0.01 ? Eigen::Vector3f::Zero() : Eigen::Vector3f(aa.axis() * angle);

    derivative = oriDelta / (endT - startT);
}


Eigen::Quaternionf math::LinearInterpolatedOrientation::Get(float t)
{
    if(derivative.squaredNorm() < 0.001)
    {
        return startOri;
    }
    float f = Helpers::ILerp(startT, endT, t);
    if(clamp)
    {
        f = Helpers::Clamp(f, 0, 1);
    }
    Helpers::Lerp(startOri, endOri, f);
}

Eigen::Vector3f math::LinearInterpolatedOrientation::GetDerivative(float t)
{
    if(clamp && (t < startT || t > endT))
    {
        return Eigen::Vector3f::Zero();
    }
    return derivative;

}
