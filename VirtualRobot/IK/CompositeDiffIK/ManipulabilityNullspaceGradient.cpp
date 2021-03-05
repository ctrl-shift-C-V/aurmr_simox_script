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
* @author     André Meixner (andre dot meixner at kit dot edu)
* @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
*             GNU General Public License
*/

#include "ManipulabilityNullspaceGradient.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/math/Helpers.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <cfloat>

namespace VirtualRobot
{

NullspaceManipulability::NullspaceManipulability(AbstractManipulabilityTrackingPtr manipulabilityTracking, float k_gain, const Eigen::MatrixXd& manipulabilityDesired) :
    CompositeDiffIK::NullspaceGradient(manipulabilityTracking->getJointNames()),
    manipulabilityTracking(manipulabilityTracking),
    k_gain(k_gain),
    manipulabilityDesired(manipulabilityDesired)
{
}

double NullspaceManipulability::computeDistance() {
    return manipulabilityTracking->computeDistance(manipulabilityDesired);
}

void NullspaceManipulability::init(CompositeDiffIK::Parameters&)
{
    // Does not need any initial parameters
}

Eigen::VectorXf NullspaceManipulability::getGradient(CompositeDiffIK::Parameters& /*params*/, int /*stepNr*/)
{
    Eigen::VectorXf velocities = manipulabilityTracking->calculateVelocity(manipulabilityDesired);
    // check if nan
    unsigned int nan = 0;
    for (unsigned int i = 0; i < velocities.rows(); i++) {
        if (std::isnan(velocities(i))) {
            velocities(i) = 0;
            nan++;
        }
    }
    if (nan > 0)
        std::cout << "Nan in nullspace manipulability velocities: " << nan << "/" << velocities.rows() << " \n";
    return k_gain * velocities;
}

}
