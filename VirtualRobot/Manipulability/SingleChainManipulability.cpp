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
* @package    VirtualRobot
* @author     Andre Meixner
* @author     Noémie Jaquier
* @copyright  2021 Andre Meixner
*             GNU Lesser General Public License
*
*/
#include "SingleChainManipulability.h"

#include <Visualization/VisualizationFactory.h>
#include <Visualization/VisualizationNode.h>
#include <Eigen/Dense>
#include "../IK/DifferentialIK.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "VirtualRobot/Robot.h"
#include <SimoxUtility/math/convert/pos_quat_to_mat4f.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>


namespace VirtualRobot
{

Eigen::MatrixXd AbstractSingleChainManipulability::computeManipulability(const Eigen::MatrixXd &jacobian, AbstractManipulability::Type type) {
    Eigen::MatrixXd velocityManipulability = jacobian * jacobian.transpose();

    switch (type) {
    case Velocity:
        return velocityManipulability;
    case Force:
        return velocityManipulability.inverse();
    default:
        throw std::runtime_error("Unkown manipulability type");
    }
}




SingleRobotNodeSetManipulability::SingleRobotNodeSetManipulability(const RobotNodeSetPtr &rns, Mode mode, Type type, bool convertMMtoM)
    : SingleRobotNodeSetManipulability(rns, rns->getTCP(), mode, type, RobotNodePtr(), convertMMtoM)
{
}

SingleRobotNodeSetManipulability::SingleRobotNodeSetManipulability(const RobotNodeSetPtr &rns, const RobotNodePtr &node, Mode mode, Type type, const RobotNodePtr &coordSystem, bool convertMMtoM)
    : AbstractSingleChainManipulability(mode, type), rns(rns), node(node), coordSystem(coordSystem)
{
    ik.reset(new DifferentialIK(rns, coordSystem, JacobiProvider::eSVDDamped));
    ik->convertModelScalingtoM(convertMMtoM);
}

RobotNodeSetPtr SingleRobotNodeSetManipulability::getRobotNodeSet() {
    return rns;
}

RobotPtr SingleRobotNodeSetManipulability::getRobot() {
    return rns->getRobot();
}

void SingleRobotNodeSetManipulability::setConvertMMtoM(bool value) {
    ik->convertModelScalingtoM(value);
}

Eigen::Vector3f SingleRobotNodeSetManipulability::getLocalPosition() {
    return rns->getTCP()->getPositionInRootFrame();
}

Eigen::Vector3f SingleRobotNodeSetManipulability::getGlobalPosition() {
    return rns->getTCP()->getGlobalPosition();
}

std::vector<std::string> SingleRobotNodeSetManipulability::getJointNames() {
    return rns->getNodeNames();
}

Eigen::MatrixXd SingleRobotNodeSetManipulability::computeJacobian(IKSolver::CartesianSelection mode) {
    return ik->getJacobianMatrix(node, mode).cast<double>();
}




SingleChainManipulability::SingleChainManipulability(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian,
                                                     AbstractManipulability::Mode mode, AbstractManipulability::Type type,
                                                     const std::vector<std::string> &jointNames,
                                                     const Eigen::Vector3f &globalPosition, const Eigen::Vector3f &localPosition) :
    AbstractSingleChainManipulability(mode, type),
    jacobian(jacobian),
    jointNames(jointNames),
    globalPosition(globalPosition),
    localPosition(localPosition)
{
}

Eigen::Vector3f SingleChainManipulability::getLocalPosition() {
    return globalPosition;
}

Eigen::Vector3f SingleChainManipulability::getGlobalPosition() {
    return localPosition;
}

std::vector<std::string> SingleChainManipulability::getJointNames() {
    return jointNames;
}

void SingleChainManipulability::setJacobian(const Eigen::Matrix<double, 6, Eigen::Dynamic> &jacobian) {
    this->jacobian = jacobian;
}

void SingleChainManipulability::setLocalPosition(const Eigen::Vector3f &localPosition) {
    this->localPosition = localPosition;
}

void SingleChainManipulability::setGlobalPosition(const Eigen::Vector3f &globalPosition) {
    this->globalPosition = globalPosition;
}

Eigen::MatrixXd SingleChainManipulability::computeJacobian(IKSolver::CartesianSelection mode) {
    return GetJacobianSubMatrix(jacobian, mode);
}



}
