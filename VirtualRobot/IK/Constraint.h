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
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Constraint_h_
#define _VirtualRobot_Constraint_h_

#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/IK/JacobiProvider.h>

#include <boost/shared_ptr.hpp>

class SoSeparator;

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Constraint : public JacobiProvider, public boost::enable_shared_from_this<Constraint>
    {
        public:
            Constraint(const RobotNodeSetPtr &nodeSet);

            virtual bool getRobotPoseForConstraint(Eigen::Matrix4f &pose);

            virtual void visualize(SoSeparator *sep);
            virtual void visualizeContinuously(SoSeparator *sep);
            void setVisualizationColor(const Eigen::Vector4f &color);

            virtual std::string getConstraintType() = 0;

            void setPriority(int priority);
            int getPriority();

        protected:
            Eigen::Vector4f visualizationColor;
            int priority;
    };

    typedef boost::shared_ptr<Constraint> ConstraintPtr;
}

#endif