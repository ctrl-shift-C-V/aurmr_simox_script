#pragma once


#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MJCF/Document.h>


namespace VirtualRobot::mujoco
{


    class BodySanitizer
    {
    public:
        
        /// Constructor.
        BodySanitizer(RobotPtr& robot);
        
        /// Virtual destructor.
        virtual ~BodySanitizer();
        
        
        /// Sanitize the given root body and its children.
        virtual void sanitize(mjcf::Body root) = 0;
        
        
        
    protected:
        
        /// The robot.
        RobotPtr& robot;
        
    };

}
