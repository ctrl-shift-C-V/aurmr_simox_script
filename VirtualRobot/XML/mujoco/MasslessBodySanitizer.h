#pragma once

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MJCF/Document.h>


namespace VirtualRobot
{
namespace mujoco
{

    class MergedBodySet
    {
    public:
        
        MergedBodySet();
        MergedBodySet(const std::string& bodyName);
        
        void addBody(const std::string& bodyName);
        bool containsBody(const std::string& bodyName) const;
        
        const std::string& getMergedBodyName() const;
        
        
    private:
        
        void updateMergedBodyName();
        
        std::string mergedBodyName;
        std::vector<std::string> originalBodyNames;
        
    };
    

    class MasslessBodySanitizer
    {
    public:
        
        MasslessBodySanitizer(RobotPtr& robot);
       
        /// Set the scaling for lengths (e.g. positions) (to m).
        void setLengthScale(float toMeter);
        
        void sanitize(mjcf::Body root);
        
        const std::vector<MergedBodySet>& getMergedBodySets() const;

        const std::string& getMergedBodyName(const std::string& originalBodyName);
        
        MergedBodySet& getMergedBodySetWith(const std::string& bodyName);
        
        
    private:
        
        void sanitizeRecursive(mjcf::Body body);
        void sanitizeLeafBody(mjcf::Body body);
        
        void mergeBodies(mjcf::Body body, mjcf::Body childBody, Eigen::Matrix4f& accChildPose);
        
        const std::string t = "| ";
        
        /// Scaling factor of lengths (to m).
        float lengthScale = 1.0f;
        
        /// The robot.
        RobotPtr& robot;
        
        std::vector<MergedBodySet> mergedBodySets;
        
    };
    
} 
}
