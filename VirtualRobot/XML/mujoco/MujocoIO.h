#pragma once

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MJCF/Document.h>

#include "RobotMjcf.h"
#include "body_sanitizer/BodySanitizer.h"


namespace VirtualRobot::mujoco
{
    
    /// Body sanitization mode.
    enum class BodySanitizeMode
    {
        NONE,           ///< No sanitization (boundmass still handles massless bodies).
        DUMMY_MASS,     ///< Add dummy mass to massless bodies.
        MERGE,          ///< Merge massless bodies.
    };
    BodySanitizeMode toBodySanitizeMode(const std::string& string);
    
    
    /**
     * @brief Converts a VirtualRobot robot model to MuJoCo MJCF format.
     */
    class MujocoIO
    {
    public:
        
        /// Constructor.
        /// @throws VirtualRobotException if robot is null
        MujocoIO(RobotPtr robot);
        

        // CONFIGURATION
        
        /// Set whether paths (e.g. meshes) shall be relative instead of absolute.
        /// This can cause problems when including the generated model from another directory.
        void setUseRelativePaths(bool useRelative);
        
        /// Set the scaling for lengths (to m).
        void setLengthScale(float value);
        /// Set the scaling for meshes (to m).
        void setMeshScale(float value);
        /// Set the scaling for mass (to kg).
        void setMassScale(float value);

        /// Set the actuator type.
        void setActuatorType(ActuatorType value);

        /// Set the body sanitize mode.
        void setBodySanitizeMode(BodySanitizeMode mode);
        
        /// Get the world mount mode.
        WorldMountMode getWorldMountMode() const;
        /// Set the world mount mode.
        void setWorldMountMode(const WorldMountMode& value);
        
        void setVerbose(bool value);
        
        
        // CONVERT
        
        
        
        
        /**
         * @brief Create a Mujoco XML (MJCF) document for the given robot.
         * @param filename   the output filename (without directory)
         * @param basePath   the output directory
         * @param meshRelDir the directory relative to basePath where meshes shall be placed
         * @return Absolute path to saved robot model file.
         */
        std::string saveMJCF(const std::string& filename, const std::string& basePath,
                             const std::string& meshRelDir);
        
        
        /// Get MJCF.
        RobotMjcf& getMjcf();
        const RobotMjcf& getMjcf() const;
        
        
    private:
        
        /// Scale all lengths by lengthScaling.
        void scaleLengths(mjcf::AnyElement elem);
        
        
        // Configuration
        
        /// The actuator type.
        ActuatorType actuatorType = ActuatorType::MOTOR;
        
        /// Sanitize mode.
        BodySanitizeMode bodySanitizeMode = BodySanitizeMode::DUMMY_MASS;
        
        /// World mount mode.
        WorldMountMode worldMountMode = WorldMountMode::FIXED;
        
        /// Verbose printing.
        bool verbose = false;
        
        
        // Input
        
        /// The input robot.
        RobotPtr robot;
        
        
        // Processing
        
        /// The robot MJCF document.
        RobotMjcf mjcf;
        
        /// Sanitizes massless bodies.
        std::unique_ptr<mujoco::BodySanitizer> bodySanitizer;
        
        const std::string t = "| ";
        
    };
    
    
}
