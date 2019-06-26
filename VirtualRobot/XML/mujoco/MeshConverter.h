#pragma once

#include <VirtualRobot/Visualization/TriMeshModel.h>

#include "Mesh.h"


namespace VirtualRobot::mujoco
{
    /**
     * @brief Converts from and to MuJoCo mesh formats.
     */
    class MeshConverter
    {
    public:
        
        // From mujoco::Mesh
        
        /// Construct a VirtualRobot::TriMeshModel from mujoco::Mesh.
        static VirtualRobot::TriMeshModel toVirtualRobot(const mujoco::Mesh& mesh,
                                                         float scaling = 1.0);
        static VirtualRobot::TriMeshModelPtr toVirtualRobotPtr(const mujoco::Mesh& mesh,
                                                               float scaling = 1.0);
       
        
        // To mujoco::Mesh
        
        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        static mujoco::Mesh fromVirtualRobot(const VirtualRobot::TriMeshModel& triMeshModel,
                                             float scaling = 1.0);
        
        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        static mujoco::Mesh toMujoco(const VirtualRobot::TriMeshModel& triMeshModel,
                                     float scaling = 1.0);
        
    private:
        
        /// Private constructor.
        MeshConverter() = default;
        
    };

}
