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
        
        // STATELESS API (static).
        
        // From mujoco::Mesh to VirtualRobot::TriMeshModel
        
        /// Construct a VirtualRobot::TriMeshModel from mujoco::Mesh.
        static VirtualRobot::TriMeshModel toVirtualRobot(const mujoco::Mesh& mesh,
                                                         float scaling = 1.0);
        static VirtualRobot::TriMeshModelPtr toVirtualRobotPtr(const mujoco::Mesh& mesh,
                                                               float scaling = 1.0);
       
        
        // To mujoco::Mesh (from VirtualRobot::TriMeshModel)
        
        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        static mujoco::Mesh fromVirtualRobot(const VirtualRobot::TriMeshModel& triMeshModel,
                                             float scaling = 1.0);
        
        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        static mujoco::Mesh toMujoco(const VirtualRobot::TriMeshModel& triMeshModel,
                                     float scaling = 1.0);
        
    public:
        
        // STATEFUL API.
        
        
        /// Constructor.
        MeshConverter();
        
        
        /// Set the scaling.
        float getScaling() const;
        /// Get the scaling.
        void setScaling(float value);
        
        
        // From mujoco::Mesh to VirtualRobot::TriMeshModel
        
        /// Construct a VirtualRobot::TriMeshModel from mujoco::Mesh.
        VirtualRobot::TriMeshModel toVirtualRobot(const mujoco::Mesh& mesh);
        VirtualRobot::TriMeshModelPtr toVirtualRobotPtr(const mujoco::Mesh& mesh);
       
        
        // To mujoco::Mesh (from VirtualRobot::TriMeshModel)
        
        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        mujoco::Mesh fromVirtualRobot(const VirtualRobot::TriMeshModel& triMeshModel);
        
        /// Construct mujoco::Mesh from a VirtualRobot::TriMeshModel.
        mujoco::Mesh toMujoco(const VirtualRobot::TriMeshModel& triMeshModel);
        
        
        
    private:
        
        /// Check whether the command `MESHLABSERVER` is available using `system("which ...")`.
        static bool checkMeshlabserverAvailable();
        
        
        /// Command used to convert mesh files to STL.
        static const std::string MESHLABSERVER;
        
        
    private:
        
        /// The scaling.
        float scaling = 1.0f;
        
    };

}
