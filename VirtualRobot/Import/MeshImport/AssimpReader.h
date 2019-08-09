#pragma once

#include <VirtualRobot/VirtualRobot.h>

#include <stdio.h>
#include <string>

struct aiScene;

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT AssimpReader
    {
    public:
        AssimpReader(float eps = FLT_MIN, float scaling = 1);
        virtual ~AssimpReader() = default;

        static std::string get_description()
        {
            return "Open Asset Import Library";
        }
        static std::string get_extensions();

        // read data and store it to trimesh
        bool readFileAsTriMesh(const std::string& _filename, const TriMeshModelPtr& t, bool mergeMultipleMeshes = false, bool ignoreMissingNormals = false);
        bool readBufferAsTriMesh(const std::string_view& v, const TriMeshModelPtr& t, bool mergeMultipleMeshes = false, bool ignoreMissingNormals = false);

        TriMeshModelPtr readFileAsTriMesh(const std::string& filename, bool mergeMultipleMeshes = false, bool ignoreMissingNormals = false);
        TriMeshModelPtr readBufferAsTriMesh(const std::string_view& v, bool mergeMultipleMeshes = false, bool ignoreMissingNormals = false);

        ManipulationObjectPtr readFileAsManipulationObject(const std::string& filename, const std::string& name = "", bool mergeMultipleMeshes = false, bool ignoreMissingNormals = false);
        ManipulationObjectPtr readBufferAsManipulationObject(const std::string_view& v, const std::string& name = "", bool mergeMultipleMeshes = false, bool ignoreMissingNormals = false);

        /** Set the threshold to be used for considering two point to be equal.
            Can be used to merge small gaps */
        void set_epsilon(float _eps);

        /// Returns the threshold to be used for considering two point to be equal.
        float epsilon() const;

        void setScaling(float s);
    private:
        float scaling;
        float eps;
    };

    typedef boost::shared_ptr<AssimpReader> AssimpReaderPtr;
}

