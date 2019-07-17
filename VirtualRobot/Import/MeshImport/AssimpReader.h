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

        std::string get_description() const
        {
            return "Open Asset Import Library";
        }
        std::string get_extensions() const;

        // read data and store it to trimesh
        bool read(const std::string& _filename, const TriMeshModelPtr& t);
        bool readFromBuffer(const std::string_view& v, const TriMeshModelPtr& t);

        /** Set the threshold to be used for considering two point to be equal.
            Can be used to merge small gaps */
        void set_epsilon(float _eps);

        /// Returns the threshold to be used for considering two point to be equal.
        float epsilon() const;

        void setScaling(float s);
    private:
        bool read(const aiScene* scene, const TriMeshModelPtr& t, const std::string& filename);
        float scaling;
        float eps;
    };

    typedef boost::shared_ptr<AssimpReader> AssimpReaderPtr;
}

