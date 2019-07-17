#include "AssimpReader.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>

namespace VirtualRobot
{
    AssimpReader::AssimpReader(float eps, float scaling) :
        scaling{scaling}, eps{eps}
    {}

    std::string AssimpReader::get_extensions() const
    {
        static const auto extensions = []
        {
            const std::string upper =
            "3D 3DS 3MF AC AC3D ACC AMJ ASE ASK B3D BLEND BVH CMS COB "  \
            "DAE DXF ENFF FBX LWO LWS LXO MD2 MD3 MD5 MDC MDL MESH MOT " \
            "MS3D NDO NFF OBJ OFF OGEX PLY PMX PRJ Q3O Q3S RAW SCN SIB " \
            "SMD STP STL STLA STLB TER UC VTA X X3D XGL ZGL";
            std::string both = upper + upper;
            std::transform(
                both.begin(), both.begin() + upper.size(), both.begin(),
                [](unsigned char c)
            {
                return std::tolower(c);
            }
            );
            return both;
        }();
        return extensions;
    }

    bool AssimpReader::read(const std::string& filename, const TriMeshModelPtr& t)
    {
        //code adapted from http://assimp.sourceforge.net/lib_html/usage.html
        Assimp::Importer importer;
        // And have it read the given file with some example postprocessing
        // Usually - if speed is not the most important aspect for you - you'll
        // propably to request more postprocessing than we do in this example.
        const aiScene* scene = importer.ReadFile(
                                   filename,
                                   aiProcess_JoinIdenticalVertices    |
                                   aiProcess_Triangulate              |
                                   aiProcess_GenSmoothNormals         |
                                   aiProcess_SortByPType
                               );
        return read(scene, t, filename);
    }
    bool AssimpReader::readFromBuffer(const std::string_view& v, const TriMeshModelPtr& t)
    {
        //code adapted from http://assimp.sourceforge.net/lib_html/usage.html
        Assimp::Importer importer;
        // And have it read the given file with some example postprocessing
        // Usually - if speed is not the most important aspect for you - you'll
        // propably to request more postprocessing than we do in this example.
        const aiScene* scene = importer.ReadFileFromMemory(
                                   v.data(), v.size(),
                                   aiProcess_JoinIdenticalVertices    |
                                   aiProcess_Triangulate              |
                                   aiProcess_GenSmoothNormals         |
                                   aiProcess_SortByPType
                               );
        return read(scene, t, "");
    }

    void AssimpReader::set_epsilon(float _eps)
    {
        eps = _eps;
    }

    float AssimpReader::epsilon() const
    {
        return eps;
    }

    void AssimpReader::setScaling(float s)
    {
        scaling = s;
    }

    bool AssimpReader::read(const aiScene* scene, const TriMeshModelPtr& t, const std::string& filename)
    {
        if (!t)
        {
            VR_ERROR << "Tri mesh is null\n";
            return false;
        }
        if (!scene)
        {
            VR_ERROR << "unable to load the file '" << filename << "'\n";
            return false;
        }
        if (!scene->mRootNode)
        {
            VR_ERROR << "mesh from '" << filename << "' has no root node\n";
            return false;
        }
        if (scene->mNumMeshes != 1)
        {
            VR_ERROR << "mesh from '" << filename << "' has not exactly one mesh\n";
            return false;
        }
        if (!scene->mMeshes[0])
        {
            VR_ERROR << "mesh from '" << filename << "' is null\n";
            return false;
        }

        const aiMesh& m = *(scene->mMeshes[0]);
        if (!(m.mVertices && m.mNumVertices))
        {
            VR_ERROR << "mesh from '" << filename << "' has no vertices\n";
            return false;
        }
        if (!(m.mFaces && m.mNumFaces))
        {
            VR_ERROR << "mesh from '" << filename << "' has no vertices\n";
            return false;
        }
        if (!m.mNormals)
        {
            VR_ERROR << "mesh from '" << filename << "' has no normals (and none were generated when loading it)\n";
            return false;
        }

        t->clear();
        for (unsigned i = 0; i < m.mNumVertices; ++i)
        {
            const auto& v = m.mVertices[i];
            const auto& n = m.mNormals[i];
            t->addVertex({v.x, v.y, v.z});
            t->addNormal({n.x, n.y, n.z});
        }
        for (unsigned i = 0; i < m.mNumFaces; ++i)
        {
            const auto& f = m.mFaces[i];
            if (f.mNumIndices != 3)
            {
                VR_ERROR << "mesh from '" << filename << "' has face (# " << i << ") with the wrong number of vertices\n";
                return false;
            }
            MathTools::TriangleFace fc;
            fc.id1 = f.mIndices[0];
            fc.id2 = f.mIndices[1];
            fc.id3 = f.mIndices[2];
            fc.idNormal1 = f.mIndices[0];
            fc.idNormal2 = f.mIndices[1];
            fc.idNormal3 = f.mIndices[2];
            t->addFace(fc);
        }

        t->scale(scaling);
        t->mergeVertices(eps);
        return true;
    }
}

