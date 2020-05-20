#include "AssimpReader.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/ManipulationObject.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>

namespace
{
    bool readScene(
        const aiScene* scene,
        const VirtualRobot::TriMeshModelPtr& t,
        const std::string& filename,
        const VirtualRobot::AssimpReader::Parameters& param,
        VirtualRobot::AssimpReader::ResultMetaData& meta
    )
    {
        meta.loadingSuccessful = false;
        meta.regeneratedNormals = false;
        meta.skippedFaces = 0;
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
        if (scene->mNumMeshes == 0)
        {
            VR_ERROR << "mesh from '" << filename << "' has no mesh\n";
            return false;
        }
        if (scene->mNumMeshes > 1 && !param.mergeMultipleMeshes)
        {
            VR_ERROR << "mesh from '" << filename << "' has not exactly one mesh"
                     << " It has " << scene->mNumMeshes << " meshes\n";
            return false;
        }
        if (!scene->mMeshes[0])
        {
            VR_ERROR << "mesh from '" << filename << "' is null\n";
            return false;
        }

        t->clear();
        for (std::size_t idxMesh = 0; idxMesh < scene->mNumMeshes; ++idxMesh)
        {
#define error_log VR_ERROR << "mesh[" << idxMesh + 1 << " / "               \
                           << scene->mNumMeshes << "] from '" << filename << "'"

            const long vertexIdxOffset = t->vertices.size();
            const aiMesh& m = *(scene->mMeshes[idxMesh]);
            if (!(m.mVertices && m.mNumVertices))
            {
                error_log << " has no vertices\n";
                return false;
            }
            if (!(m.mFaces && m.mNumFaces))
            {
                error_log << " has no vertices\n";
                return false;
            }
            if (!m.mNormals)
            {
                if (!param.ignoreMissingNormals)
                {
                    error_log << " has no normals (and none were generated when loading it)\n";
                    return false;
                }
                if (param.verbose)
                {
                    error_log << " has no normals (and none were generated when loading it) (ignoring this)\n";
                }
                meta.regeneratedNormals = true;
            }

            for (unsigned i = 0; i < m.mNumVertices; ++i)
            {
                const auto& v = m.mVertices[i];
                t->addVertex({v.x, v.y, v.z});
                if (m.mNormals)
                {
                    const auto& n = m.mNormals[i];
                    t->addNormal(Eigen::Vector3f{n.x, n.y, n.z}.normalized());
                }
            }
            for (unsigned i = 0; i < m.mNumFaces; ++i)
            {
                const auto& f = m.mFaces[i];
                if (f.mNumIndices != 3)
                {
                    if (param.verbose || !param.skipInvalidFaces)
                        error_log << " has face (# " << i
                                  << ") with the wrong number of vertices ("
                                  << f.mNumIndices << ")\n";
                    if (param.skipInvalidFaces)
                    {
                        ++meta.skippedFaces;
                        continue;
                    }

                    return false;
                }
                if (
                    f.mIndices[0] >= m.mNumVertices ||
                    f.mIndices[1] >= m.mNumVertices ||
                    f.mIndices[2] >= m.mNumVertices
                )
                {
                    error_log << " has vertex index out of bounds for face # " << i
                              << " \n";
                    return false;
                }
                VirtualRobot::MathTools::TriangleFace fc;
                fc.id1 = vertexIdxOffset + f.mIndices[0];
                fc.id2 = vertexIdxOffset + f.mIndices[1];
                fc.id3 = vertexIdxOffset + f.mIndices[2];
                fc.idNormal1 = vertexIdxOffset + f.mIndices[0];
                fc.idNormal2 = vertexIdxOffset + f.mIndices[1];
                fc.idNormal3 = vertexIdxOffset + f.mIndices[2];
                t->addFace(fc);
            }
#undef error_log
        }

        t->scale(param.scaling);
        t->mergeVertices(param.eps);
        t->addMissingNormals();
        meta.loadingSuccessful = true;
        return true;
    }
}

namespace VirtualRobot
{
    AssimpReader::AssimpReader(float eps, float scaling)
    {
        parameters.scaling = scaling;
        parameters.eps = eps;
    }

    std::string AssimpReader::get_extensions()
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

    bool AssimpReader::readFileAsTriMesh(const std::string& filename, const TriMeshModelPtr& t)
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
        return readScene(scene, t, filename, parameters, resultMetaData);
    }
    bool AssimpReader::readBufferAsTriMesh(const std::string_view& v, const TriMeshModelPtr& t)
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
        return readScene(scene, t, "<MEMORY_BUFFER>", parameters, resultMetaData);
    }

    TriMeshModelPtr AssimpReader::readFileAsTriMesh(const std::string& filename)
    {
        auto tri = std::make_shared<TriMeshModel>();
        if (!readFileAsTriMesh(filename, tri))
        {
            return nullptr;
        }
        return tri;
    }

    TriMeshModelPtr AssimpReader::readBufferAsTriMesh(const std::string_view& v)
    {
        auto tri = std::make_shared<TriMeshModel>();
        if (!readBufferAsTriMesh(v, tri))
        {
            return nullptr;
        }
        return tri;
    }

    ManipulationObjectPtr AssimpReader::readFileAsManipulationObject(const std::string& filename, const std::string& name)
    {
        if (auto tri = readFileAsTriMesh(filename))
        {
            return std::make_shared<ManipulationObject>(name, tri, filename);
        }
        return nullptr;
    }
    ManipulationObjectPtr AssimpReader::readBufferAsManipulationObject(const std::string_view& v, const std::string& name)
    {
        if (auto tri = readBufferAsTriMesh(v))
        {
            return std::make_shared<ManipulationObject>(name, tri);
        }
        return nullptr;
    }
}

