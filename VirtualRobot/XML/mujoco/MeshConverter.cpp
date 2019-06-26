#include "MeshConverter.h"

#include <VirtualRobot/VirtualRobotChecks.h>


namespace VirtualRobot::mujoco
{


VirtualRobot::TriMeshModelPtr MeshConverter::toVirtualRobotPtr(const Mesh& mesh, float scaling)
{
    return VirtualRobot::TriMeshModelPtr( 
                new VirtualRobot::TriMeshModel(toVirtualRobot(mesh, scaling)));
}

VirtualRobot::TriMeshModel MeshConverter::toVirtualRobot(const Mesh& mj, float scaling)
{
    VirtualRobot::TriMeshModel vr;
    
    // copy vertices
    for (int i = 0; i < mj.nvertex(); ++i)
    {
        vr.addVertex(mj.vertexPosition(i) * scaling);
    }
    
    // copy normals
    for (int i = 0; i < mj.nnormal(); ++i)
    {
        vr.addNormal(mj.vertexNormal(i));
    }
    
    // copy faces
    for (int i = 0; i < mj.nface(); ++i)
    {
        Eigen::Vector3i faceVertexIndex = mj.faceVertexIndex(i);
        
        // check before casting
        VR_CHECK_NONNEGATIVE(faceVertexIndex.minCoeff());
        VR_CHECK_LESS(faceVertexIndex.maxCoeff(), static_cast<int>(vr.vertices.size()));
        
        Eigen::Matrix<unsigned int, 3, 1> vertexIDs = faceVertexIndex.cast<unsigned int>();
        
        VirtualRobot::MathTools::TriangleFace triFace;
        
        // set position indices
        triFace.set(vertexIDs(0), vertexIDs(1), vertexIDs(2));
        
        // set normals (if present)
        if (vertexIDs.maxCoeff() < vr.normals.size())
        {
            triFace.setNormal(vertexIDs(0), vertexIDs(1), vertexIDs(2));
            
            Eigen::Vector3f normal = Eigen::Vector3f::Zero();
            for (int i = 0; i < vertexIDs.size(); ++i)
            {
                normal += vr.normals[vertexIDs(i)];
            }
            triFace.normal = normal / vertexIDs.size();
        }
        else
        {
            // create a normal from the vertices
            triFace.normal = vr.CreateNormal(vr.vertices[triFace.id1], vr.vertices[triFace.id2],
                    vr.vertices[triFace.id3]);
        }
        
        vr.addFace(triFace);
    }
    
    return vr;
}


template <typename ValueT>
static ValueT max3(ValueT a, ValueT b, ValueT c)
{
    return std::max(a, std::max(b, c));
}

Mesh MeshConverter::fromVirtualRobot(const VirtualRobot::TriMeshModel& vr, float scaling)
{
    Mesh mj;
    
    const auto& vertices = vr.vertices;
    const auto& normals = vr.normals;

    for (const auto& face : vr.faces)
    {
        VR_CHECK_EQUAL(mj.nvertex(), mj.nnormal());

        int index = static_cast<int>(mj.nvertex());

        VR_CHECK_LESS_EQUAL(max3(face.id1, face.id2, face.id3), vertices.size());
        mj.addVertexPosition(vertices.at(face.id1) * scaling);
        mj.addVertexPosition(vertices.at(face.id2) * scaling);
        mj.addVertexPosition(vertices.at(face.id3) * scaling);

        if (max3(face.idNormal1, face.idNormal2, face.idNormal3) < normals.size())
        {
            mj.addVertexNormal(normals.at(face.idNormal1));
            mj.addVertexNormal(normals.at(face.idNormal2));
            mj.addVertexNormal(normals.at(face.idNormal3));
        }
        else
        {
            // take anything in face.normal, normalize it and use it
            Eigen::Vector3f normal = face.normal;
            if (normal.isZero(1e-6f))
            {
                VR_WARNING << "No normal information in TriMeshModel";
                normal = Eigen::Vector3f::UnitX();
            }
            normal.normalize();

            // add it 3 times (once for each vertex position)
            int i = 3;
            while (i-- > 0) // use convergence operator
            {
                mj.addVertexNormal(normal);
            }
        }

        mj.addFaceVertexIndex({ index, index + 1, index + 2 });
    }
    
    return mj;
}

Mesh MeshConverter::toMujoco(const VirtualRobot::TriMeshModel& triMeshModel, float scaling)
{
    return fromVirtualRobot(triMeshModel, scaling);
}

}
