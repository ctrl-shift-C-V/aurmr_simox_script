#include "WorkspaceRepresentation.h"
#include "../VirtualRobotException.h"
#include "../Robot.h"
#include "../RobotNodeSet.h"
#include "../Compression/CompressionRLE.h"
#include "../Compression/CompressionBZip2.h"
#include "../SceneObjectSet.h"
#include "../Nodes/RobotNode.h"
#include "../Visualization/Visualization.h"
#include "../Visualization/VisualizationFactory.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../Visualization/ColorMap.h"
#include "../ManipulationObject.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "SimoxUtility/math/periodic/periodic_clamp.h"
#include "VirtualRobot.h"
#include <VirtualRobot/Random.h>
#include <fstream>
#include <cmath>
#include <cfloat>
#include <climits>
#include <thread>

#include <Eigen/Geometry>

namespace VirtualRobot
{
    using std::cout;
    using std::endl;

    WorkspaceRepresentation::WorkspaceRepresentation(RobotPtr robot)
    {
        THROW_VR_EXCEPTION_IF(!robot, "Need a robot ptr here");
        this->robot = robot;
        type = "WorkspaceRepresentation";
        versionMajor = 2;
        versionMinor = 9;
        orientationType = Hopf;//EulerXYZExtrinsic;
        reset();
    }

    int WorkspaceRepresentation::sumAngleReachabilities(int x0, int x1, int x2) const
    {
        int res = 0;

        if (!data->hasEntry(x0, x1, x2))
        {
            return 0;
        }

        for (int d = 0; d < numVoxels[3]; d++)
        {
            for (int e = 0; e < numVoxels[4]; e++)
            {
                for (int f = 0; f < numVoxels[5]; f++)
                {
                    res += data->get(x0, x1, x2, d, e, f);
                }
            }
        }

        return res;
    }


    int WorkspaceRepresentation::avgAngleReachabilities(int x0, int x1, int x2) const
    {
        int res = 0;

        if (!data->hasEntry(x0, x1, x2))
        {
            return 0;
        }

        for (int d = 0; d < numVoxels[3]; d++)
        {
            for (int e = 0; e < numVoxels[4]; e++)
            {
                for (int f = 0; f < numVoxels[5]; f++)
                {
                    res += data->get(x0, x1, x2, d, e, f);
                }
            }
        }

        res /=  numVoxels[3]* numVoxels[4]* numVoxels[5];

        return res;
    }


    void WorkspaceRepresentation::uncompressData(const unsigned char* source, int size, unsigned char* dest)
    {
        unsigned char count;
        unsigned char value;

        for (int i = 0; i < size / 2; i++)
        {
            count = *source;
            source++;
            value = *source;
            source++;
            memset(dest, (int)value, sizeof(unsigned char) * count);
            dest += count;
        }
    }

    unsigned char* WorkspaceRepresentation::compressData(const unsigned char* source, int size, int& compressedSize)
    {
        // on large arrays sometimes an out-of-memory exception is thrown, so in order to reduce the size of the array, we assume we can compress it
        // hence, we have to check if the compressed size does not exceed the original size on every pos increase
        unsigned char* dest;

        try
        {
            dest  = new unsigned char[/*2 * */size];
        }
        catch(const std::exception &e)
        {
            VR_ERROR << "Error:" << e.what() << endl << "Could not assign " << size << " bytes of memory. Reduce size of WorkspaceRepresentation data..." << std::endl;
            throw;
        }
        catch (...)
        {
            VR_ERROR << "Could not assign " << size << " bytes of memory. Reduce size of WorkspaceRepresentation data..." << std::endl;
            throw;
        }

        int pos = 0;

        unsigned char curValue = source[0];
        unsigned char count = 1;

        for (int i = 1; i < size; i++)
        {
            if (source[i] == curValue)
            {
                if (count == 255)
                {
                    dest[pos] = 255;
                    dest[pos + 1] = curValue;
                    pos += 2;
                    THROW_VR_EXCEPTION_IF(pos >= size, "Could not perform run-length compression. Data is too cluttered!!!");

                    count = 1;
                }
                else
                {
                    count++;
                }
            }
            else
            {
                dest[pos] = count;
                dest[pos + 1] = curValue;
                pos += 2;
                THROW_VR_EXCEPTION_IF(pos >= size, "Could not perform run-length compression. Data is too cluttered!!!");

                curValue = source[i];
                count = 1;
            }
        }

        if (count > 0)
        {
            dest[pos] = count;
            dest[pos + 1] = curValue;
            pos += 2;
            THROW_VR_EXCEPTION_IF(pos >= size, "Could not perform run-length compression. Data is too cluttered!!!");
        }

        compressedSize = pos;
        return dest;
    }

    void WorkspaceRepresentation::load(const std::string& filename)
    {
        std::ifstream file(filename.c_str(), std::ios::in | std::ios::binary);
        THROW_VR_EXCEPTION_IF(!file, "File could not be read.");
        reset();

        try
        {
            std::string tmpString;

            std::string tmpStr2 = type;
            tmpStr2 += " Binary File";

            // Check file type
            FileIO::readString(tmpString, file);
            bool fileTypeOK = false;

            if (tmpString == "WorkspaceRepresentation Binary File" ||
                tmpString == "Reachability Binary File" ||
                tmpString == "Reachbaility Binary File" || // typo in old versions
                tmpString == "Manipulability Binary File" ||
                tmpString == "ReachabilitySpace Binary File" ||
                tmpString == tmpStr2)
            {
                fileTypeOK = true;
            }


            THROW_VR_EXCEPTION_IF(!fileTypeOK, "Wrong file format:" << tmpString);

            // Check version
            int version[2];
            version[0] = (int)(FileIO::read<ioIntTypeRead>(file));
            version[1] = (int)(FileIO::read<ioIntTypeRead>(file));

            // first check if the current version is used
            if (version[0] != versionMajor || version[1] != versionMinor)
            {
                std::cout << "File version: " << version[0] << "." << version[1] << std::endl;
                // now check if an older version is used
                THROW_VR_EXCEPTION_IF(
                    (version[0] > 2) ||
                    (version[0] == 2 && !(version[1]>=0 && version[1] <= 9)) ||
                    (version[0] == 1 && !(version[1] == 0 || version[1] == 2 || version[1] == 3)
                    ),  "Wrong file format version");
            }
            if (version[0] > 2 || (version[0] == 2 && version[1] > 7))
            {
                orientationType = Hopf;
            }
            else if (version[0] == 2 && version[1] > 6)
            {
                orientationType = EulerXYZExtrinsic;
            }
            else if (version[0] == 2 && version[1] == 6)
            {
                orientationType = EulerXYZ;
            }
            else
            {
                orientationType = RPY;
            }

            versionMajor = version[0];
            versionMinor = version[1];
            // Check Robot name
            FileIO::readString(tmpString, file);
            THROW_VR_EXCEPTION_IF(tmpString != robot->getType(), "Wrong Robot");

            // Check Node Set
            FileIO::readString(tmpString, file);
            nodeSet = robot->getRobotNodeSet(tmpString);
            THROW_VR_EXCEPTION_IF(!nodeSet, "Node Set does not exist.");

            if (version[0] > 1 || (version[0] == 1 && version[1] > 0))
            {
                int nrNodes = (int)(FileIO::read<ioIntTypeRead>(file));

                THROW_VR_EXCEPTION_IF(nrNodes >= 0 && nodeSet->getSize() != static_cast<std::size_t>(nrNodes), "Node Sets don't match (size differs).");

                // Check joint limits
                std::vector<RobotNodePtr> nodes = nodeSet->getAllRobotNodes();

                for (auto & node : nodes)
                {
                    float limits[2];
                    FileIO::readArray<float>(limits, 2, file);

                    //limits[0] = (int)(FileIO::read<ioIntTypeRead>(file));
                    //limits[1] = (int)(FileIO::read<ioIntTypeRead>(file));
                    if (fabs(node->getJointLimitLo() - limits[0]) > 0.01 || fabs(node->getJointLimitHi() - limits[1]) > 0.01)
                    {
                        VR_WARNING << "Joint limit mismatch for " << node->getName() << ", min: " << node->getJointLimitLo() << " / " << limits[0] << ", max: " << node->getJointLimitHi() << " / " << limits[1] << std::endl;
                    }
                }
            }

            // Check TCP
            FileIO::readString(tmpString, file);
            tcpNode = robot->getRobotNode(tmpString);
            THROW_VR_EXCEPTION_IF(!tcpNode, "Unknown TCP");

            // Check Base Joint
            if (version[0] > 1 || (version[0] == 1 &&  version[1] > 0))
            {
                FileIO::readString(tmpString, file);
                if (tmpString.empty() || !robot->hasRobotNode(tmpString))
                {
                    VR_WARNING << "Base ndoe not gifven/known:" << tmpString << std::endl;
                }
                else
                {
                    baseNode = robot->getRobotNode(tmpString);
                    THROW_VR_EXCEPTION_IF(!baseNode, "Unknown Base Joint");
                }
            }


            // Static collision model
            FileIO::readString(tmpString, file);

            if (tmpString != "" && tmpString != "not set")
            {
                staticCollisionModel = robot->getRobotNodeSet(tmpString);
            }

            // Dynamic collision model
            FileIO::readString(tmpString, file);

            if (tmpString != "" && tmpString != "not set")
            {
                dynamicCollisionModel = robot->getRobotNodeSet(tmpString);
            }

            buildUpLoops = (int)(FileIO::read<ioIntTypeRead>(file));

            collisionConfigs = (int)(FileIO::read<ioIntTypeRead>(file));
            discretizeStepTranslation = FileIO::read<float>(file);
            discretizeStepRotation = FileIO::read<float>(file);

            for (int & numVoxel : numVoxels)
            {
                numVoxel = (int)(FileIO::read<ioIntTypeRead>(file));
            }

            //FileIO::readArray<int>(numVoxels, 6, file);
            int voxelFilledCount = (int)(FileIO::read<ioIntTypeRead>(file));
            int maxEntry = (int)(FileIO::read<ioIntTypeRead>(file));

            for (int i = 0; i < 6; i++)
            {
                minBounds[i] = FileIO::read<float>(file);
                maxBounds[i] = FileIO::read<float>(file);
                spaceSize[i] = maxBounds[i] - minBounds[i];
            }

            for (int i = 0; i < 6; i++)
            {
                achievedMinValues[i] = FileIO::read<float>(file);
                achievedMaxValues[i] = FileIO::read<float>(file);
            }

            if ((version[0] > 2) || (version[0] == 2 && version[1] >= 2))
            {
                if (!customLoad(file))
                {
                    VR_ERROR << "Custom loading failed?!" << std::endl;
                }
            }

            // Read Data
            FileIO::readString(tmpString, file);
            THROW_VR_EXCEPTION_IF(tmpString != "DATA_START", "Bad file format, expecting DATA_START.");

            long size = numVoxels[0] * numVoxels[1] * numVoxels[2] * numVoxels[3] * numVoxels[4] * numVoxels[5];
            data.reset(new WorkspaceDataArray(numVoxels[0], numVoxels[1], numVoxels[2], numVoxels[3], numVoxels[4], numVoxels[5], true));

            if (version[0] <= 1 || (version[0] == 2 && version[1] <= 3))
            {
                // one data block
                unsigned char* d = new unsigned char[size];

                if (version[0] == 1 && version[1] <= 2)
                {
                    // Data is uncompressed
                    FileIO::readArray<unsigned char>(d, size, file);
                }
                else
                {
                    // Data is compressed
                    int compressedSize = (int)(FileIO::read<ioIntTypeRead>(file));
                    unsigned char* compressedData = new unsigned char[compressedSize];
                    FileIO::readArray<unsigned char>(compressedData, compressedSize, file);

                    if ((version[0] > 2) || (version[0] == 2 && version[1] >= 1))
                    {
                        CompressionRLE::RLE_Uncompress(compressedData, d, compressedSize);
                    }
                    else
                    {
                        uncompressData(compressedData, compressedSize, d);
                    }

                    delete[] compressedData;
                }

                // convert old data format
                unsigned char* dRot;
                unsigned int sizeX0 = numVoxels[1] * numVoxels[2] * numVoxels[3] * numVoxels[4] * numVoxels[5];
                unsigned int sizeX1 = numVoxels[2] * numVoxels[3] * numVoxels[4] * numVoxels[5];
                unsigned int sizeX2 = numVoxels[3] * numVoxels[4] * numVoxels[5];
                unsigned int sizeX3 = numVoxels[4] * numVoxels[5];
                unsigned int sizeX4 = numVoxels[5];
                dRot = new unsigned char[numVoxels[3]*numVoxels[4]*numVoxels[5]];

                for (int x = 0; x < numVoxels[0]; x++)
                    for (int y = 0; y < numVoxels[1]; y++)
                        for (int z = 0; z < numVoxels[2]; z++)
                        {
                            for (int a = 0; a < numVoxels[3]; a++)
                                for (int b = 0; b < numVoxels[4]; b++)
                                    for (int c = 0; c < numVoxels[5]; c++)
                                    {
                                        dRot[a * sizeX3 + b * sizeX4 + c] =
                                            d[x * sizeX0 + y * sizeX1 + z * sizeX2 + a * sizeX3 + b * sizeX4 + c];
                                    }

                            data->setDataRot(dRot, x, y, z);
                        }

                delete [] dRot;
                delete[] d;
            }
            else
            {
                // data is split, only rotations are given in blocks
                // Data is compressed

                bool compressionBZIP2 = false;

                if (version[0] > 2 || (version[0] == 2 && version[1] >= 5))
                {
                    compressionBZIP2 = true;
                }

                if (compressionBZIP2)
                {
                    int dataSize = numVoxels[3] * numVoxels[4] * numVoxels[5];
                    unsigned char* uncompressedData = new unsigned char[dataSize];
                    CompressionBZip2Ptr bzip2(new CompressionBZip2(&file));

                    for (int x = 0; x < numVoxels[0]; x++)
                        for (int y = 0; y < numVoxels[1]; y++)
                            for (int z = 0; z < numVoxels[2]; z++)
                            {
                                int n;
                                bool readOK = bzip2->read((void*)(uncompressedData), dataSize, n);

                                if (!readOK || (n != dataSize && n != 0))
                                {
                                    VR_ERROR << "Invalid number of bytes?!" << std::endl;
                                    bzip2->close();
                                    file.close();
                                    return;
                                }

                                if (n == 0) // no data in block
                                {
                                    continue;
                                }

                                // check if we need to set the data (avoid to allocate memory for empty data blocks)
                                bool empty = true;

                                for (int i = 0; i < dataSize; i++)
                                {
                                    if (uncompressedData[i] != 0)
                                    {
                                        empty = false;
                                        break;
                                    }
                                }

                                if (!empty)
                                {
                                    data->setDataRot(uncompressedData, x, y, z);
                                }
                            }

                    delete[] uncompressedData;
                    bzip2->close();
                }
                else
                {
                    int maxCompressedSize = numVoxels[3] * numVoxels[4] * numVoxels[5] * 3;
                    unsigned char* compressedData = new unsigned char[maxCompressedSize];
                    unsigned char* uncompressedData = new unsigned char[numVoxels[3]*numVoxels[4]*numVoxels[5]];

                    for (int x = 0; x < numVoxels[0]; x++)
                        for (int y = 0; y < numVoxels[1]; y++)
                            for (int z = 0; z < numVoxels[2]; z++)
                            {
                                int compressedSize = (int)(FileIO::read<ioIntTypeRead>(file));

                                FileIO::readArray<unsigned char>(compressedData, compressedSize, file);

                                CompressionRLE::RLE_Uncompress(compressedData, uncompressedData, compressedSize);
                                data->setDataRot(uncompressedData, x, y, z);
                            }

                    delete[] compressedData;
                    delete[] uncompressedData;
                }
            }


            data->setVoxelFilledCount(voxelFilledCount);
            data->setMaxEntry(maxEntry);

            FileIO::readString(tmpString, file);
            THROW_VR_EXCEPTION_IF(tmpString != "DATA_END", "Bad file format, expecting DATA_END");
        }
        catch (VirtualRobotException& e)
        {
            VR_ERROR << e.what() << std::endl;
            file.close();
            throw;
        }

        file.close();
    }

    void WorkspaceRepresentation::save(const std::string& filename)
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet, "No WorkspaceRepresentation data loaded");

        std::ofstream file;
        file.open(filename.c_str(), std::ios::out | std::ios::binary);
        THROW_VR_EXCEPTION_IF(!file.is_open(), "Could not open file");

        try
        {
            // File type
            std::string tmpStr = type;
            tmpStr += " Binary File";
            FileIO::writeString(file, tmpStr);

            // Version
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(versionMajor));
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(versionMinor));

            // Robot type
            FileIO::writeString(file, robot->getType());

            // Node set name
            FileIO::writeString(file, nodeSet->getName());

            // Joint limits
            const std::vector<RobotNodePtr> nodes = nodeSet->getAllRobotNodes();
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(nodes.size()));

            for (const auto & node : nodes)
            {
                FileIO::write<float>(file, node->getJointLimitLo());
                FileIO::write<float>(file, node->getJointLimitHi());
            }

            // TCP name
            FileIO::writeString(file, tcpNode->getName());

            // Base Joint name
            std::string bNode;
            if (baseNode)
                bNode = baseNode->getName();

            FileIO::writeString(file, bNode);

            // Collision models
            if (staticCollisionModel)
            {
                FileIO::writeString(file, staticCollisionModel->getName());
            }
            else
            {
                FileIO::writeString(file, "not set");
            }

            if (dynamicCollisionModel)
            {
                FileIO::writeString(file, dynamicCollisionModel->getName());
            }
            else
            {
                FileIO::writeString(file, "not set");
            }

            // Build loops
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(buildUpLoops));

            // Collisions
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(collisionConfigs));

            // DiscretizeStep*
            FileIO::write<float>(file, discretizeStepTranslation);
            FileIO::write<float>(file, discretizeStepRotation);

            // Number of voxels
            //FileIO::writeArray<ioIntTypeWrite>(file, numVoxels, 6);
            for (int & numVoxel : numVoxels)
            {
                FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)numVoxel);
            }

            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(data->getVoxelFilledCount()));
            FileIO::write<ioIntTypeWrite>(file, (ioIntTypeWrite)(data->getMaxEntry()));

            // Workspace extend
            for (int i = 0; i < 6; i++)
            {
                FileIO::write<float>(file, minBounds[i]);
                FileIO::write<float>(file, maxBounds[i]);
            }

            // Workspace achieved values
            for (int i = 0; i < 6; i++)
            {
                FileIO::write<float>(file, achievedMinValues[i]);
                FileIO::write<float>(file, achievedMaxValues[i]);
            }

            if (!customSave(file))
            {
                VR_ERROR << "Custom saving failed?!" << std::endl;
            }

            // Data
            FileIO::writeString(file, "DATA_START");

            if (!data->save(file))
            {
                VR_ERROR << "Unable to store data!" << std::endl;
                return;
            }

            FileIO::writeString(file, "DATA_END");
        }
        catch (VirtualRobotException& e)
        {
            std::cout << "exception: " << e.what() << std::endl;
            file.close();
            throw;
        }

        file.close();
    }

    int WorkspaceRepresentation::getMaxEntry() const
    {
        if (!data)
        {
            return 0;
        }

        return data->getMaxEntry();
    }

    int WorkspaceRepresentation::getMaxEntry(const Eigen::Vector3f& position_global) const
    {
        Eigen::Matrix4f gp;
        gp.setIdentity();
        gp.block(0, 3, 3, 1) = position_global;

        // get voxels

        // get voxels
        unsigned int v[3];

        if (!getVoxelFromPosition(gp, v))
        {
            return 0;
        }

        return getMaxEntry(v[0], v[1], v[2]);
    }

    int WorkspaceRepresentation::getMaxEntry(int x0, int x1, int x2) const
    {
        int maxValue = 0;

        for (int a = 0; a < getNumVoxels(3); a += 1)
        {
            for (int b = 0; b < getNumVoxels(4); b += 1)
            {
                for (int c = 0; c < getNumVoxels(5); c += 1)
                {
                    int value = data->get(x0, x1, x2, a, b, c);

                    if (value >= maxValue)
                    {
                        maxValue = value;
                    }
                }
            }
        }

        return maxValue;

    }

    float WorkspaceRepresentation::getVoxelSize(int dim) const
    {
        if (dim < 0 || dim > 6)
        {
            return 0.0f;
        }

        if (numVoxels[dim] <= 0)
        {
            return 0.0f;
        }

        return spaceSize[dim] / numVoxels[dim];
    }

    RobotNodePtr WorkspaceRepresentation::getBaseNode()
    {
        return baseNode;
    }

    RobotNodePtr WorkspaceRepresentation::getTCP()
    {
        return tcpNode;
    }

    RobotNodeSetPtr WorkspaceRepresentation::getNodeSet()
    {
        return nodeSet;
    }

    Eigen::Matrix4f WorkspaceRepresentation::getToLocalTransformation() const
    {
        if (baseNode)
        {
            return Eigen::Isometry3f(baseNode->getGlobalPose()).inverse().matrix();
        }
        else
        {
            return Eigen::Matrix4f::Identity();
        }
    }

    Eigen::Matrix4f WorkspaceRepresentation::getToGlobalTransformation() const
    {
        if (baseNode)
        {
            return baseNode->getGlobalPose();
        }
        else
        {
            return Eigen::Matrix4f::Identity();
        }
    }

    void WorkspaceRepresentation::toLocal(Eigen::Matrix4f& p) const
    {
        p = getToLocalTransformation() * p;
        //if (baseNode)
        //p = baseNode->toLocalCoordinateSystem(p);
    }
    void WorkspaceRepresentation::toGlobal(Eigen::Matrix4f& p) const
    {
        p = getToGlobalTransformation() * p;
        //if (baseNode)
        //p = baseNode->toGlobalCoordinateSystem(p);
    }

    void WorkspaceRepresentation::toLocalVec(Eigen::Vector3f& positionGlobal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionGlobal;
        toLocal(t);
        positionGlobal = t.block(0, 3, 3, 1);
    }


    void WorkspaceRepresentation::toGlobalVec(Eigen::Vector3f& positionLocal) const
    {
        Eigen::Matrix4f t;
        t.setIdentity();
        t.block(0, 3, 3, 1) = positionLocal;
        toGlobal(t);
        positionLocal = t.block(0, 3, 3, 1);
    }

    void WorkspaceRepresentation::setCurrentTCPPoseEntryIfLower(unsigned char e)
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "No WorkspaceRepresentation data loaded");

        Eigen::Matrix4f p = tcpNode->getGlobalPose();
        toLocal(p);

        float x[6];
        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        if (data->get(x, this) < e)
        {
            data->setDatum(x, e, this);
        }


        buildUpLoops++;
    }

    bool WorkspaceRepresentation::getVoxelFromPose(float x[6], unsigned int v[6]) const
    {
        float pos;
        int a;

        for (int i = 0; i < 6; i++)
        {
            pos = ((x[i] - minBounds[i]) / spaceSize[i]) * (float)numVoxels[i];
            a = (int)(pos);

            if (a < 0)
            {
                // check for rounding errors
                if (a==-1 && (fabs(float(a) - pos)<0.5f))
                    a = 0;
                else
                    return false;    //pos[i] = 0; // if pose is outside of voxel space, ignore it
            }
            else if (a >= numVoxels[i])
            {
                // check for rounding errors
                if (a==numVoxels[i] && (fabs(float(a) - pos)<0.5f))
                    a = numVoxels[i]-1;
                else
                    return false;    //pos[i] = m_nVoxels[i]-1; // if pose is outside of voxel space, ignore it
            }

            v[i] = a;
        }

        return true;
    }

    bool WorkspaceRepresentation::getVoxelFromPose(const Eigen::Matrix4f& globalPose, unsigned int v[6]) const
    {
        float x[6];
        Eigen::Matrix4f p = globalPose;
        toLocal(p);
        matrix2Vector(p, x);
        return getVoxelFromPose(x, v);
    }



    bool WorkspaceRepresentation::getVoxelFromPosition(float x[3], unsigned int v[3]) const
    {
        int a;

        for (int i = 0; i < 3; i++)
        {
            a = (int)(((x[i] - minBounds[i]) / spaceSize[i]) * (float)numVoxels[i]);
            if (a < 0)
            {
                return false;    //pos[i] = 0; // if pose is outside of voxel space, ignore it
            }
            else if (a >= numVoxels[i])
            {
                return false;    //pos[i] = m_nVoxels[i]-1; // if pose is outside of voxel space, ignore it
            }

            v[i] = a;
        }

        return true;
    }

    bool WorkspaceRepresentation::getVoxelFromPosition(const Eigen::Matrix4f &globalPose, unsigned int v[3]) const
    {
        float x[6];
        Eigen::Matrix4f p = globalPose;
        toLocal(p);
        matrix2Vector(p, x);
        float x2[3];
        x2[0] = x[0];
        x2[1] = x[1];
        x2[2] = x[2];
        return getVoxelFromPosition(x2, v);
    }


    bool WorkspaceRepresentation::setRobotNodesToRandomConfig(VirtualRobot::RobotNodeSetPtr nodeSet, bool checkForSelfCollisions /*= true*/)
    {
        if (!nodeSet)
        {
            nodeSet = this->nodeSet;
        }

        if (!nodeSet)
        {
            return false;
        }

        float rndValue;
        float minJ, maxJ;
        Eigen::VectorXf v(nodeSet->getSize());
        float maxLoops = 1000;

        int loop = 0;

        // for (unsigned int i = 0; i < nodeSet->getSize(); i++)
        //     {
        //         std::cout << "node set has: " << (*nodeSet)[i]->getName() << std::endl;
        //     }

        do
        {
            // std::cout << "self collision check is set to: " << !(!checkForSelfCollisions || !staticCollisionModel || !dynamicCollisionModel) << std::endl;
            // std::cout << "node set size is: " << nodeSet->getSize() << std::endl;
            for (unsigned int i = 0; i < nodeSet->getSize(); i++)
            {
                rndValue = RandomFloat(); // value from 0 to 1
                minJ = (*nodeSet)[i]->getJointLimitLo();
                maxJ = (*nodeSet)[i]->getJointLimitHi();
                v[i] = minJ + ((maxJ - minJ) * rndValue);
            }

            robot->setJointValues(nodeSet, v);
            SceneObjectSetPtr stand = robot->getRobotNodeSet("TheStand");

            // check for collisions
            if (!checkForSelfCollisions || !staticCollisionModel || !dynamicCollisionModel)
            {
                return true;
            }

            // if (!robot->getCollisionChecker()->checkCollision(staticCollisionModel, dynamicCollisionModel))
            // {
            //     // std::cout << "successfully set to random pose" << std::endl;
            //     // std::cout << "joint values are: " << v << " and no colide" << std::endl;
            //     return true;
            // }

            if (!robot->getCollisionChecker()->checkCollision(staticCollisionModel, dynamicCollisionModel, stand))
            {
                return true;
            }
            // std::cout << "joint values are: " << v << " and there is colide" << std::endl;
            collisionConfigs++;
            loop++;
        }
        while (loop < maxLoops);

        return false;
    }


    void WorkspaceRepresentation::addPose(const Eigen::Matrix4f& globalPose, PoseQualityMeasurementPtr /*qualMeasure*/)
    {
         addPose(globalPose);
    }


    void WorkspaceRepresentation::addPose(const Eigen::Matrix4f& globalPose)
    {
        VR_ASSERT(data);
        Eigen::Matrix4f p = globalPose;
        toLocal(p);

        float x[6];
        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }



        data->increaseDatum(x, this);

        buildUpLoops++;
    }


    void WorkspaceRepresentation::print()
    {
        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << type << " - Status:" << std::endl;

        if (data)
        {
            if (nodeSet)
            {
                std::cout << "Kinematic Chain / RobotNodeSet: " << nodeSet->getName() << std::endl;
            }

            std::cout << "Base Joint: ";

            if (baseNode)
            {
                std::cout << baseNode->getName() << std::endl;
            }
            else
            {
                std::cout << "<GLOBAL POSE>" << std::endl;
            }

            std::cout << "TCP Joint: ";

            if (tcpNode)
            {
                std::cout << tcpNode->getName() << std::endl;
            }
            else
            {
                std::cout << "<not set>" << std::endl;
            }

            std::cout << "Orientation representation: ";

            switch (orientationType)
            {
                case RPY:
                    std::cout << "RPY" << std::endl;
                    break;

                case EulerXYZ:
                    std::cout << "EulerXYZ-Intrinsic" << std::endl;
                    break;

                case EulerXYZExtrinsic:
                    std::cout << "EulerXYZ-Extrinsic" << std::endl;
                    break;

                case Hopf:
                    std::cout << "Hopf" << std::endl;
                    break;

                default:
                    std::cout << "NYI" << std::endl;
            }

            std::cout << "CollisionModel static: ";

            if (staticCollisionModel)
            {
                std::cout << staticCollisionModel->getName() << std::endl;
            }
            else
            {
                std::cout << "<not set>" << std::endl;
            }

            std::cout << "CollisionModel dynamic: ";

            if (dynamicCollisionModel)
            {
                std::cout << dynamicCollisionModel->getName() << std::endl;
            }
            else
            {
                std::cout << "<not set>" << std::endl;
            }

            std::cout << "Used " << buildUpLoops << " loops for building the random configs " << std::endl;
            std::cout << "Discretization step sizes: Translation: " << discretizeStepTranslation << " - Rotation: " << discretizeStepRotation << std::endl;
            std::cout << type << " data extends: " << numVoxels[0] << "x" << numVoxels[1] << "x" << numVoxels[2] << "x" << numVoxels[3] << "x" << numVoxels[4] << "x" << numVoxels[5] << std::endl;
            long long nv = (long long)numVoxels[0] * (long long)numVoxels[1] * (long long)numVoxels[2] * (long long)numVoxels[3] * (long long)numVoxels[4] * (long long)numVoxels[5];
            std::cout << "Filled " << data->getVoxelFilledCount() << " of " << nv << " voxels" << std::endl;
            std::cout << "Collisions: " << collisionConfigs << std::endl;
            std::cout << "Maximum entry in a voxel: " << (int)data->getMaxEntry() << std::endl;
            std::cout << type << " workspace extend (as defined on construction):" << std::endl;
            std::cout << "Min boundary (local): ";

            for (float minBound : minBounds)
            {
                std::cout << minBound << ",";
            }

            std::cout << std::endl;
            std::cout << "Max boundary (local): ";

            for (float maxBound : maxBounds)
            {
                std::cout << maxBound << ",";
            }

            std::cout << std::endl;
            std::cout << "6D values achieved during buildup:" << std::endl;
            std::cout << "Minimum 6D values: ";

            for (float achievedMinValue : achievedMinValues)
            {
                std::cout << achievedMinValue << ",";
            }

            std::cout << std::endl;
            std::cout << "Maximum 6D values: ";

            for (float achievedMaxValue : achievedMaxValues)
            {
                std::cout << achievedMaxValue << ",";
            }

            std::cout << std::endl;
            customPrint();
        }
        else
        {
            std::cout << type << " not created yet..." << std::endl;
        }

        std::cout << "-----------------------------------------------------------" << std::endl;
        std::cout << std::endl;
    }

    void WorkspaceRepresentation::reset()
    {
        data.reset();
        nodeSet.reset();
        tcpNode.reset();
        baseNode.reset();
        staticCollisionModel.reset();
        dynamicCollisionModel.reset();
        buildUpLoops = 0;
        collisionConfigs = 0;
        discretizeStepTranslation = 0;
        discretizeStepRotation = 0;

        for (int i = 0; i < 6; i++)
        {
            minBounds[i] = FLT_MAX;
            maxBounds[i] = -FLT_MAX;
            achievedMinValues[i] = FLT_MAX;
            achievedMaxValues[i] = -FLT_MAX;
            numVoxels[i] = 0;
            spaceSize[i] = 0;
        }
    }

    void WorkspaceRepresentation::initialize(RobotNodeSetPtr nodeSet,
                                             float discretizeStepTranslation,
                                             float discretizeStepRotation,
                                             const float minBounds[6],
                                             const float maxBounds[6],
            SceneObjectSetPtr staticCollisionModel,
            SceneObjectSetPtr dynamicCollisionModel,
            RobotNodePtr baseNode /*= RobotNodePtr()*/,
            RobotNodePtr tcpNode /*= RobotNodePtr()*/,
            bool adjustOnOverflow /* = true */)
    {
        reset();
        THROW_VR_EXCEPTION_IF((discretizeStepTranslation <= 0.0f || discretizeStepRotation <= 0.0f), "Need positive discretize steps");

        for (int i = 0; i < 6; i++)
        {
            THROW_VR_EXCEPTION_IF(minBounds[i] >= maxBounds[i], "Min/MaxBound error");
        }

        THROW_VR_EXCEPTION_IF(!nodeSet, "NULL data, need a nodeSet");
        THROW_VR_EXCEPTION_IF(!nodeSet->isKinematicChain(), "nodeSet must be a valid kinematic chain!");
        this->nodeSet = nodeSet;
        this->tcpNode = nodeSet->getTCP();

        if (tcpNode)
        {
            this->tcpNode = tcpNode;
        }

        THROW_VR_EXCEPTION_IF(!robot->hasRobotNode(this->tcpNode), "robot does not know tcp:" << this->tcpNode->getName());
        this->baseNode = baseNode;

        if (baseNode && !robot->hasRobotNode(baseNode))
        {
            THROW_VR_EXCEPTION("Robot does not know basenode:" << baseNode->getName());
        }

        THROW_VR_EXCEPTION_IF(nodeSet->hasRobotNode(baseNode), " baseNode is part of RobotNodeSet! This is not a good idea, since the globalPose of the baseNode will change during buildup of WorkspaceRepresentation data...");
        this->staticCollisionModel = staticCollisionModel;
        this->dynamicCollisionModel = dynamicCollisionModel;

        if (!staticCollisionModel || !dynamicCollisionModel)
        {
            staticCollisionModel.reset();
            dynamicCollisionModel.reset();
        }
        else
        {
            THROW_VR_EXCEPTION_IF(staticCollisionModel->getCollisionChecker() != dynamicCollisionModel->getCollisionChecker(), "Need same collision checker instance!");
        }

        // build data
        this->discretizeStepTranslation = discretizeStepTranslation;
        this->discretizeStepRotation = discretizeStepRotation;

        for (int i = 0; i < 6; i++)
        {
            this->minBounds[i] = minBounds[i];
            this->maxBounds[i] = maxBounds[i];
            spaceSize[i] = maxBounds[i] - minBounds[i];

            if (i < 3)
            {
                numVoxels[i] = (int)(spaceSize[i] / discretizeStepTranslation) + 1;
            }
            else
            {
                numVoxels[i] = (int)(spaceSize[i] / discretizeStepRotation) + 1;
            }

            THROW_VR_EXCEPTION_IF((numVoxels[i] <= 0), " numVoxels <= 0 in dimension " << i);
        }
        // std::cout << "minBounds[0]: " << minBounds[0] << std::endl;
        // std::cout << "minBounds[1]: " << minBounds[1] << std::endl;
        // std::cout << "minBounds[2]: " << minBounds[2] << std::endl;
        // std::cout << "maxBounds[0]: " << maxBounds[0] << std::endl;
        // std::cout << "maxBounds[1]: " << maxBounds[1] << std::endl;
        // std::cout << "maxBounds[2]: " << maxBounds[2] << std::endl;
        // std::cout << "discretizeStepTranslation: " << discretizeStepTranslation << std::endl;
        // std::cout << "discretizeStepRotation: " << discretizeStepRotation << std::endl;
        // std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
        // std::cout << "numvoxels[0]: " << numVoxels[0] << std::endl;
        // std::cout << "numvoxels[1]: " << numVoxels[1] << std::endl;
        // std::cout << "numvoxels[2]: " << numVoxels[2] << std::endl;
        // std::cout << "numvoxels[3]: " << numVoxels[3] << std::endl;
        // std::cout << "numvoxels[4]: " << numVoxels[4] << std::endl;
        // std::cout << "numvoxels[5]: " << numVoxels[5] << std::endl;

        data.reset(new WorkspaceDataArray(numVoxels[0], numVoxels[1], numVoxels[2], numVoxels[3], numVoxels[4], numVoxels[5], adjustOnOverflow));

        customInitialize();
    }

    void WorkspaceRepresentation::binarize()
    {
        if (data)
        {
            data->binarize();
        }
    }

    unsigned char WorkspaceRepresentation::getEntry(const Eigen::Matrix4f& globalPose) const
    {
        if (!data)
        {
            VR_ERROR << "NULL DATA" << std::endl;
            return 0;
        }

        float x[6];

        Eigen::Matrix4f p = globalPose;
        toLocal(p);

        matrix2Vector(p, x);

        // get entry
        return data->get(x, this);
    }


    Eigen::Matrix4f WorkspaceRepresentation::getPoseFromVoxel(unsigned int v[6], bool transformToGlobalPose)
    {
        float x[6];

        for (int j = 0; j < 6; j++)
        {
            x[j] = float(v[j]) + 0.5f;
        }

        return getPoseFromVoxel(x, transformToGlobalPose);
    }

    bool WorkspaceRepresentation::getPoseFromVoxel(unsigned int x[6], float v[6]) const
    {
        for (int i = 0; i < 6; i++)
        {
            if (/*(x[i] < 0) ||*/ (x[i] >= (unsigned int)(numVoxels[i])))
            {
                return false;
            }

            v[i] = ((((float) x[i]) * spaceSize[i]) / ((float)numVoxels[i]))  + minBounds[i];

            if (i < 3)
            {
                v[i] += discretizeStepTranslation / 2;
            }
            else
            {
                v[i] += discretizeStepRotation / 2;
            }
        }

        return true;
    }

    void WorkspaceRepresentation::invalidateBehindRobot(const bool inverted)
    {
        int step = 1;

        Eigen::Vector3f size;
        size(0) = spaceSize[0] / numVoxels[0];
        size(1) = spaceSize[1] / numVoxels[1];
        size(2) = spaceSize[2] / numVoxels[2];


        for (int a = 0; a < numVoxels[0]; a += step)
        {
            float voxelPositionX = minBounds[0] + (a + 0.5f) * size(0);

            for (int b = 0; b < numVoxels[1]; b += step)
            {

                float voxelPositionY = minBounds[1] + (b + 0.5f) * size(1);

                for(int c = 0; c < numVoxels[2]; c+= step)
                {
                    if(inverted)
                    {
                        if(voxelPositionX < 0)
                        {
                            data->reset(a,b,c);
                        }

                        // 45 deg to the front for the other hands workspace
                        // if(-voxelPositionY > voxelPositionX)
                        if(voxelPositionY < 0)
                        {
                            data->reset(a,b,c);
                        }

                    }else {

                        if(voxelPositionX > 0)
                        {
                            data->reset(a,b,c);
                        }

                         // 45 deg to the front for the other hands workspace
                        if(voxelPositionY < 0)
                        // if(-voxelPositionY > -voxelPositionX)
                        {
                            data->reset(a,b,c);
                        }
                    }
                }
            }
        }
    }


    WorkspaceRepresentation::VolumeInfo WorkspaceRepresentation::computeVolumeInformation()
    {
        WorkspaceRepresentation::VolumeInfo result;
        result.borderVoxelCount3D = 0;
        result.filledVoxelCount3D = 0;
        result.volume3D = 0;
        result.volumeFilledVoxels3D = 0;
        result.volumeVoxel3D = 0;
        result.voxelCount3D = numVoxels[0]*numVoxels[1]*numVoxels[2];
        for (int a = 0; a < numVoxels[0]; a++)
        {
            for (int b = 0; b < numVoxels[1]; b++)
            {
                for (int c = 0; c < numVoxels[2]; c++)
                {
                    int value = sumAngleReachabilities(a, b, c);

                    if (value > 0)
                    {
                        result.filledVoxelCount3D++;
                        if (a==0 || b==0 || c==0 || a==numVoxels[0]-1 || b==numVoxels[1]-1 || c==numVoxels[2]-1)
                            result.borderVoxelCount3D++;
                        else
                        {
                            int neighborEmptyCount = 0;
                            if (sumAngleReachabilities(a-1, b, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a+1, b, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b-1, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b+1, c)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b, c-1)==0)
                                neighborEmptyCount++;
                            if (sumAngleReachabilities(a, b, c+1)==0)
                                neighborEmptyCount++;
                            if (neighborEmptyCount>=1)
                                result.borderVoxelCount3D++;
                        }
                    }
                }
            }
        }
        double discrM = discretizeStepTranslation * 0.001;
        double voxelVolume = discrM * discrM * discrM;
        result.volumeVoxel3D = (float)voxelVolume;
        result.volumeFilledVoxels3D = (float)((double)result.filledVoxelCount3D * voxelVolume);
        result.volume3D = (float)(((double)result.filledVoxelCount3D - 0.5*(double)result.borderVoxelCount3D) * voxelVolume);

        return result;
    }

    Eigen::Matrix4f WorkspaceRepresentation::getPoseFromVoxel(float v[6], bool transformToGlobalPose /*= true*/)
    {
        float x[6];

        for (int j = 0; j < 6; j++)
        {
            x[j] = minBounds[j] + v[j] * getVoxelSize(j);
        }

        Eigen::Matrix4f m;
        vector2Matrix(x, m);

        //MathTools::posrpy2eigen4f(x,m);
        if (transformToGlobalPose)
        {
            toGlobal(m);
        }

        return m;
    }

    Eigen::Matrix4f WorkspaceRepresentation::sampleCoveredPose()
    {
        int maxLoops = 10000;
        int i = 0;
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        unsigned int nV[6];
        float x[6];

        while (i < maxLoops)
        {
            for (int j = 0; j < 6; j++)
            {
                nV[j] = rand() % numVoxels[j];
            }

            if (isCovered(nV))
            {
                // create pose

                for (int j = 0; j < 6; j++)
                {
                    x[j] = minBounds[j] + ((float)nV[j] + 0.5f) * getVoxelSize(j);
                }

                vector2Matrix(x, m);
                //MathTools::posrpy2eigen4f(x,m);
                toGlobal(m);
                return m;
            }

            i++;
        }

        VR_ERROR << "Could not find a valid pose?!" << std::endl;
        return m;
    }

    int WorkspaceRepresentation::fillHoles(unsigned int minNeighbors)
    {
        // copy data
        WorkspaceDataPtr newData(data->clone());

        unsigned int x[6];
        int res = 0;

        if (minNeighbors == 0)
        {
            minNeighbors = 1;
        }

        for (x[0] = 1; x[0] < (unsigned int)numVoxels[0] - 1; x[0]++)
            for (x[1] = 1; x[1] < (unsigned int)numVoxels[1] - 1; x[1]++)
                for (x[2] = 1; x[2] < (unsigned int)numVoxels[2] - 1; x[2]++)
                    for (x[3] = 1; x[3] < (unsigned int)numVoxels[3] - 1; x[3]++)
                        for (x[4] = 1; x[4] < (unsigned int)numVoxels[4] - 1; x[4]++)
                            for (x[5] = 1; x[5] < (unsigned int)numVoxels[5] - 1; x[5]++)
                            {
                                if (data->get(x) > 0)
                                {
                                    int sum = 0;
                                    int count = 0;

                                    for (unsigned int & i : x)
                                    {
                                        i--;

                                        if (isCovered(x))
                                        {
                                            sum += data->get(x);
                                            count++;
                                        }

                                        i++;
                                        i++;

                                        if (isCovered(x))
                                        {
                                            sum += data->get(x);
                                            count++;
                                        }

                                        i--;
                                    }

                                    if (count >= (int)minNeighbors)
                                    {
                                        res++;
                                        sum /= count;
                                        newData->setDatum(x, (unsigned char)sum);
                                    }


                                }
                            }

        data = newData;
        return res;
    }

    int WorkspaceRepresentation::getNumVoxels(int dim) const
    {
        VR_ASSERT((dim >= 0 && dim < 6));

        return numVoxels[dim];
    }

    float WorkspaceRepresentation::getMinBound(int dim) const
    {
        VR_ASSERT((dim >= 0 && dim < 6));

        return minBounds[dim];
    }

    float WorkspaceRepresentation::getMaxBound(int dim) const
    {
        VR_ASSERT((dim >= 0 && dim < 6));

        return maxBounds[dim];
    }

    unsigned char WorkspaceRepresentation::getVoxelEntry(unsigned int a, unsigned int b, unsigned int c, unsigned int d, unsigned int e, unsigned int f) const
    {
        if (/*a < 0 || b < 0 || c < 0 || d < 0 || e < 0 || f < 0 ||*/
                int(a) >= numVoxels[0] || int(b) >= numVoxels[1] || int(c) >= numVoxels[2] || int(d) >= numVoxels[3] || int(e) >= numVoxels[4] || int(f) >= numVoxels[5])
        {
            return 0;
        }

        return data->get(a, b, c, d, e, f);
    }

    int WorkspaceRepresentation::getMaxSummedAngleReachablity()
    {
        int maxValue = 0;

        for (int a = 0; a < getNumVoxels(0); a += 1)
        {
            for (int b = 0; b < getNumVoxels(1); b += 1)
            {
                for (int c = 0; c < getNumVoxels(2); c += 1)
                {
                    int value = sumAngleReachabilities(a, b, c);

                    if (value >= maxValue)
                    {
                        maxValue = value;
                    }
                }
            }
        }

        return maxValue;
    }

    bool WorkspaceRepresentation::isCovered(const Eigen::Matrix4f& globalPose)
    {
        return (getEntry(globalPose) > 0);
    }


    bool WorkspaceRepresentation::isCovered(unsigned int v[6])
    {
        return (data->get(v) > 0);
    }


    void WorkspaceRepresentation::setVoxelEntry(unsigned int v[6], unsigned char e)
    {
        data->setDatum(v, e);
        buildUpLoops++;
    }

    void WorkspaceRepresentation::setCurrentTCPPoseEntry(unsigned char e)
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "No WorkspaceRepresentation data loaded");

        Eigen::Matrix4f p = tcpNode->getGlobalPose();
        setEntry(p, e);

    }

    bool WorkspaceRepresentation::checkForParameters(RobotNodeSetPtr nodeSet, float steps, float storeMinBounds[6], float storeMaxBounds[6], RobotNodePtr baseNode, RobotNodePtr tcpNode)
    {
        if (!robot || !nodeSet || !nodeSet->isKinematicChain())
        {
            VR_WARNING << "invalid data" << std::endl;
            return false;
        }

        if (!tcpNode)
        {
            tcpNode = nodeSet->getTCP();
        }

        if (!robot->hasRobotNode(tcpNode))
        {
            VR_ERROR << "robot does not know tcp:" << tcpNode->getName() << std::endl;
            return false;
        }

        if (baseNode && !robot->hasRobotNode(baseNode))
        {
            VR_ERROR << "robot does not know baseNode:" << baseNode->getName() << std::endl;
            return false;
        }

        for (int i = 0; i < 6; i++)
        {
            storeMinBounds[i] = FLT_MAX;
            storeMaxBounds[i] = -FLT_MAX;
        }

        Eigen::VectorXf c;
        nodeSet->getJointValues(c);
        bool visuSate = robot->getUpdateVisualizationStatus();
        robot->setUpdateVisualization(false);

        // toLocal uses this->baseNode!
        RobotNodePtr tmpBase = this->baseNode;
        this->baseNode = baseNode;

        for (int i = 0; i < steps; i++)
        {
            setRobotNodesToRandomConfig(nodeSet, false);
            Eigen::Matrix4f p = tcpNode->getGlobalPose();
            toLocal(p);

            float x[6];
            matrix2Vector(p, x);

            // check for achieved values
            for (int i = 0; i < 6; i++)
            {
                if (x[i] < storeMinBounds[i])
                {
                    storeMinBounds[i] = x[i];
                }

                if (x[i] > storeMaxBounds[i])
                {
                    storeMaxBounds[i] = x[i];
                }
            }
        }

        robot->setJointValues(nodeSet, c);

        robot->setUpdateVisualization(visuSate);

        // assume higher values
        for (int i = 0; i < 6; i++)
        {
            float sizex = storeMaxBounds[i] - storeMinBounds[i];
            float factor = 0.2f;

            if (i > 2)
            {
                factor = 0.05f;    // adjustment for rotation is smaller
            }

            storeMinBounds[i] -= sizex * factor;
            storeMaxBounds[i] += sizex * factor;
        }

        this->baseNode = tmpBase;
        return true;

    }

    WorkspaceRepresentation::WorkspaceCut2DPtr WorkspaceRepresentation::createCut(const Eigen::Matrix4f& referencePose, float cellSize, bool sumAngles) const
    {
        WorkspaceCut2DPtr result(new WorkspaceCut2D());
        result->referenceGlobalPose = referencePose;

        Eigen::Vector3f minBB, maxBB;

        getWorkspaceExtends(minBB, maxBB);
        result->minBounds[0] = minBB(0);
        result->maxBounds[0] = maxBB(0);
        result->minBounds[1] = minBB(1);
        result->maxBounds[1] = maxBB(1);
        result->minBounds[2] = minBB(2);
        result->maxBounds[2] = maxBB(2);

        THROW_VR_EXCEPTION_IF(cellSize <= 0.0f, "Invalid parameter");

        float sizeX = result->maxBounds[0] - result->minBounds[0];
        int numVoxelsX = (int)(sizeX / cellSize);
        float sizeY = result->maxBounds[1] - result->minBounds[1];
        int numVoxelsY = (int)(sizeY / cellSize);
        float sizeZ = result->maxBounds[2] - result->minBounds[2];
        int numVoxelsZ = (int)(sizeZ / cellSize);

        Eigen::Matrix4f tmpPose = referencePose;
        Eigen::Matrix4f localPose;

        float x[6];
        unsigned int v[6];

        result->entries.resize(numVoxelsZ, numVoxelsY);

        // determine voxel range we want to cut
        // lower and higher bound of x direction of bins
        float xLower = referencePose(0,3);
        float xUpper = xLower + 153.0;
        // hardcode each bin's bound in z and y direction
        std::vector<float> zBounds = {1043.85, 1158.15, 1380.4, 1507.5, 1659.8};
        std::vector<float> yBounds = {-457.2, -228.6, 0.0, 228.6, 457.2};
        result->entries.setZero();
        float zLower = 1043.85;//704.85;
        float zUpper = 1659.8;//1320.8;
        float yLower = -457.2;
        float yUpper = 457.2;
        // resulting 4 by 4 bin reachability matrix
        WorkspaceCut2DPtr binReach(new WorkspaceCut2D());
        // Eigen::Matrix4f binReach;
        binReach->entries.resize(4,4);
        binReach->entries.setZero();
        std::cout << "cell size is: " << cellSize <<std::endl;

        // loop through all voxels in z, y, and x
        // tmpPose is the center position of each voxel
        for (int a = 0; a < numVoxelsZ; a++)
        {
            tmpPose(2, 3) = result->minBounds[2] + (float)a * cellSize + 0.5f * cellSize;
            if (tmpPose(2,3)  < zLower || tmpPose(2,3)  > zUpper) { continue; }
            for (int b = 0; b < numVoxelsY; b++)
            {
                tmpPose(1, 3) = result->minBounds[1] + (float)b * cellSize + 0.5f * cellSize;
                if (tmpPose(1,3) < yLower || tmpPose(1,3) > yUpper) { continue; }
                for (int c = (int)(tmpPose(0,3) / cellSize); c < numVoxelsX; c++) {
                    tmpPose(0, 3) = result->minBounds[0] + (float)c * cellSize + 0.5*cellSize;
                    if (tmpPose(0,3) < xLower || tmpPose(0,3) > xUpper) { continue; }

                    if (sumAngles)
                    {
                        // store voxel(a,b)'s reachability in result->entries
                        localPose = tmpPose;
                        toLocal(localPose);
                        matrix2Vector(localPose,x);
                        if (!getVoxelFromPose(x, v))
                        {
                            result->entries(a, b) = 0;

                        } else {
                            result->entries(a, b) = sumAngleReachabilities(v[0],v[1],v[2]);
                        }


                    } else
                    {
                        result->entries(a, b) = getEntry(tmpPose);
                    }
                    // loop through our 4 * 4 matrix to sum all voxels's reachability
                    // inside a bin as the bin's reachability
                    for(int i=0; i < 4; i++){
                        if (tmpPose(1,3) >= yBounds[i] && tmpPose(1,3) <= yBounds[i+1]){
                            for (int j=0; j < 4; j++) {
                                if (tmpPose(2,3) >= zBounds[j] && tmpPose(2,3) <= zBounds[j+1]) {
                                    binReach->entries(3-j, 3-i) = binReach->entries(4-j-1, 4-i-1) + result->entries(a, b);
                                }
                            }
                        }
                    }
                }
            }
        }

        // print our 4 by 4 matrix
        // std::cout << "bin reach is: \n" << binReach << std::endl;
        return result;
        // return binReach;
    }

  /*
    WorkspaceRepresentation::WorkspaceCut2DPtr WorkspaceRepresentation::createCut(Eigen::Matrix4f gp, float cellSize) const
    {
        result->referenceGlobalPose = refPose;

        Eigen::Matrix4f tmpPose = refPose;
        float x[6];
        unsigned int v[6];

        for (int a = 0; a < numVoxelsX; a++)
        {
            tmpPose(0, 3) = result->minBounds[0] + (float)a * cellSize + 0.5f * cellSize;

            for (int b = 0; b < numVoxelsY; b++)
            {
                tmpPose(1, 3) = result->minBounds[1] + (float)b * cellSize + 0.5f * cellSize;
                localPose = tmpPose;
                toLocal(localPose);
                matrix2Vector(localPose,x);

                if (!getVoxelFromPose(x, v))
                {
                    //result->entries(a, b) = 0;
                    continue;
                }
                result->entries(a, b) = sumAngleReachabilities(v[0],v[1],v[2]);
            }
        }

        return result;

    }*/

    WorkspaceRepresentation::WorkspaceCut2DPtr WorkspaceRepresentation::createCut(float heightPercent, float cellSize, bool sumAngles) const
    {
        THROW_VR_EXCEPTION_IF(cellSize <= 0.0f, "Invalid parameter");
        THROW_VR_EXCEPTION_IF(heightPercent < 0.0f || heightPercent>1.0f, "Invalid parameter");

        WorkspaceCut2DPtr result(new WorkspaceCut2D());

        Eigen::Vector3f minBB, maxBB;

        getWorkspaceExtends(minBB, maxBB);
        result->minBounds[0] = minBB(0);
        result->maxBounds[0] = maxBB(0);
        result->minBounds[1] = minBB(1);
        result->maxBounds[1] = maxBB(1);
        result->minBounds[2] = minBB(2);
        result->maxBounds[2] = maxBB(2);

        float sizeX = result->maxBounds[0] - result->minBounds[0];
        int numVoxelsX = (int)(sizeX / cellSize);
        float sizeY = result->maxBounds[1] - result->minBounds[1];
        int numVoxelsY = (int)(sizeY / cellSize);
        float sizeZ = result->maxBounds[2] - result->minBounds[2];
        int numVoxelsZ = (int)(sizeZ / cellSize);

        float sizeZGlobal = maxBB(2) - minBB(2);
        float poseZGlobal = minBB(2) + heightPercent*sizeZGlobal;

        float sizeXGlobal = maxBB(0) - minBB(0);
        float poseXGlobal = minBB(0) + heightPercent*sizeXGlobal;
        float sizeYGlobal = maxBB(1) - minBB(1);
        float poseYGlobal = minBB(1) + heightPercent*sizeYGlobal;

        // std::cout << "sizex global is : \n" << sizeXGlobal << std::endl;
        // std::cout << "sizey global is : \n" << sizeYGlobal << std::endl;

        result->entries.resize(numVoxelsZ, numVoxelsY);
        result->entries.setZero();

        Eigen::Matrix4f refPose = getToGlobalTransformation();
        Eigen::Matrix4f refPose_2 = getToGlobalTransformation();
        RobotNodePtr pod = this->robot->getRobotNode("my_random_joint_x");
        Eigen::Matrix4f p = pod->getGlobalPose();
        std::cout << "pod pose is :" << p << std::endl;
        std::cout << "baseNode is :" << baseNode->getName() << std::endl;
        // std::cout << "baseNode local pose is :\n" << getToLocalTransformation() << std::endl;
        std::cout << "refPose is : \n" << refPose << std::endl;
        // refPose_2(2,3) = poseZGlobal;
        // WorkspaceRepresentation::WorkspaceCut2DPtr x_y_result = createCut(refPose_2, cellSize, sumAngles);
        refPose(0,3) = p(0,3);
        refPose(1,3) = p(1,3);
        // std::cout << "posZGlobal is : " << poseZGlobal << std::endl;
        // std::cout << "refPose 2 and 3 now should equal to posZGlobal and is : \n" << refPose << std::endl;
        return createCut(refPose, cellSize, sumAngles);
    }

    bool WorkspaceRepresentation::getWorkspaceExtends(Eigen::Vector3f& storeMinBBox, Eigen::Vector3f& storeMaxBBox) const
    {
        Eigen::Vector3f quadPos[8];
        float x, y, z;

        for (int i = 0; i < 8; i++)
        {
            if (i % 2 == 0)
            {
                x = minBounds[0];
            }
            else
            {
                x = maxBounds[0];
            }

            if ((i >> 1) % 2 == 0)
            {
                y = minBounds[1];
            }
            else
            {
                y = maxBounds[1];
            }

            if ((i >> 2) % 2 == 0)
            {
                z = minBounds[2];
            }
            else
            {
                z = maxBounds[2];
            }

            quadPos[i](0) = x;
            quadPos[i](1) = y;
            quadPos[i](2) = z;
            toGlobalVec(quadPos[i]);
        }

        storeMinBBox = quadPos[0];
        storeMaxBBox = quadPos[0];

        for (auto & quadPo : quadPos)
        {
            for (int i = 0; i < 3; i++)
            {
                if (quadPo(i) < storeMinBBox(i))
                {
                    storeMinBBox(i) = quadPo(i);
                }

                if (quadPo(i) > storeMaxBBox(i))
                {
                    storeMaxBBox(i) = quadPo(i);
                }
            }
        }

        return true;
    }

    std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> WorkspaceRepresentation::createCutTransformations(WorkspaceRepresentation::WorkspaceCut2DPtr cutXY, RobotNodePtr referenceNode, const float maxAngle)
    {
        THROW_VR_EXCEPTION_IF(!cutXY, "NULL data");
        (void) maxAngle;  // Unused.

        std::vector<WorkspaceRepresentation::WorkspaceCut2DTransformationPtr> result;

        //float x,y,z;
        //z = cutXY->referenceGlobalPose(2,3);

        int nX = cutXY->entries.rows();
        int nY = cutXY->entries.cols();

        float sizeX = (cutXY->maxBounds[0] - cutXY->minBounds[0]) / (float)nX;
        float sizeY = (cutXY->maxBounds[1] - cutXY->minBounds[1]) / (float)nY;

        for (int x = 0; x < nX; x++)
        {
            for (int y = 0; y < nY; y++)
            {
                int v = cutXY->entries(x, y);

                if (v > 0)
                {
                    WorkspaceCut2DTransformationPtr tp(new WorkspaceCut2DTransformation());
                    tp->value = v;
                    float xPos = cutXY->minBounds[0] + (float)x * sizeX + 0.5f * sizeX; // center of voxel
                    float yPos = cutXY->minBounds[1] + (float)y * sizeY + 0.5f * sizeY; // center of voxel
                    tp->transformation = cutXY->referenceGlobalPose;
                    tp->transformation(0, 3) = xPos;
                    tp->transformation(1, 3) = yPos;

                    if (referenceNode)
                    {
                        tp->transformation = referenceNode->toLocalCoordinateSystem(tp->transformation);
                    }

                    Eigen::Isometry3f pose(tp->transformation);

                    float yaw = std::atan2(pose.translation().y(), pose.translation().x()) - M_PI_2f32;
                    yaw = simox::math::periodic_clamp(yaw, -M_PIf32, M_PIf32);

                    // if (yaw < maxAngle and yaw > -maxAngle)
                    // {
                        result.push_back(tp);
                    // }
                }
            }
        }

        return result;
    }

    float WorkspaceRepresentation::getDiscretizeParameterTranslation()
    {
        return discretizeStepTranslation;
    }

    float WorkspaceRepresentation::getDiscretizeParameterRotation()
    {
        return discretizeStepRotation;
    }

    void WorkspaceRepresentation::addCurrentTCPPose()
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "No reachability data loaded");
        Eigen::Matrix4f p = tcpNode->getGlobalPose();
        // std::cout << "current pose is: \n" << p << std::endl;
        // const auto& jointValues = nodeSet->getJointValues();
        // std::cout << "joint values are: \n[ ";
        // for (const auto& value : jointValues)
        //     std::cout << value << ' ';
        // std::cout << "]\n";
        addPose(p);
    }

    void WorkspaceRepresentation::setEntry(const Eigen::Matrix4f& poseGlobal, unsigned char e)
    {
        setEntryCheckNeighbors(poseGlobal, e, 0);
    }

    void WorkspaceRepresentation::setEntryCheckNeighbors(const Eigen::Matrix4f& poseGlobal, unsigned char e, unsigned int neighborVoxels)
    {
        Eigen::Matrix4f p = poseGlobal;
        toLocal(p);

        float x[6];
        matrix2Vector(p, x);
        //MathTools::eigen4f2rpy(p,x);

        // check for achieved values
        for (int i = 0; i < 6; i++)
        {
            if (x[i] < achievedMinValues[i])
            {
                achievedMinValues[i] = x[i];
            }

            if (x[i] > achievedMaxValues[i])
            {
                achievedMaxValues[i] = x[i];
            }
        }

        // get voxels
        unsigned int v[6];

        if (getVoxelFromPose(x, v))
        {

            data->setDatumCheckNeighbors(v, e, neighborVoxels);
        }

        buildUpLoops++;

    }

    MathTools::OOBB WorkspaceRepresentation::getOOBB(bool achievedValues) const
    {
        Eigen::Vector3f minBB;
        Eigen::Vector3f maxBB;

        if (achievedValues)
        {
            minBB << achievedMinValues[0], achievedMinValues[1], achievedMinValues[2];
            maxBB << achievedMaxValues[0], achievedMaxValues[1], achievedMaxValues[2];
        }
        else
        {
            minBB << minBounds[0], minBounds[1], minBounds[2];
            maxBB << maxBounds[0], maxBounds[1], maxBounds[2];
        }

        MathTools::OOBB oobb(minBB, maxBB, getToGlobalTransformation());
        return oobb;
    }

    void WorkspaceRepresentation::clear()
    {
        data->clear();
        buildUpLoops = 0;
        collisionConfigs = 0;

        for (int i = 0; i < 6; i++)
        {
            achievedMinValues[i] = FLT_MAX;
            achievedMaxValues[i] = -FLT_MAX;
        }
    }

    bool WorkspaceRepresentation::hasEntry(unsigned int x, unsigned int y, unsigned int z)
    {
        return data->hasEntry(x, y, z);
    }

    void WorkspaceRepresentation::matrix2Vector(const Eigen::Matrix4f& m, float x[6]) const
    {
        switch (orientationType)
        {
            case EulerXYZ:
            {
                x[0] = m(0, 3);
                x[1] = m(1, 3);
                x[2] = m(2, 3);

                Eigen::Matrix3f m_3 = m.block(0, 0, 3, 3);
                Eigen::Vector3f rotEulerxyz = m_3.eulerAngles(0, 1, 2);

                // intrinsic rotation (x y z)
                x[3] = rotEulerxyz(0);
                x[4] = rotEulerxyz(1);
                x[5] = rotEulerxyz(2);
            }
            break;

            case EulerXYZExtrinsic:
            {
                x[0] = m(0, 3);
                x[1] = m(1, 3);
                x[2] = m(2, 3);

                Eigen::Matrix3f m_3 = m.block(0, 0, 3, 3);
                Eigen::Vector3f rotEulerxyz = m_3.eulerAngles(0, 1, 2);

                // extrinsic (fixed coord system) rotation (x y z)
                x[5] = rotEulerxyz(0);
                x[4] = rotEulerxyz(1);
                x[3] = rotEulerxyz(2);
            }
            break;

            case RPY:
            {
                MathTools::eigen4f2rpy(m, x);
            }
            break;
            case Hopf:
            {
                MathTools::Quaternion q = MathTools::eigen4f2quat(m);
                Eigen::Vector3f h = MathTools::quat2hopf(q);
                x[0] = m(0,3);
                x[1] = m(1,3);
                x[2] = m(2,3);
                x[3] = h(0);
                x[4] = h(1);
                x[5] = h(2);
            }
            break;

            default:
                THROW_VR_EXCEPTION("mode nyi...");
        }
    }
    /*
    works
    Eigen::Vector3f rotEulerxyz = m_3.eulerAngles(0,1,2);
    x[3] = rotEulerxyz(0);
    x[4] = rotEulerxyz(1);
    x[5] = rotEulerxyz(2);
    m_3 =  Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
    m.block(0,0,3,3) = m_3;
    == OR ==
    float s1 = sin(x[3]);float s2 = sin(x[4]);float s3 = sin(x[5]);
    float c1 = cos(x[3]);float c2 = cos(x[4]);float c3 = cos(x[5]);
    // Euler XYZ
    m_3(0,0) =  c2*c3;               m_3(0,1) =  -c2*s3;              m_3(0,2) =  s2;
    m_3(1,0) =  c1*s3+c3*s1*s2;      m_3(1,1) =  c1*c3-s1*s2*s3;      m_3(1,2) =  -c2*s1;
    m_3(2,0) =  s1*s3-c1*c3*s2;      m_3(2,1) =  c3*s1+c1*s2*s3;      m_3(2,2) =  c1*c2;
    m.block(0,0,3,3) = m_3;
    */
    void WorkspaceRepresentation::vector2Matrix(const float x[6], Eigen::Matrix4f& m) const
    {
        switch (orientationType)
        {
            case EulerXYZ:
            {
                m.setIdentity();
                m(0, 3) = x[0];
                m(1, 3) = x[1];
                m(2, 3) = x[2];
                Eigen::Matrix3f m_3;
                m_3 = Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitX())
                      * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitZ());
                /*m_3 = Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitX());*/
                /*
                float s1 = sin(x[3]);float s2 = sin(x[4]);float s3 = sin(x[5]);
                float c1 = cos(x[3]);float c2 = cos(x[4]);float c3 = cos(x[5]);
                // Euler XYZ
                m_3(0,0) =  c2*c3;               m_3(0,1) =  -c2*s3;              m_3(0,2) =  s2;
                m_3(1,0) =  c1*s3+c3*s1*s2;      m_3(1,1) =  c1*c3-s1*s2*s3;      m_3(1,2) =  -c2*s1;
                m_3(2,0) =  s1*s3-c1*c3*s2;      m_3(2,1) =  c3*s1+c1*s2*s3;      m_3(2,2) =  c1*c2;
                */
                /*
                // Euler ZYX
                m_3(0,0) =  c1*c2;    m_3(0,1) =  c1*s2*s3-c3*s1;   m_3(0,2) =  s1*s3+c1*c3*s2;
                m_3(1,0) =  c2*s1;    m_3(1,1) =  c1*c3+s1*s2*s3;   m_3(1,2) =  c3*s1*s2-c1*s3;
                m_3(2,0) =  -s2;      m_3(2,1) =  c2*s3;            m_3(2,2) =  c2*c3;
                */

                m.block(0, 0, 3, 3) = m_3;
            }
            break;

            case EulerXYZExtrinsic:
            {
                m.setIdentity();
                m(0, 3) = x[0];
                m(1, 3) = x[1];
                m(2, 3) = x[2];
                Eigen::Matrix3f m_3;
                m_3 = Eigen::AngleAxisf(x[5], Eigen::Vector3f::UnitX())
                      * Eigen::AngleAxisf(x[4], Eigen::Vector3f::UnitY())
                      * Eigen::AngleAxisf(x[3], Eigen::Vector3f::UnitZ());
                m.block(0, 0, 3, 3) = m_3;
            }
            break;

            case RPY:
            {
                MathTools::posrpy2eigen4f(x, m);
            }
            break;


            case Hopf:
            {
                Eigen::Vector3f h;
                h << x[3],x[4],x[5];
                MathTools::Quaternion q = MathTools::hopf2quat(h);
                m = MathTools::quat2eigen4f(q);
                m(0, 3) = x[0];
                m(1, 3) = x[1];
                m(2, 3) = x[2];
            }
            break;
            default:
                THROW_VR_EXCEPTION("mode nyi...");
        }
    }

    void WorkspaceRepresentation::vector2Matrix(const Eigen::Vector3f& pos, const Eigen::Vector3f& rot, Eigen::Matrix4f& m) const
    {
        float x[6];
        x[0] = pos[0];
        x[1] = pos[1];
        x[2] = pos[2];
        x[3] = rot[0];
        x[4] = rot[1];
        x[5] = rot[2];
        vector2Matrix(x, m);
    }

    void WorkspaceRepresentation::setOrientationType(eOrientationType t)
    {
        orientationType = t;
    }

    VirtualRobot::WorkspaceRepresentationPtr WorkspaceRepresentation::clone()
    {
        VirtualRobot::WorkspaceRepresentationPtr res(new WorkspaceRepresentation(robot));
        res->setOrientationType(this->orientationType);
        res->versionMajor = this->versionMajor;
        res->versionMinor = this->versionMinor;
        res->nodeSet = this->nodeSet;
        res->type = this->type;

        res->baseNode = this->baseNode;
        res->tcpNode = this->tcpNode;
        res->staticCollisionModel = this->staticCollisionModel;
        res->dynamicCollisionModel = this->dynamicCollisionModel;
        res->buildUpLoops = this->buildUpLoops;
        res->collisionConfigs = this->collisionConfigs;
        res->discretizeStepTranslation = this->discretizeStepTranslation;
        res->discretizeStepRotation = this->discretizeStepRotation;
        memcpy(res->minBounds, this->minBounds, sizeof(float) * 6);
        memcpy(res->maxBounds, this->maxBounds, sizeof(float) * 6);
        memcpy(res->numVoxels, this->numVoxels, sizeof(float) * 6);
        memcpy(res->achievedMinValues, this->achievedMinValues, sizeof(float) * 6);
        memcpy(res->achievedMaxValues, this->achievedMaxValues, sizeof(float) * 6);
        memcpy(res->spaceSize, this->spaceSize, sizeof(float) * 6);

        res->adjustOnOverflow = this->adjustOnOverflow;
        res->data.reset(this->data->clone());

        return res;
    }

    VirtualRobot::WorkspaceDataPtr WorkspaceRepresentation::getData()
    {
        return data;
    }

    void WorkspaceRepresentation::addRandomTCPPoses(unsigned int loops, bool checkForSelfCollisions)
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "Workspace data not initialized");

        std::vector<float> c;
        nodeSet->getJointValues(c);
        bool visuSate = robot->getUpdateVisualizationStatus();
        robot->setUpdateVisualization(false);

        // resulting bin reachability and a normal vector pointing to pod
        Eigen::Matrix4f bin_reach;
        bin_reach.setZero();
        Eigen::Vector3f norm_vec(1.0f, 0.0f, 0.0f);
        // hardcode each bin's bound in x, y, z directions
        // xBounds are based on the pose of the pod which depends on the user input
        RobotNodePtr pod = this->robot->getRobotNode("my_random_joint_x");
        Eigen::Matrix4f pod_pose = pod->getGlobalPose();
        std::vector<float> xBounds = {pod_pose(0,3), pod_pose(0,3) + 153.0f};
        std::vector<float> yBounds = {-457.2f, -228.6f, 0.0f, 228.6f, 457.2f};
        std::vector<float> zBounds = {1043.85f, 1158.15f, 1380.4f, 1507.5f, 1659.8f};

        for (unsigned int i = 0; i < loops; i++)
        {
            if (setRobotNodesToRandomConfig(nodeSet, checkForSelfCollisions))
            {
                // compute weight of each pose and add to bin_reach
                Eigen::Matrix4f cur_pose = tcpNode->getGlobalPose();
                // Eigen::Matrix3f orientation_matrix = cur_pose.block<3,3>(0,0);
                Eigen::Vector3f orientation_matrix_column_3 = cur_pose.block<3,1>(0,2);
                float cos_theta = norm_vec.dot(orientation_matrix_column_3);
                if (cos_theta >= 0 && cur_pose(0, 3) > xBounds[0] && cur_pose(0, 3) < xBounds[1])
                {
                    // loop through y and z bounds
                    // if cur_pose falls inside, add weight to that bin
                    for (int i = 0; i < 4; i++) {
                        if (cur_pose(1, 3) >= yBounds[i] && cur_pose(1, 3) <= yBounds[i+1]){
                            for (int j = 0; j < 4; j++) {
                                if (cur_pose(2, 3) >= zBounds[j] && cur_pose(2, 3) <= zBounds[j+1]) {
                                    float weight = 1-((acos(cos_theta) * 180 / 3.141592) / 90);
                                    bin_reach(3-j, 3-i) += weight;
                                    // bin_reach(3-j, 3-i) += cos_theta;
                                    break;
                                }
                            }
                            break;
                        }
                    }
                }
                addCurrentTCPPose();
            }
            else
            {
                VR_WARNING << "Could not find collision-free configuration...";
            }
        }
        // std::cout << "bin reach is : \n" << bin_reach << std::endl;

        robot->setUpdateVisualization(visuSate);
        // nodeSet->setJointValues(c);
    }

    void WorkspaceRepresentation::addRandomTCPPoses(unsigned int loops, unsigned int numThreads, bool checkForSelfCollisions)
    {
        THROW_VR_EXCEPTION_IF(!data || !nodeSet || !tcpNode, "Workspace data not initialized");

        if (numThreads > loops)
        {
            VR_ERROR << "Number of threads can not be bigger then number of tcp poses to add.";
            return;
        }
        // std::cout << "+++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
        // std::cout << "Thread ==========" << std::endl;
        // std::cout << "number of poses: " << loops << std::endl;
        // std::cout << "self collisions is set to " << checkForSelfCollisions << std::endl;
        std::vector<std::thread> threads(numThreads);
        unsigned int numPosesPerThread = loops / numThreads; // todo

        for (unsigned int i = 0; i < numThreads; i++)
        {
            threads[i] = std::thread([=, this] ()
            {
                // each thread gets a cloned robot
                CollisionCheckerPtr cc(new CollisionChecker());
                RobotPtr clonedRobot = this->robot->clone("clonedRobot_" + std::to_string(i), cc);
                clonedRobot->setUpdateVisualization(false);
                RobotNodeSetPtr clonedNodeSet = clonedRobot->getRobotNodeSet(this->nodeSet->getName());
                RobotNodePtr clonedTcpNode = clonedRobot->getRobotNode(this->tcpNode->getName());

                SceneObjectSetPtr static_collision_model = this->staticCollisionModel;
                if (static_collision_model && clonedRobot->hasRobotNodeSet(static_collision_model->getName()))
                {
                    static_collision_model = clonedRobot->getRobotNodeSet(static_collision_model->getName());
                }

                SceneObjectSetPtr dynamic_collision_model = this->dynamicCollisionModel;
                if (dynamicCollisionModel && clonedRobot->hasRobotNodeSet(dynamicCollisionModel->getName()))
                {
                    dynamicCollisionModel = clonedRobot->getRobotNodeSet(dynamicCollisionModel->getName());
                }

                // now sample some configs and add them to the workspace data
                for (unsigned int j = 0; j < numPosesPerThread; j++)
                {
                    float rndValue;
                    float minJ, maxJ;
                    Eigen::VectorXf v(clonedNodeSet->getSize());
                    float maxLoops = 1000;

                    bool successfullyRandomized = false;

                    for (int k = 0; k < maxLoops; k++)
                    {
                        for (unsigned int l = 0; l < clonedNodeSet->getSize(); l++)
                        {
                            rndValue = RandomFloat(); // value from 0 to 1
                            minJ = (*nodeSet)[l]->getJointLimitLo();
                            maxJ = (*nodeSet)[l]->getJointLimitHi();
                            v[l] = minJ + ((maxJ - minJ) * rndValue);
                        }

                        clonedRobot->setJointValues(clonedNodeSet, v);

                        // check for collisions
                        if (!checkForSelfCollisions || !static_collision_model || !dynamicCollisionModel)
                        {
                            successfullyRandomized = true;
                            break;
                        }

                        if (!clonedRobot->getCollisionChecker()->checkCollision(static_collision_model, dynamicCollisionModel))
                        {
                            successfullyRandomized = true;
                            break;
                        }

                        this->collisionConfigs++;
                    }

                    if (successfullyRandomized)
                    {
                        Eigen::Matrix4f p = clonedTcpNode->getGlobalPose();
                        addPose(p);
                    }
                    else
                    {
                        VR_WARNING << "Could not find collision-free configuration...";
                    }
                }
            });
        }

        // wait for all threads to finish
        for (unsigned int i = 0; i < numThreads; i++)
        {
            threads[i].join();
        }
    }

} // namespace VirtualRobot
