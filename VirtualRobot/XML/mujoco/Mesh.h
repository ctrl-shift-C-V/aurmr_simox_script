/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    MujocoX::ArmarXObjects::Mesh
 * @author     Rainer Kartmann ( rainer dot kartmann at student dot kit dot edu )
 * @date       2019
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>


namespace VirtualRobot::mujoco
{
    class Mesh
    {
    public:

        static Mesh fromFile(const std::string& filename);

    public:

        /// Constructor.
        Mesh();

        /// Read from a file.
        void read(const std::string& filename);
        /// Read from an in stream.
        void read(std::istream& is);

        /// Write to a file.
        void write(const std::string& filename) const;
        /// Write to an out stream.
        void write(std::ostream& os) const;

        int32_t nvertex() const
        {
            return _nvertex;
        }
        int32_t nnormal() const
        {
            return _nnormal;
        }
        int32_t ntexcoord() const
        {
            return _ntexcoord;
        }
        int32_t nface() const
        {
            return _nface;
        }

        std::vector<float>& vertexPositions()
        {
            return _vertex_positions;
        }
        const std::vector<float>& vertexPositions() const
        {
            return _vertex_positions;
        }

        std::vector<float>& vertexNormals()
        {
            return _vertex_normals;
        }
        const std::vector<float>& vertexNormals() const
        {
            return _vertex_normals;
        }

        std::vector<float>& vertexTexcoords()
        {
            return _vertex_texcoords;
        }
        const std::vector<float>& vertexTexcoords() const
        {
            return _vertex_texcoords;
        }

        std::vector<int32_t>& faceVertexIndices()
        {
            return _face_vertex_indices;
        }
        const std::vector<int32_t>& faceVertexIndices() const
        {
            return _face_vertex_indices;
        }

        Eigen::Vector3f vertexPosition(int32_t i) const;
        Eigen::Vector3f vertexNormal(int32_t i) const;
        Eigen::Vector2f vertexTexcoord(int32_t i) const;
        Eigen::Vector3i faceVertexIndex(int32_t i) const;

        bool hasNormals() const
        {
            return _nnormal > 0;
        }
        bool hasTexcoords() const
        {
            return _ntexcoord > 0;
        }
        bool hasFaces() const
        {
            return _nface > 0;
        }


        void addVertexPosition(const Eigen::Vector3f& position);
        void addVertexNormal(const Eigen::Vector3f& normal);
        void addVertexTexcoord(const Eigen::Vector2f& texcoord);
        void addFaceVertexIndex(const Eigen::Vector3i& face);


        /**
         * @brief Get the bounding box of vertex positions.
         * The bounding box is returned as a 3x2 matrix:
         *    min   max
         * ( x_min x_max )  < x limits
         * ( y_min y_max )  < y limits
         * ( z_min z_max )  < z limits
         */
        Eigen::Matrix<float, 3, 2> getVertexPositionsBoundingBox() const;

        void setTextureFile(const std::string& path);
        const std::string& getTextureFile() const;
        
        /**
         * @brief Split this mesh into num meshes by randomly assigning its
         * faces to the new meshes.
         */
        std::vector<Mesh> splitRandomly(std::size_t num) const;
        
        void setFilename(const std::string& path);
        const std::string& getFilename() const;

        
    private:

        /// Resize the vectors to the according number of elements.
        /// _n* values must be nonnegative.
        void resizeVectors();

        int32_t _nvertex = 0;
        int32_t _nnormal = 0;
        int32_t _ntexcoord = 0;
        int32_t _nface = 0;

        std::vector<float> _vertex_positions;  // 3*nvertex
        std::vector<float> _vertex_normals;    // 3*nnormal
        std::vector<float> _vertex_texcoords;  // 2*ntexcoord
        std::vector<int32_t> _face_vertex_indices;  // 3*nface

        std::string _texture_file;
        std::string _filename;
    };
}
