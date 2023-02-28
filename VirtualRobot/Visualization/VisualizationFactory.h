/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../VirtualRobot.h"
#include "../AbstractFactoryMethod.h"
#include "../MathTools.h"
#include "../BoundingBox.h"
#include "../Primitive.h"

#include <Eigen/Core>
#include <string>

namespace VirtualRobot
{
    class VisualizationNode;

    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationFactory  : public ::AbstractFactoryMethod<VisualizationFactory, void*>
    {
    public:

        struct Color
        {
            Color() = default;
            Color(float r, float g, float b, float transparency = 0.0f): r(r), g(g), b(b), transparency(transparency) {}
            float r = 0.5f, g = 0.5f, b = 0.5f;
            float transparency = 1;
            bool isNone() const
            {
                return transparency >= 1.0f;
            }
            static Color Blue(float transparency = 0.0f)
            {
                return Color(0.2f, 0.2f, 1.0f, transparency);
            }
            static Color Red(float transparency = 0.0f)
            {
                return Color(1.0f, 0.2f, 0.2f, transparency);
            }
            static Color Green(float transparency = 0.0f)
            {
                return Color(0.2f, 1.0f, 0.2f, transparency);
            }
            static Color Black(float transparency = 0.0f)
            {
                return Color(0, 0, 0, transparency);
            }
            static Color Gray()
            {
                return Color(0.5f, 0.5f, 0.5f, 0);
            }
            static Color None()
            {
                return Color(0.0f, 0.0f, 0.0f, 1.0f);
            }
            static Color CustomColor(float weight)
            {
                float blue = 1.0f - weight;
                float red = weight;
                return Color(red, 0.0f, blue, 0.0f);
            }
        };

        struct PhongMaterial
        {
            PhongMaterial() = default;
            Color emission;
            Color ambient;
            Color diffuse;
            Color specular;
            float shininess{0};
            Color reflective;
            float reflectivity{0};
            Color transparent;
            float transparency{0};
            float refractionIndex{0};
        };

        VisualizationFactory() = default;
        virtual ~VisualizationFactory() = default;

        virtual void init(int& /*argc*/, char* /*argv*/[], const std::string& /*appName*/)
        {
        }

        virtual VisualizationNodePtr getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& /*primitives*/, bool /*boundingBox*/ = false, Color /*color*/ = Color::Gray())
        {
            return nullptr;
        }
        virtual VisualizationNodePtr getVisualizationFromFile(const std::string& /*filename*/, bool /*boundingBox*/ = false, float /*scaleX*/ = 1.0f, float /*scaleY*/ = 1.0f, float /*scaleZ*/ = 1.0f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr getVisualizationFromFile(const std::ifstream& /*ifs*/, bool /*boundingBox*/ = false, float /*scaleX*/ = 1.0f, float /*scaleY*/ = 1.0f, float /*scaleZ*/ = 1.0f)
        {
            return nullptr;
        }
        /*!
            A box, dimensions are given in mm.
        */
        virtual VisualizationNodePtr createBox(float /*width*/, float /*height*/, float /*depth*/, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createLine(const Eigen::Vector3f& /*from*/, const Eigen::Vector3f& /*to*/, float /*width*/ = 1.0f, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createLine(const Eigen::Matrix4f& /*from*/, const Eigen::Matrix4f& /*to*/, float /*width*/ = 1.0f, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createSphere(float /*radius*/, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createCircle(float /*radius*/, float /*circleCompletion*/, float /*width*/, float /*colorR*/ = 1.0f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f, size_t /*numberOfCircleParts*/ = 30)
        {
            return nullptr;
        }

        virtual VisualizationNodePtr createTorus(float /*radius*/, float /*tubeRadius*/, float /*completion*/ = 1.0f, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f, float /*transparency*/ = 0.0f, int /*sides*/ = 8, int /*rings*/ = 30)
        {
            return nullptr;
        }

        virtual VisualizationNodePtr createCircleArrow(float /*radius*/, float /*tubeRadius*/, float /*completion*/ = 1, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f, float /*transparency*/ = 0.0f, int /*sides*/ = 8, int /*rings*/ = 30)
        {
            return nullptr;
        }

        virtual VisualizationNodePtr createCylinder(float /*radius*/, float /*height*/, float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createCoordSystem(float /*scaling*/ = 1.0f, std::string* /*text*/ = NULL, float /*axisLength*/ = 100.0f, float /*axisSize*/ = 3.0f, int /*nrOfBlocks*/ = 10)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createBoundingBox(const BoundingBox& /*bbox*/, bool /*wireFrame*/ = false)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f& /*position*/, float /*radius*/, float /*transparency*/,  float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }

        virtual VisualizationNodePtr createTriMeshModelVisualization(const TriMeshModelPtr& /*model*/, const Eigen::Matrix4f& /*pose*/, float /*scaleX*/ = 1.0f, float /*scaleY*/ = 1.0f, float /*scaleZ*/ = 1.0f)
        {
            return nullptr;
        }

        virtual VisualizationNodePtr createTriMeshModelVisualization(const TriMeshModelPtr& /*model*/, bool /*showNormals*/, const Eigen::Matrix4f& /*pose*/, bool /*showLines*/ = true)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createPlane(const Eigen::Vector3f& /*position*/, const Eigen::Vector3f& /*normal*/, float /*extend*/, float /*transparency*/,  float /*colorR*/ = 0.5f, float /*colorG*/ = 0.5f, float /*colorB*/ = 0.5f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createPlane(const MathTools::Plane& plane, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return createPlane(plane.p, plane.n, extend, transparency, colorR, colorG, colorB);
        }
        virtual VisualizationNodePtr createArrow(const Eigen::Vector3f& /*n*/, float /*length*/ = 50.0f, float /*width*/ = 2.0f, const Color& /*color*/ = Color::Gray())
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createTrajectory(TrajectoryPtr /*t*/, Color /*colorNode*/ = Color::Blue(), Color /*colorLine*/ = Color::Gray(), float /*nodeSize*/ = 15.0f, float /*lineSize*/ = 4.0f)
        {
            return nullptr;
        }
        virtual VisualizationNodePtr createText(const std::string& /*text*/, bool /*billboard*/ = false, float /*scaling*/ = 1.0f, Color /*c*/ = Color::Black(), float /*offsetX*/ = 20.0f, float /*offsetY*/ = 20.0f, float /*offsetZ*/ = 0.0f)
        {
            return nullptr;
        }
        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \param showAxes If true, the axes are visualized
            \param axesHeight The height of the axes (measured from the body surface)
            \param axesWidth The width of the axes.
            \return A VisualizationNode containing the visualization.
        */
        virtual VisualizationNodePtr createEllipse(float /*x*/, float /*y*/, float /*z*/, bool /*showAxes*/ = true, float /*axesHeight*/ = 4.0f, float /*axesWidth*/ = 8.0f)
        {
            return nullptr;
        }
        /*!
            Move local visualization by homogeneous matrix m. (MM)
        */
        virtual void applyDisplacement(VisualizationNodePtr /*o*/, Eigen::Matrix4f& /*m*/) {}

        /*!
            Create an empty VisualizationNode.
        */
        virtual VisualizationNodePtr createVisualization()
        {
            return nullptr;
        }

        /*!
            Create a united visualization.
        */
        virtual VisualizationNodePtr createUnitedVisualization(const std::vector<VisualizationNodePtr>& /*visualizations*/) const
        {
            return nullptr;
        }

        /*!
            Here, a manual cleanup can be called, no Coin3D access possible after this.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup() {}

    };
    typedef std::shared_ptr<VisualizationFactory::Color> ColorPtr;

} // namespace VirtualRobot

