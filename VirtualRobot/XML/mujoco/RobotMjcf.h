#pragma once

#include <filesystem>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MJCF/Document.h>


namespace VirtualRobot::mujoco
{

    /**
     * @brief The RobotMjcf class allows building a MuJoCo MJCF model from a 
     * VirtualRobot robot model.
     */
    class RobotMjcf
    {
    public:
        
        /// Construct with robot.
        RobotMjcf(RobotPtr robot);
        
        
        /// Get the document.
        mjcf::Document& getDocument();
        const mjcf::Document& getDocument() const;
        
        
        // PREPARATION
        
        /// Get the robot.
        RobotPtr getRobot() const;
        
        /// Get the body encapsulating the robot structure.
        mjcf::Body getRobotBody() const;
        
        /// Get the body of the given robot node, if it exists.
        /// @throw std::range_error If no body has been added for this node.
        mjcf::Body getRobotNodeBody(const std::string& nodeName) const;
        
        
        /// Set the path to the output XML file.
        void setOutputFile(const std::filesystem::path& filePath);
        
        
        // MODIFIERS
        
        // META
        
        /// Make a compiler section.
        void addCompiler(const std::string& angle = "radian", bool balanceIneratia = true);
        
        /// Make defaults class with robot's name.
        void addDefaultsClass(float meshScale = 1.0f);
        
        
        // ASSETS
        
        /// Add skybox texture.
        void addSkybox(const Eigen::Vector3f& rgb1 = { .8f, .9f, .95f }, 
                       const Eigen::Vector3f& rgb2 = { .4f, .6f, .80f });
        
        
        // ROBOT NODES
        
        /// Add a body for the given robot node as child of `parent`.
        mjcf::Body addNodeBody(RobotNodePtr node, mjcf::Body parent,
                               bool addJoint = true, bool addInertial = true);
        
        /// Add a joint for the given node in `body` and return it.
        mjcf::Joint addNodeJoint(RobotNodePtr node, mjcf::Body body);
        /// Add an inertial for the given node in `body` and return it.
        mjcf::Inertial addNodeInertial(RobotNodePtr node, mjcf::Body body);
        
        
        /**
         * @brief Add a body element for the given robot node.
         * If its parent does not exist, create the parent body first.
         * If `node` is the robot root node, it is attached to the robot body.
         * @param addJoint Whether to add a joint, if applicable.
         * @param addInertial Whether to add an inertial, if applicable.
         * @return The node body.
         */
        mjcf::Body addNodeBody(RobotNodePtr node, bool addJoint = true, bool addInertial = true);

        
        /// Add bodies for all robot nodes.
        void addNodeBodies();
        /// Add bodies for the specified nodes.
        void addNodeBodies(RobotNodeSetPtr nodeSet);
        /// Add bodies for the specified nodes.
        void addNodeBodies(const std::vector<std::string>& nodeNames);
        
        
        /**
         * @brief Add a mesh asset for the given node and a mesh geom to the node's body.
         * @param node The node.
         * @throws std::range_error If no body has been added for `node`.
         */
        void addNodeBodyMesh(RobotNodePtr node);
        
        /// Convert the mesh model of the given node to STL, which can be loaded by MuJoCo.
        void convertNodeMeshToSTL(RobotNodePtr node);
        
        static void convertNodeMeshToSTL(const std::filesystem::path& sourceFile,
                                         const std::filesystem::path& targetPath,
                                         bool skipIfExists = true);
        
        void addNodeBodyMeshes();
        void addNodeBodyMeshes(RobotNodeSetPtr nodeSet);
        void addNodeBodyMeshes(const std::vector<std::string>& nodeNames);
        
        
        
    private:

        /// The robot.
        RobotPtr robot;
        
        /// The document. Using a pointer allows to easily reset it.
        mjcf::DocumentPtr document { new mjcf::Document() };
        
        /// The path to the output XML file.
        std::filesystem::path outputFile;
        
        
        /// Scaling for lengths, such as positions and translations (to m).
        float lengthScale = 1.f;
        /// Scaling for lengths, such as positions and translations (to m).
        float meshScale = 1.f;
        /// Scaling for mass (to kg).
        float massScale = 1.f;
        
        
        /// The robot body. (Not part of the original robot structure.)
        mjcf::Body robotBody;
        
        /// Map of robot node names to XML elements.
        std::map<std::string, mjcf::Body> nodeBodies;
        
    };

}
