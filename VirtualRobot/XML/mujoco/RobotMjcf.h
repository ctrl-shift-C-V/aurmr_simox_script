#pragma once

#include <filesystem>

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/MJCF/Document.h>

#include "exceptions.h"


namespace VirtualRobot::mujoco
{

    /// Actuator type.
    enum class ActuatorType
    {
        MOTOR, POSITION, VELOCITY,
    };
    ActuatorType toActuatorType(const std::string& string);
    

    /**
     * @brief The RobotMjcf class allows building a MuJoCo MJCF model from a 
     * VirtualRobot robot model.
     */
    class RobotMjcf
    {
    public:
        
        /// Construct with robot.
        RobotMjcf(RobotPtr robot);
        
        
        /// Reset the document and related data.
        void reset();
        
        /// Get the document.
        mjcf::Document& getDocument();
        const mjcf::Document& getDocument() const;
        
        
        // ELEMENT ACCESS
        
        /// Get the robot.
        RobotPtr getRobot() const;
        
        /// Get the body encapsulating the robot structure.
        mjcf::Body getRobotBody() const;
        
        
        /**
         * @brief Get the body of the given robot node, if it exists.
         * @throw error::NoBodyOfRobotNode If there is no body for this node.
         */
        mjcf::Body getRobotNodeBody(const std::string& nodeName) const;
        
        /// Indicate whether a body for the given node exists.
        bool hasRobotNodeBody(const std::string& nodeName) const;

        
        // PREPARATION
        
        /// Set the path to the output XML file.
        void setOutputFile(const std::filesystem::path& filePath);
        
        /// Set the path to the directory where meshes shall be stored.
        void setOutputMeshDirectory(const std::filesystem::path& path);
        
        
        // MODIFIERS
        
        // META
        
        /**
         * @brief Fill the compiler section.
         * @param angleRadian If true, set `angle` to "radian", otherwise to "degree".
         * @param boundMass If true, set `boundmass` to `document->getDummyMass()`.
         * @param balanceIneratia Value for `balanceinertia`.
         */
        void addCompiler(bool angleRadian = true,
                         bool boundMass = true,
                         bool balanceIneratia = true);
        
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
        
        /**
         * @brief Add a joint for the given node in `body`.
         * @throw error::NodeIsNoJoint If `node` is not a joint node.
         */
        mjcf::Joint addNodeJoint(RobotNodePtr node, mjcf::Body body);
        
        /// Add an inertial for the given node in `body`.
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
         * @throws error::NoBodyOfRobotNode()
         */
        void addNodeBodyMesh(RobotNodePtr node);
        
        /**
         * @brief Convert the mesh model of the given node to STL, which can 
         * be loaded by MuJoCo.
         * @return The path to the resulting STL. On error, returns an empty path.
         */
        std::filesystem::path convertNodeMeshToSTL(RobotNodePtr node);
        
        /// Add meshes to all node bodies.
        void addNodeBodyMeshes();
        /// Add meshes to the specified nodes.
        void addNodeBodyMeshes(RobotNodeSetPtr nodeSet);
        /// Add meshes to the specified nodes.
        void addNodeBodyMeshes(const std::vector<std::string>& nodeNames);
        
        
        /**
         * @brief Add a mocap body without a weld constraint.
         * 
         * If `className` is not empty, adds class defaults for geom/RGBA.
         * 
         * @param bodyName  Name of the mocap body.
         * @param className The mocap defaults class name.
         * @param geomSize  Size of the mocap's geom.
         * @return The mocap body.
         */
        mjcf::Body addMocapBody(const std::string& bodyName,
                                const std::string& className = "mocap",
                                float geomSize = 0.01f);
        
        /**
         * @brief Add a mocap body and a weld constraint to `weldBodyName`.
         * 
         * In addition to the mocap body, adds an equality weld constraint
         * as well as a contact exclude between the mocap body and the welded
         * body. 
         * 
         * If `className` is not empty, adds equality defaults.
         * 
         * @param weldBodyName Name of the body to be welded to the mocap body.
         * @param bodyName     Name of the mocap body.
         * @param className    Name of mocap defaults class.
         * @param geomSize     Size of the mocap's geom.
         * @return The mocap body.
         */
        mjcf::Body addMocapBodyWeld(const std::string& weldBodyName,
                                    const std::string& bodyName,
                                    const std::string& className = "mocap",
                                    float geomSize = 0.01f);
        
        /**
         * @brief Add a mocap body and a weld constraint to the robot body.
         * @param bodyName Name of the mocap body. If left empty, is derived from the robot name.
         * @see addMocapBodyWeld()
         */
        mjcf::Body addMocapBodyWeldRobot(const std::string& bodyName = "",
                                         const std::string& className = "mocap");
        
        
        // CONTACT EXCLUDES
        
        /// Add contact exclude elements of all nodes to their IgnoreCollision elements.
        void addContactExcludes(bool addParentChildExcludes = false);
        /// Add contact exclude elements of specified nodes to their IgnoreCollision elements.
        void addContactExcludes(RobotNodeSetPtr nodeSet, bool addParentChildExcludes = false);
        /**
         * @brief Add contact exclude elements of specified nodes to their 
         * `IgnoreCollision` elements.
         * @param addParentChildExcludes
         *      If true, also add contact excludes between bodies and their children.
         */
        void addContactExcludes(const std::vector<std::string>& nodeNames,
                                bool addParentChildExcludes = false);
        
        
        /// Add contact excludes of the given body to all node bodies (useful for mocap bodies).
        void addMocapContactExcludes(mjcf::Body mocap);
        
        
        // ACTUATORS
        
        /**
         * @brief Add an actuator for the given joint with the given type.
         * @return The actuator (one of mjcf::Actuator{Motor, Position, Velocity}).
         */
        mjcf::AnyElement addJointActuator(mjcf::Joint joint, ActuatorType type);
        
        /**
         * @brief Adds an actuator for the joint of the given node with the given type.
         * 
         * If the node's body has no joint, a joint is added before adding the actuator.
         * 
         * @return Same as addJointActuator().
         * @throw error::NodeIsNoJoint If `node` is not a joint node.
         */
        mjcf::AnyElement addNodeActuator(RobotNodePtr node, ActuatorType type);
        
        
        /// Add actuators for all joint nodes with.
        void addActuators(ActuatorType type);
        /// Add actuators for specified nodes. Non-joint nodes are ignored.
        void addActuators(RobotNodeSetPtr nodeSet, ActuatorType type);
        /// Add actuators for specified nodes. Non-joint nodes are ignored.
        void addActuators(const std::vector<std::string>& nodeNames, ActuatorType type);
        /// Add actuators for nodes with specified type. Non-joint nodes are ignored.
        void addActuators(const std::map<std::string, ActuatorType>& nodeTypeMap);
        
        
        // SENSORS
        
        
    private:

        /// The robot.
        RobotPtr robot;
        
        /// The document. Using a pointer allows to easily reset it.
        mjcf::DocumentPtr document { new mjcf::Document() };
        
        /// The path to the output XML file.
        std::filesystem::path outputFile;
        /// Absolute path to the directory where meshes shall be stored.
        std::filesystem::path outputMeshDir;
        
        
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
