#ifndef __RapidXMLWrapper_H_
#define __RapidXMLWrapper_H_

#include "rapidxml.hpp"
#include "rapidxml_print.hpp"

#include "RapidXMLForwardDecl.h"

#include <utility>
#include <iostream>
#include <fstream>
#include <memory>

#include "SimoxUtility/algorithm/string.h"

namespace simox {

namespace xml {

//! @brief Represents an xml node in the RapidXMLWrapper
class RapidXMLWrapperNode : public std::enable_shared_from_this<RapidXMLWrapperNode>
{

    friend class RapidXMLWrapperRootNode;

protected:
    std::shared_ptr<rapidxml::xml_document<> > doc;
    rapidxml::xml_node<>* node;
    RapidXMLWrapperNodePtr parent;

    const char* cloneString(const std::string& str) {
        return doc->allocate_string(str.c_str());
    }

    RapidXMLWrapperNode(std::shared_ptr<rapidxml::xml_document<> > doc, rapidxml::xml_node<>* node, RapidXMLWrapperNodePtr parent)
        : doc(doc), node(node), parent(parent)
    { }

    RapidXMLWrapperNode(std::shared_ptr<rapidxml::xml_document<> > doc, rapidxml::node_type node_type, RapidXMLWrapperNodePtr parent) : doc(doc), parent(parent) {
        node = doc->allocate_node(node_type);
    }

    RapidXMLWrapperNode(std::shared_ptr<rapidxml::xml_document<> > doc, rapidxml::node_type node_type, RapidXMLWrapperNodePtr parent, const std::string& name) : doc(doc), parent(parent) {
        node = doc->allocate_node(node_type, cloneString(name));
    }

    void check() const {
        if (!node) {
            throw error::XMLFormatError("NullPointerException");
        }
    }

    [[noreturn]] inline void throwError(const char* message) {
        throw error::XMLFormatError("Error at " + getPath() + "! " + message);
    }

    [[noreturn]] inline void rethrowXMLFormatError() {
        try
        {
            throw;
        }
        catch (error::SimoxError &e) {
            throwError(e.what());
        }
        catch (...) {
            throwError("Unknown Error.");
        }
    }

public:
    static RapidXMLWrapperNodePtr NullNode() {
        RapidXMLWrapperNodePtr wrapper(new RapidXMLWrapperNode(std::shared_ptr<rapidxml::xml_document<> >(), NULL, nullptr));
        return wrapper;
    }

    /**
        * @brief get_node_ptr only for legacy code.
        * @return internal pointer
        */
    rapidxml::xml_node<>* get_node_ptr() const {
        return node;
    }

    /*! Returns the first child node (optional: with the given name).
        @throws error::XMLFormatError if the node does not exist. */
    RapidXMLWrapperNodePtr first_node(const char* name = 0) {
        check();

        rapidxml::xml_node<>* node = this->node->first_node(name, 0, false);

        if (!node) throw error::XMLFormatError(std::string("Node '") + name + "' does not exist in node " + getPath());

        RapidXMLWrapperNodePtr wrapper(new RapidXMLWrapperNode(doc, node, shared_from_this()));
        return wrapper;
    }

    /*! Returns the first child node with the given name and attribute. */
    RapidXMLWrapperNodePtr first_node_with_attribute(const char* name, const std::string &attrName, const std::string &attrValue) {
        check();
        std::vector<RapidXMLWrapperNodePtr> n = nodes(name);
        for (std::vector<RapidXMLWrapperNodePtr>::iterator it; n.begin(), it != n.end(); ++it) {
            if ((*it)->has_attribute(attrName.c_str()) && (*it)->attribute_value(attrName.c_str()) == attrValue) {
                return (*it);
            }
        }
        return NullNode();
    }

    /*! Returns all child nodes (optional: just with the given name). */
    std::vector<RapidXMLWrapperNodePtr> nodes(const char* name = 0) {
        std::vector<RapidXMLWrapperNodePtr> vec;
        nodes(name, vec);
        return vec;
    }

    void nodes(const char* name, std::vector<RapidXMLWrapperNodePtr>& vec) {
        for (RapidXMLWrapperNodePtr n = first_node(name); n->is_valid(); n = n->next_sibling(name)) {
            vec.push_back(n);
        }
    }

    /*! Returns the attribute value.
        @param attrName The name of the attribute.
        @throws error::XMLFormatError if the attribute does not exist. */
    std::string attribute_value(const char* attrName) const {
        check();
        rapidxml::xml_attribute<>* attrib = node->first_attribute(attrName, 0, false);

        if (!attrib)
        {
            throw error::XMLFormatError(std::string("Attribute '") + attrName + "' does not exist in node " + getPath());
        }

        return std::string(attrib->value());
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    T attribute_value_(const char* attrName) {
        try { return alg::to_<T>(attribute_value(attrName)); } catch (...) { rethrowXMLFormatError(); };
    }

    template<typename T>
    T attribute_value_(const attribute::XMLAttribute<T> &attribute) {
        try { return alg::to_<T>(attribute_value(attribute.attributeName.c_str())); } catch (...) { rethrowXMLFormatError(); };
    }

    std::vector<std::string> attribute_value_vec_str(const char* attrName, const std::string &delimiter = ";") {
        try { return alg::split(attribute_value(attrName), delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    std::vector<std::string> attribute_value_vec_str(const char* attrName, unsigned int size, const std::string &delimiter = ";") {
        try { return alg::split_check_size(attribute_value(attrName), size, delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    /*! Checks if a node has a specific attribute.
        @param attrName The name of the attribute.*/
    bool has_attribute(const char* attrName) const
    {
        check();
        return node->first_attribute(attrName, 0, false) != 0;
    }

    /*! Checks if a node has a specific child node.
        @param attrName The name of the child node.*/
    bool has_node(const char* nodeName) const
    {
        check();
        return node->first_node(nodeName, 0, false) != 0;
    }

    /*! Returns the value of an attribute if the attribute exists, otherwise a default value.
        @param attrName The name of the attribute.
        @param defaultValue The default value.*/
    std::string attribute_value_or_default(const char* attrName, const std::string& defaultValue) const
    {
        check();
        rapidxml::xml_attribute<>* attrib = node->first_attribute(attrName, 0, false);

        if (!attrib)
        {
            return defaultValue;
        }

        return std::string(attrib->value());
    }

    std::vector<std::pair<std::string, std::string> > get_all_attributes()
    {
        check();
        std::vector<std::pair<std::string, std::string> > attributes;

        rapidxml::xml_attribute<>* attrib = node->first_attribute(0, 0, false);

        while (attrib)
        {
            std::string name = std::string(attrib->name());
            std::string value = std::string(attrib->value());
            std::pair<std::string, std::string> attrPair(name, value);
            attributes.push_back(attrPair);
            attrib = attrib->next_attribute();
        }

        return attributes;
    }

    /*! Returns the content of an xml node.*/
    std::string value() const
    {
        check();
        return std::string(node->value());
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    T value_() {
        try { return alg::to_<T>(value()); } catch (...) { rethrowXMLFormatError(); };
    }

    std::vector<std::string> value_vec_str(const std::string &delimiter = ";") {
        try { return alg::split(value(), delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    std::vector<std::string> value_vec_str(unsigned int size, const std::string &delimiter = ";") {
        try { return alg::split_check_size(value(), size, delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    std::vector<T> value_vec_(const std::string &delimiter = "\t ") {
        try { return alg::to_vec<T>(value(), delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    std::vector<T> value_vec_(unsigned int size, const std::string &delimiter = "\t ") {
        try { return alg::to_vec<T>(value(), size, delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    Eigen::VectorXf value_eigen_vec(const std::string &delimiter = "\t ") {
        try { return alg::to_eigen_vec(value(), delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    Eigen::VectorXf value_eigen_vec(unsigned int size, const std::string &delimiter = "\t ") {
        try { return alg::to_eigen_vec_check_rows<float>(value(), size, delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    Eigen::Vector3f value_eigen_vec3(const std::string &delimiter = "\t ") {
        try { return alg::to_eigen_vec<float, 3>(value(), delimiter); } catch (...) { rethrowXMLFormatError(); };
    }

    Eigen::Matrix4f value_matrix4f() {
        Eigen::Matrix4f matrix;
        for (int i = 1; i <= 4; i++) {
            std::string row = "row" + simox::alg::to_string(i);
            simox::xml::RapidXMLWrapperNodePtr rowNode = first_node(row.c_str());
            for (int j = 1; j <= 4; j++) {
                std::string column = "c" + simox::alg::to_string(j);
                matrix(i - 1, j - 1) = rowNode->attribute_value_<float>(column.c_str());
            }
        }
        return matrix;
    }

    /*! Returns the name of an xml node.*/
    std::string name() const
    {
        check();
        return std::string(node->name());
    }

    rapidxml::node_type type()
    {
        check();
        return node->type();
    }

    std::string first_node_value(const char* nodeName = 0) const
    {
        check();
        rapidxml::xml_node<>* childNode = node->first_node(nodeName, 0, false);

        if (!childNode)
        {
            throw error::XMLFormatError(std::string("Node '") + nodeName + "' does not exist in node " + getPath());
        }

        return std::string(childNode->value());
    }

    std::string first_node_value_or_default(const char* name, const std::string& defaultValue) const
    {
        check();
        rapidxml::xml_node<>* childNode = node->first_node(name, 0, false);

        if (!childNode)
        {
            return defaultValue;
        }

        return std::string(childNode->value());
    }
    RapidXMLWrapperNodePtr next_sibling(const char* name = 0)
    {
        check();
        RapidXMLWrapperNodePtr wrapper(new RapidXMLWrapperNode(doc, node->next_sibling(name, 0, false), shared_from_this()));
        return wrapper;
    }

    bool is_valid() const
    {
        return node != NULL;
    }

    std::string getPath() const
    {
        check();
        std::string result = name();
        rapidxml::xml_node<>* p = node->parent();

        while (p)
        {
            result = std::string(p->name()) + "/" + result;
            p = p->parent();
        }

        return result;
    }

    /*! Appends a new attribute to the current node.
    @return The current Node.*/
    RapidXMLWrapperNodePtr append_attribute(const std::string& name, const std::string& value) {
        node->append_attribute(doc->allocate_attribute(cloneString(name), cloneString(value)));
        return shared_from_this();
    }

    template<typename T>
    RapidXMLWrapperNodePtr append_attribute(const attribute::XMLAttribute<T>& attribute, T value) {
        return append_attribute(attribute.attributeName, alg::to_string(value)); // will not throw error
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    RapidXMLWrapperNodePtr append_attribute(const std::string& name, T value) {
        try { return append_attribute(name, alg::to_string(value)); } catch (...) { rethrowXMLFormatError(); };
    }

    RapidXMLWrapperNodePtr append_attribute(const std::string& name, const std::vector<std::string> value, const std::string &delimiter = ";") {
        try { return append_attribute(name, alg::to_string(value, delimiter)); } catch (...) { rethrowXMLFormatError(); };
    }

    /*! Appends a new node on the current node.
    @return The new Node.*/
    RapidXMLWrapperNodePtr append_node(const std::string& name) {
        RapidXMLWrapperNodePtr node(new RapidXMLWrapperNode(doc, rapidxml::node_element, shared_from_this(), name));
        this->node->append_node(node->node);
        return node;
    }

    /*! Appends a new node on the current node and deletes the first node with the same name
    @return The new Node.*/
    RapidXMLWrapperNodePtr append_node_non_duplicate(const std::string& name) {
        remove_first_node_if_present(name);
        return append_node(name);
    }

    /*! Adds std::string content to the current node.
    @return The current Node.*/
    RapidXMLWrapperNodePtr append_data_node(const std::string& value) {
        this->node->append_node(doc->allocate_node(rapidxml::node_data, 0, cloneString(value)));
        return shared_from_this();
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    RapidXMLWrapperNodePtr append_data_node(T value) {
        try { return append_data_node(alg::to_string(value)); } catch (...) { rethrowXMLFormatError(); };
    }

    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    RapidXMLWrapperNodePtr append_data_node(const std::vector<T> &value, const std::string &delimiter = " ") {
        try { return append_data_node(alg::to_string(value, delimiter)); } catch (...) { rethrowXMLFormatError(); };
    }

    RapidXMLWrapperNodePtr append_data_node(const Eigen::VectorXf value, const std::string &delimiter = " ") {
        try { return append_data_node(alg::to_string(value, delimiter)); } catch (...) { rethrowXMLFormatError(); };
    }

    RapidXMLWrapperNodePtr append_data_node(const std::vector<std::string> value, const std::string &delimiter = ";") {
        try { return append_data_node(alg::to_string(value, delimiter)); } catch (...) { rethrowXMLFormatError(); };
    }

    RapidXMLWrapperNodePtr append_data_node(const Eigen::Matrix4f &matrix, const std::string &name) {
        simox::xml::RapidXMLWrapperNodePtr matrixNode = append_node(name);
        for (int i = 1; i <= 4; i++) {
            std::string row = "row" + alg::to_string(i);
            simox::xml::RapidXMLWrapperNodePtr rowNode = matrixNode->append_node(row);
            for (int j = 1; j <= 4; j++) {
                std::string column = "c" + alg::to_string(j);
                rowNode->append_attribute(column, matrix(i - 1, j - 1));
            }
        }
        return matrixNode;
    }

    RapidXMLWrapperNodePtr append_string_node(const std::string& name, const std::string& value) {
        this->node->append_node(doc->allocate_node(rapidxml::node_element, cloneString(name), cloneString(value)));
        return shared_from_this();
    }

    void remove_first_node(const std::string &name) {
        RapidXMLWrapperNodePtr node = first_node(name.c_str());
        this->node->remove_node(node->node);
    }

    void remove_first_node_if_present(const std::string &name) {
        if (has_node(name.c_str())) remove_first_node(name);
    }

    void remove_attribute(const std::string &name) {
        this->node->remove_attribute(this->node->first_attribute(name.c_str()));
    }

    void remove_attribute_if_present(const std::string &name) {
        if (has_attribute(name.c_str())) remove_attribute(name);
    }

    /*! Creates an xml std::string representation of this xml nodes' structure
    @param indent Usage of tabs in the std::string representation for better readability. */
    std::string print(bool indent = true) {
        std::string s;
        rapidxml::print(std::back_inserter(s), *node, indent ? 0 : rapidxml::print_no_indenting);
        return s;
    }

    /*! Saves this xml nodes' structure as xml document.
    @param path Path where the xml document should be saved.
    @param indent Usage of tabs in the std::string representation for better readability.*/
    virtual void saveToFile(const std::string& path, bool indent = true) {
        std::ofstream file;
        file.open(path.c_str());
        file << print(indent);
        file.close();
    }

    const rapidxml::xml_node<>* getNode() {
        return node;
    }
};

//! @brief Helper class for reading information from an xml format via Rapid XML
class RapidXMLWrapperRootNode : public RapidXMLWrapperNode
{
private:
    char* cstr; // The string must persist for the lifetime of the document.
    std::filesystem::path path;

    RapidXMLWrapperRootNode(const std::string& xml, const  std::filesystem::path &path =  std::filesystem::path())
        : RapidXMLWrapperNode(getDocument(xml), NULL, nullptr), path(path)
    {
        node = doc->first_node();
    }

    RapidXMLWrapperRootNode(std::shared_ptr<rapidxml::xml_document<> > doc, const std::string &name)
        : RapidXMLWrapperNode(doc, rapidxml::node_element, nullptr, name), path(std::filesystem::path())
    {
        cstr = new char[0];
    }

    std::shared_ptr<rapidxml::xml_document<> > getDocument(const std::string& xml) {
        if (xml.empty()) throw error::XMLFormatError(std::string("Empty xml!"));
        cstr = new char[xml.size() + 1];        // Create char buffer to store std::string copy
        strcpy(cstr, xml.c_str());              // Copy std::string into char buffer
        try {
            std::shared_ptr<rapidxml::xml_document<> > doc(new rapidxml::xml_document<>());
            doc->parse<0>(cstr);                    // Pass the non-const char* to parse()
            return doc;
        }
        catch (rapidxml::parse_error e) {
            std::string msg = e.what();
            throw error::XMLFormatError(std::string("No valid xml format! " + msg));
        }
    }

public:
    ~RapidXMLWrapperRootNode() {
        delete[] cstr;                          // free buffer memory when all is finished
    }

public:
    static std::string ReadFileContents(const std::string& path) {
        try {
            std::ifstream in(path.c_str());

            if (!in || !in.is_open()) {
                throw error::XMLFormatError("Could not open XML file " + path);
            }

            std::stringstream buffer;
            buffer << in.rdbuf();
            std::string xmlString(buffer.str());
            in.close();

            return xmlString;
        }
        catch (const std::ifstream::failure& e) {
            throw error::XMLFormatError("Could not open XML file because " + std::string(e.what()));
        }
    }

    /*!
        Creates a RapidXMLWrapper.
        @param xml The xml as string.
        @return The RapidXMLWrapper for the xml string.
    */
    static RapidXMLWrapperRootNodePtr FromXmlString(const std::string& xml, const std::string &path = std::string()) {
        RapidXMLWrapperRootNodePtr wrapper(new RapidXMLWrapperRootNode(xml, path));
        return wrapper;
    }

    /*!
        Creates a RapidXMLWrapper.
        @param xml Path to an xml document.
        @return The RapidXMLWrapper for the xml document.
    */
    static RapidXMLWrapperRootNodePtr FromFile(const std::string& path) {
        return FromXmlString(ReadFileContents(path), path);
    }

    /*! Creates a root node with the given name.
    @return The root Node. */
    static RapidXMLWrapperRootNodePtr createRootNode(const std::string& name) {
        std::shared_ptr<rapidxml::xml_document<> > doc(new rapidxml::xml_document<>());
        RapidXMLWrapperNodePtr declaration(new RapidXMLWrapperNode(doc, rapidxml::node_declaration, nullptr));
        declaration->append_attribute("version", "1.0");
        declaration->append_attribute("encoding", "utf-8");
        doc->append_node(declaration->node);
        RapidXMLWrapperRootNodePtr rootNode = RapidXMLWrapperRootNodePtr(new RapidXMLWrapperRootNode(doc, name));
        doc->append_node(rootNode->node);
        return rootNode;
    }

    /*! Saves the created xml structure as xml document.
    @param path Path where the xml document should be saved.
    @param indent Usage of tabs in the std::string representation for better readability.*/
    void saveToFile(const std::string& path, bool indent = true) override {
        setPath(path);
        RapidXMLWrapperNode::saveToFile(path, indent);
    }

    /*! Saves the created xml structure as xml document at the same path it was loaded
    @param indent Usage of tabs in the std::string representation for better readability.
    @throws error::XMLFormatError if path value is empty */
    void saveToFile(bool indent = true) {
        if (!path.empty()) saveToFile(path, indent);
        else throw error::XMLFormatError("Empty path variable!");
    }

    void setPath(const std::string &path) {
        this->path = path;
    }

    std::string getPath() {
        return path;
    }
};

}
}

#endif
