#pragma once

#include <memory>
#include <filesystem>
#include "SimoxUtility/error/SimoxError.h"

namespace rapidxml
{
    template<class Ch>
    class xml_node;
}

namespace simox
{

namespace error
{
    class XMLFormatError : public SimoxError
    {
    public:
        XMLFormatError(const std::string &message = std::string()) : SimoxError(message) {}
    };
}

namespace xml
{

namespace attribute
{
    template<typename T, typename std::enable_if<std::is_fundamental<T>::value>::type* = nullptr>
    struct XMLAttribute
    {
        XMLAttribute(const std::string &attributeName) : attributeName(attributeName)
        {
        }

        std::string attributeName;
    };
}

class RapidXMLWrapperNode;
typedef std::shared_ptr<RapidXMLWrapperNode> RapidXMLWrapperNodePtr;

class RapidXMLWrapperRootNode;
typedef std::shared_ptr<RapidXMLWrapperRootNode> RapidXMLWrapperRootNodePtr;

}

}
