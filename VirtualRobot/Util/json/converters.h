#pragma once

#include <SimoxUtility/json/converters.h>


// Legacy overloads redirecting to Simox/math/json
// Do not add functions here, but add them to <SimoxUtility/json/converters.h>


namespace VirtualRobot::json
{
    Eigen::Matrix4f posquat2eigen4f(const std::string& str)
    {
        return ::simox::json::posquat2eigen4f(str);
    }
    Eigen::Matrix4f posquat2eigen4f(const char* str)
    {
        return ::simox::json::posquat2eigen4f(str);
    }
    Eigen::Matrix4f posquat2eigen4f(const nlohmann::json& j)
    {
        return ::simox::json::posquat2eigen4f(j);
    }

    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const std::string& str)
    {
        return ::simox::json::posquatArray2eigen4fVector(str);
    }
    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const char* str)
    {
        return ::simox::json::posquatArray2eigen4fVector(str);
    }

    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const nlohmann::json& j)
    {
        return ::simox::json::posquatArray2eigen4fVector(j);
    }

    std::string eigen4f2posquatJson(const Eigen::Matrix4f& mx)
    {
        return ::simox::json::eigen4f2posquatJson(mx);
    }
    std::string eigen4fVector2posquatArrayJson(const std::vector<Eigen::Matrix4f>& vec)
    {
        return ::simox::json::eigen4fVector2posquatArrayJson(vec);
    }

    std::map<std::string, float> json2NameValueMap(const std::string& str)
    {
        return ::simox::json::json2NameValueMap(str);
    }
    std::map<std::string, float> json2NameValueMap(const char* str)
    {
        return ::simox::json::json2NameValueMap(str);
    }
    std::map<std::string, float> json2NameValueMap(const nlohmann::json& j)
    {
        return ::simox::json::json2NameValueMap(j);
    }
}
