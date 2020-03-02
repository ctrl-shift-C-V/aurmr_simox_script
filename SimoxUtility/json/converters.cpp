#include <boost/lexical_cast.hpp>

#include <VirtualRobot/MathTools.h>

#include "converters.h"

namespace VirtualRobot::json
{
    Eigen::Matrix4f posquat2eigen4f(const std::string& str)
    {
        return posquat2eigen4f(::nlohmann::json::parse(str));
    }
    Eigen::Matrix4f posquat2eigen4f(const char* str)
    {
        return posquat2eigen4f(::nlohmann::json::parse(str));
    }
    Eigen::Matrix4f posquat2eigen4f(const nlohmann::json& j)
    {
        return VirtualRobot::MathTools::posquat2eigen4f(
                   j.at("x").get<float>(),
                   j.at("y").get<float>(),
                   j.at("z").get<float>(),
                   j.at("qx").get<float>(),
                   j.at("qy").get<float>(),
                   j.at("qz").get<float>(),
                   j.at("qw").get<float>()
               );
    }

    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const std::string& str)
    {
        return posquatArray2eigen4fVector(::nlohmann::json::parse(str));
    }
    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const char* str)
    {
        return posquatArray2eigen4fVector(::nlohmann::json::parse(str));
    }

    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const nlohmann::json& j)
    {
        if (!j.is_array())
        {
            throw std::invalid_argument{"posquatArray2eigen4fVector: json has to be an array"};
        }
        std::vector<Eigen::Matrix4f> result;
        result.reserve(j.size());
        for (const auto& element : j)
        {
            result.emplace_back(posquat2eigen4f(element));
        }
        return result;
    }

    std::string eigen4f2posquatJson(const Eigen::Matrix4f& mx)
    {
        nlohmann::json j = nlohmann::json::object();
        const auto quat = VirtualRobot::MathTools::eigen4f2quat(mx);
        j["qx"] = quat.x;
        j["qy"] = quat.y;
        j["qz"] = quat.z;
        j["qw"] = quat.w;

        j["x"] = mx(0, 3);
        j["y"] = mx(1, 3);
        j["z"] = mx(2, 3);
        return j.dump(4);
    }
    std::string eigen4fVector2posquatArrayJson(const std::vector<Eigen::Matrix4f>& vec)
    {
        nlohmann::json jar = nlohmann::json::array();
        for (const auto& mx : vec)
        {
            nlohmann::json j = nlohmann::json::object();
            const auto quat = VirtualRobot::MathTools::eigen4f2quat(mx);
            j["qx"] = quat.x;
            j["qy"] = quat.y;
            j["qz"] = quat.z;
            j["qw"] = quat.w;

            j["x"] = mx(0, 3);
            j["y"] = mx(1, 3);
            j["z"] = mx(2, 3);
            jar.push_back(j);
        }
        return jar.dump(4);
    }

    std::map<std::string, float> json2NameValueMap(const std::string& str)
    {
        return json2NameValueMap(::nlohmann::json::parse(str));
    }
    std::map<std::string, float> json2NameValueMap(const char* str)
    {
        return json2NameValueMap(::nlohmann::json::parse(str));
    }
    std::map<std::string, float> json2NameValueMap(const nlohmann::json& j)
    {
        if (!j.is_object())
        {
            throw std::invalid_argument{"json2NameValueMap: json has to be an object"};
        }
        std::map<std::string, float> result;
        for (const auto& el : j.items())
        {
            result[el.key()] = boost::lexical_cast<float>(el.value());
        }
        return result;
    }
}
