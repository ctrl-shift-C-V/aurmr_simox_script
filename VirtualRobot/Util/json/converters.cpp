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
}



