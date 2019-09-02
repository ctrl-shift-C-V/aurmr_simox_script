#pragma once

#include <Eigen/Core>

#include "json.hpp"


namespace VirtualRobot::json
{
    Eigen::Matrix4f posquat2eigen4f(const std::string& str);
    Eigen::Matrix4f posquat2eigen4f(const char* str);
    Eigen::Matrix4f posquat2eigen4f(const nlohmann::json& j);
}
