#pragma once

#include <Eigen/Core>

#include "json.hpp"


namespace VirtualRobot::json
{
    Eigen::Matrix4f posquat2eigen4f(const std::string& str);
    Eigen::Matrix4f posquat2eigen4f(const char* str);
    Eigen::Matrix4f posquat2eigen4f(const nlohmann::json& j);

    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const std::string& str);
    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const char* str);
    std::vector<Eigen::Matrix4f> posquatArray2eigen4fVector(const nlohmann::json& j);

    std::string eigen4f2posquatJson(const Eigen::Matrix4f& str);
    std::string eigen4fVector2posquatArrayJson(const std::vector<Eigen::Matrix4f>& str);
}
