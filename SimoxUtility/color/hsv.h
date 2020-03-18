#pragma once

#include <Eigen/Core>

#include "Color.h"


namespace simox::color
{

    /// Convert an RGB color to HSV (hue in degrees).
    Eigen::Vector3f rgb_to_hsv(const Eigen::Vector3f& rgb);
    /// Convert an HSV color to RGB (hue in degrees).
    Eigen::Vector3f hsv_to_rgb(const Eigen::Vector3f& hsv);

    /// Convert an RGB color to HSV (hue in degrees).
    Eigen::Vector3i rgb_to_hsv(const Eigen::Vector3i& rgb);
    /// Convert an HSV color to RGB (hue in degrees).
    Eigen::Vector3i hsv_to_rgb(const Eigen::Vector3i& hsv);

}
