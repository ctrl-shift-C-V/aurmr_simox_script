#pragma once

#include <Eigen/Geometry>

namespace simox::math {

Eigen::Affine3f interpolatePose(const Eigen::Affine3f &posePre, const Eigen::Affine3f &poseNext, float t);

} // namespace simox::math