#include "invert.h"

#include <SimoxUtility/math/pose.h>


void simox::math::invert_pose(Eigen::Matrix4f& pose)
{
    orientation(pose).transposeInPlace();
    position(pose) = - orientation(pose) * position(pose);
}
