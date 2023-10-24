//
// Created by usl on 11/29/20.
//

#ifndef CALIB_RELATIVEPOSE_H
#define CALIB_RELATIVEPOSE_H

#include "types/Pose.h"

#include <Eigen/Eigen>

#include <iostream>
#include <vector>

using namespace calib_type;

namespace calib_core
{
class relativePose
{
public:
  /// Time stamp of scan i
  double timestamp_i;
  /// Time stamp of scan j
  double timestamp_j;
  /// Odometry pose
  Eigen::Matrix4d odometry_ij;
};
}  // namespace calib_core
#endif  //CALIB_RELATIVEPOSE_H
