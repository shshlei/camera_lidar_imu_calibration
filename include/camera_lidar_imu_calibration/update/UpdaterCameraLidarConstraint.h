//
// Created by usl on 4/5/22.
//

#ifndef CAMERA_IMU_LIDAR_CALIBRATION_UPDATERCAMERALIDARCONSTRAINT_H
#define CAMERA_IMU_LIDAR_CALIBRATION_UPDATERCAMERALIDARCONSTRAINT_H

#include "camera_lidar_imu_calibration/update/UpdaterOptions.h"
#include "camera_lidar_imu_calibration/state/State.h"
#include "camera_lidar_imu_calibration/state/StateHelper.h"
#include "camera_lidar_imu_calibration/utils/color.h"
#include "camera_lidar_imu_calibration/utils/quat_ops.h"

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

namespace calib_estimator
{
class UpdaterCameraLidarConstraint
{
public:
  UpdaterCameraLidarConstraint(UpdaterOptions & options)
  : _options(options)
  {
    for (int i = 1; i < 500; i++) {
      boost::math::chi_squared chi_squared_dist(i);
      chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
    }
  }

  /// Given plane measurement from both camera and lidar, estimate the state
  double updatePlaneToPlaneConstraint(State * current_state,
    Eigen::Vector4d nd_c, double timestamp_camera,
    Eigen::Vector4d nd_l, double timestamp_lidar,
    bool & did_update);

  /// Given plane measurement from both camera and lidar, estimate the state
  void updatePlaneToPlaneConstraint(State * current_state, Eigen::Matrix4d G_T_I0,
    std::vector<cv::Point3f> object_points_c0,
    Eigen::Vector4d nd_L,
    double timestamp, bool & did_update);

  /// Given plane measurement from both camera and lidar, estimate the state
  void updatePlaneToPlaneConstraint(State * current_state, Eigen::Matrix4d G_T_I0,
    std::vector<cv::Point3f> object_points_c0,
    std::map<double, Eigen::Vector4d> nd_l_vector,
    bool & did_update);

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;
};
}  // namespace calib_estimator
#endif  //CAMERA_IMU_LIDAR_CALIBRATION_UPDATERCAMERALIDARCONSTRAINT_H
