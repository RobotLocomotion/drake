#pragma once

#include <Eigen/Core>

struct ForceTorqueMeasurement {
  int frame_idx;
  Eigen::Matrix<double, 6, 1> wrench;  // [torque; force], expressed in frame
                                       // corresponding to frame_idx;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
