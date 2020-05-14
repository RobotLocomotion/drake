#pragma once

#include <Eigen/Core>

#include "drake/attic_warning.h"

struct ForceTorqueMeasurement {
  int frame_idx;
  Eigen::Matrix<double, 6, 1> wrench;  // [torque; force], expressed in frame
                                       // corresponding to frame_idx;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
