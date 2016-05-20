#pragma once

#include <Eigen/Core>

struct ForceTorqueMeasurement {
  int frame_idx;
  Eigen::Matrix<double, 6, 1> wrench;  // [torque; force], expressed in frame
                                       // corresponding to frame_idx;
};
