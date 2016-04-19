#ifndef DRAKE_SYSTEMS_PLANTS_FORCETORQUEMEASUREMENT_H_
#define DRAKE_SYSTEMS_PLANTS_FORCETORQUEMEASUREMENT_H_

#include <Eigen/Core>

struct ForceTorqueMeasurement {
  int frame_idx;
  Eigen::Matrix<double, 6, 1> wrench;  // [torque; force], expressed in frame
                                       // corresponding to frame_idx;
};

#endif  // DRAKE_SYSTEMS_PLANTS_FORCETORQUEMEASUREMENT_H_
