/*
 * ForceTorqueMeasurement.h
 *
 *  Created on: May 11, 2015
 *      Author: twan
 */

#ifndef SYSTEMS_PLANTS_FORCETORQUEMEASUREMENT_H_
#define SYSTEMS_PLANTS_FORCETORQUEMEASUREMENT_H_

#include <Eigen/Core>

struct ForceTorqueMeasurement {
  int frame_idx;
  Eigen::Matrix<double, 6, 1> wrench; // [torque; force], expressed in frame corresponding to frame_idx;
};

#endif /* SYSTEMS_PLANTS_FORCETORQUEMEASUREMENT_H_ */
