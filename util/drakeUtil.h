/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#include "mex.h"
#include <utility>
#include <Eigen/Core>

#ifndef DRAKE_UTIL_H_
#define DRAKE_UTIL_H_

// Helper routines
bool isa(const mxArray* mxa, const char* class_str);
bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename);


// Mex pointers shared through matlab
mxArray* createDrakeMexPointer(void* ptr, const char* deleteMethod="", const char* name="");
void* getDrakeMexPointer(const mxArray* mx);

double angleAverage(double theta1, double theta2);

std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(Eigen::Vector3d torque, Eigen::Vector3d force, Eigen::Vector3d normal, Eigen::Vector3d point_on_contact_plane);

#endif /* DRAKE_UTIL_H_ */
