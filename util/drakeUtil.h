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
mxArray* createDrakeMexPointer(void* ptr, const char* name="", int num_additional_inputs=0, mxArray *delete_fcn_additional_inputs[] = NULL);  // increments lock count
// Note: the same mex function which calls this method will be called with the syntax mexFunction(drake_mex_ptr) as the destructor

void* getDrakeMexPointer(const mxArray* mx);

template <typename Derived> inline void destroyDrakeMexPointer(const mxArray* mx)
{
  Derived typed_ptr = (Derived) getDrakeMexPointer(mx);

  //mexPrintf("deleting drake mex pointer\n");
  delete typed_ptr;
  mexUnlock();  // decrement lock count

//  mexPrintf(mexIsLocked() ? "mex file is locked\n" : "mex file is unlocked\n");
}


double angleAverage(double theta1, double theta2);

std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(Eigen::Vector3d torque, Eigen::Vector3d force, Eigen::Vector3d normal, Eigen::Vector3d point_on_contact_plane);

#endif /* DRAKE_UTIL_H_ */
