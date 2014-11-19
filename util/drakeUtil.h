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

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
    #define DLLEXPORT
#endif

// Helper routines
DLLEXPORT bool isa(const mxArray* mxa, const char* class_str);
DLLEXPORT bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename);


// Mex pointers shared through matlab
DLLEXPORT mxArray* createDrakeMexPointer(void* ptr, const char* name="", int num_additional_inputs=0, mxArray *delete_fcn_additional_inputs[] = NULL, const char* subclass_name=NULL);  // increments lock count
// Note: the same mex function which calls this method will be called with the syntax mexFunction(drake_mex_ptr) as the destructor

DLLEXPORT void* getDrakeMexPointer(const mxArray* mx);

template <typename Derived> inline void destroyDrakeMexPointer(const mxArray* mx)
{
  Derived typed_ptr = (Derived) getDrakeMexPointer(mx);

  //mexPrintf("deleting drake mex pointer\n");
  delete typed_ptr;
  mexUnlock();  // decrement lock count

//  mexPrintf(mexIsLocked() ? "mex file is locked\n" : "mex file is unlocked\n");
}


DLLEXPORT double angleAverage(double theta1, double theta2);

DLLEXPORT std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(Eigen::Vector3d torque, Eigen::Vector3d force, Eigen::Vector3d normal, Eigen::Vector3d point_on_contact_plane);

#endif /* DRAKE_UTIL_H_ */
