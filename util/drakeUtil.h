/*
 * drakeUtil.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#ifndef DRAKE_UTIL_H_
#define DRAKE_UTIL_H_

// Helper routines
bool isa(const mxArray* mxa, const char* class_str);
bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename);


// Mex pointers shared through matlab
mxArray* createDrakeMexPointer(void* ptr, const char* deleteMethod="", const char* name="");
void* getDrakeMexPointer(const mxArray* mx);


#endif /* DRAKE_UTIL_H_ */
