/*
 * drake.h
 *
 *  Created on: Jun 19, 2013
 *      Author: russt
 */

#ifndef DRAKE_H_
#define DRAKE_H_

// Helper routines
bool isa(const mxArray* mxa, const char* class_str);
bool mexCallMATLABsafe(int nlhs, mxArray* plhs[], int nrhs, mxArray* prhs[], const char* filename);


// Mex pointers shared through matlab
mxArray* constructDrakeMexPointer(void* ptr, void (*delete_fcn)(void*)=NULL);
void* getDrakeMexPointer(const mxArray* mx);
void destroyDrakeMexPointer(const mxArray* mx);



#endif /* DRAKE_H_ */
