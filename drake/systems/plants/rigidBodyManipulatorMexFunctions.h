#include "mexify.h"

#ifndef DRAKE_DRAKEMEXFUNCTIONS_H
#define DRAKE_DRAKEMEXFUNCTIONS_H

void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
//void centerOfMassmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void centerOfMassJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void centroidalMomentumMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void doKinematicsmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void findKinematicPathmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void forwardJacDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
//void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void forwardKinPositionGradientmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void geometricJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void geometricJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
void massMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
//void dynamicsBiasTermmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);

#endif //DRAKE_DRAKEMEXFUNCTIONS_H
