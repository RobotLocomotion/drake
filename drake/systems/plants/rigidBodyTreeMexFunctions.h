#pragma once

#include "drake/matlab/util/mexify.h"

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
#if defined(rbmMexFunctions_EXPORTS)
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __declspec(dllimport)
#endif
#else
#define DLLEXPORT [[gnu::visibility("default")]]
#endif

DLLEXPORT void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray *plhs[],
                                                int nrhs,
                                                const mxArray *prhs[]);
DLLEXPORT void centerOfMassmex(int nlhs, mxArray *plhs[], int nrhs,
                               const mxArray *prhs[]);
DLLEXPORT void centerOfMassJacobianmex(int nlhs, mxArray *plhs[], int nrhs,
                                       const mxArray *prhs[]);
DLLEXPORT void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray *plhs[],
                                                    int nrhs,
                                                    const mxArray *prhs[]);
DLLEXPORT void centroidalMomentumMatrixmex(int nlhs, mxArray *plhs[], int nrhs,
                                           const mxArray *prhs[]);
DLLEXPORT void doKinematicsmex(int nlhs, mxArray *plhs[], int nrhs,
                               const mxArray *prhs[]);
DLLEXPORT void findKinematicPathmex(int nlhs, mxArray *plhs[], int nrhs,
                                    const mxArray *prhs[]);
DLLEXPORT void forwardJacDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs,
                                      const mxArray *prhs[]);
DLLEXPORT void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs,
                             const mxArray *prhs[]);
DLLEXPORT void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs,
                                     const mxArray *prhs[]);
DLLEXPORT void forwardKinPositionGradientmex(int nlhs, mxArray *plhs[],
                                             int nrhs, const mxArray *prhs[]);
DLLEXPORT void geometricJacobianDotTimesVmex(int nlhs, mxArray *plhs[],
                                             int nrhs, const mxArray *prhs[]);
DLLEXPORT void geometricJacobianmex(int nlhs, mxArray *plhs[], int nrhs,
                                    const mxArray *prhs[]);
DLLEXPORT void massMatrixmex(int nlhs, mxArray *plhs[], int nrhs,
                             const mxArray *prhs[]);
DLLEXPORT void dynamicsBiasTermmex(int nlhs, mxArray *plhs[], int nrhs,
                                   const mxArray *prhs[]);
DLLEXPORT void velocityToPositionDotMappingmex(int nlhs, mxArray *plhs[],
                                               int nrhs, const mxArray *prhs[]);
DLLEXPORT void positionDotToVelocityMappingmex(int nlhs, mxArray *plhs[],
                                               int nrhs, const mxArray *prhs[]);
