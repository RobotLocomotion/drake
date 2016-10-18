#pragma once

#include "drake/matlab/util/mexify.h"

DLL_EXPORT_SYM void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray* plhs[],
                                                     int nrhs,
                                                     const mxArray* prhs[]);
DLL_EXPORT_SYM void centerOfMassmex(int nlhs, mxArray* plhs[], int nrhs,
                                    const mxArray* prhs[]);
DLL_EXPORT_SYM void centerOfMassJacobianmex(int nlhs, mxArray* plhs[], int nrhs,
                                            const mxArray* prhs[]);
DLL_EXPORT_SYM void centroidalMomentumMatrixDotTimesvmex(int nlhs,
                                                         mxArray* plhs[],
                                                         int nrhs,
                                                         const mxArray* prhs[]);
DLL_EXPORT_SYM void centroidalMomentumMatrixmex(int nlhs, mxArray* plhs[],
                                                int nrhs,
                                                const mxArray* prhs[]);
DLL_EXPORT_SYM void doKinematicsmex(int nlhs, mxArray* plhs[], int nrhs,
                                    const mxArray* prhs[]);
DLL_EXPORT_SYM void findKinematicPathmex(int nlhs, mxArray* plhs[], int nrhs,
                                         const mxArray* prhs[]);
DLL_EXPORT_SYM void forwardJacDotTimesVmex(int nlhs, mxArray* plhs[], int nrhs,
                                           const mxArray* prhs[]);
DLL_EXPORT_SYM void forwardKinmex(int nlhs, mxArray* plhs[], int nrhs,
                                  const mxArray* prhs[]);
DLL_EXPORT_SYM void forwardKinJacobianmex(int nlhs, mxArray* plhs[], int nrhs,
                                          const mxArray* prhs[]);
DLL_EXPORT_SYM void forwardKinPositionGradientmex(int nlhs, mxArray* plhs[],
                                                  int nrhs,
                                                  const mxArray* prhs[]);
DLL_EXPORT_SYM void geometricJacobianDotTimesVmex(int nlhs, mxArray* plhs[],
                                                  int nrhs,
                                                  const mxArray* prhs[]);
DLL_EXPORT_SYM void geometricJacobianmex(int nlhs, mxArray* plhs[], int nrhs,
                                         const mxArray* prhs[]);
DLL_EXPORT_SYM void massMatrixmex(int nlhs, mxArray* plhs[], int nrhs,
                                  const mxArray* prhs[]);
DLL_EXPORT_SYM void dynamicsBiasTermmex(int nlhs, mxArray* plhs[], int nrhs,
                                        const mxArray* prhs[]);
DLL_EXPORT_SYM void velocityToPositionDotMappingmex(int nlhs, mxArray* plhs[],
                                                    int nrhs,
                                                    const mxArray* prhs[]);
DLL_EXPORT_SYM void positionDotToVelocityMappingmex(int nlhs, mxArray* plhs[],
                                                    int nrhs,
                                                    const mxArray* prhs[]);
