#include <mex.h>

#include <cmath>
#include <iostream>
#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

/*
 * Mex interface for computing friction cone surface tangent basis vectors given
 *contact normals.
 *
 * MATLAB signature:
 * d =
 *     surfaceTangents(mex_model_ptr, normals)
 */

inline mxArray *getTangentsArray(RigidBodyTree *const model,
                                 Map<Matrix3Xd> const &normals) {
  const size_t numContactPairs = normals.cols();
  const mwSize cellDims[] = {1, BASIS_VECTOR_HALF_COUNT};
  mxArray *tangentCells = mxCreateCellArray(2, cellDims);

  vector<Map<Matrix3Xd> > tangents;
  for (int k = 0; k < BASIS_VECTOR_HALF_COUNT; k++) {
    mxArray *cell = mxCreateDoubleMatrix(3, numContactPairs, mxREAL);
    tangents.push_back(Map<Matrix3Xd>(mxGetPrSafe(cell), 3, numContactPairs));
    mxSetCell(tangentCells, k, cell);
  }

  model->surfaceTangents(normals, tangents);
  return tangentCells;
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 2) {
    mexErrMsgIdAndTxt(
        "Drake:surfaceTangentsmex:NotEnoughInputs",
        "Usage: [d] = surfaceTangentsmex(mex_model_ptr, normals)");
  }

  RigidBodyTree *model = (RigidBodyTree *)getDrakeMexPointer(prhs[0]);
  const size_t numNormals = mxGetN(prhs[1]);  // number of normal vectors
  const size_t dimNormals = mxGetM(prhs[1]);  // dimension of each normal vector

  if (dimNormals != 3) {
    mexErrMsgIdAndTxt("Drake:surfaceTangentsmex:InvalidNormalDimensions",
                      "Normals must be 3-Dimensional");
  }

  // Mapping Eigen Matrix to existing memory to avoid copy overhead
  Map<Matrix3Xd> normals(mxGetPrSafe(prhs[1]), 3, numNormals);

  if (nlhs > 0) {
    plhs[0] = getTangentsArray(model, normals);
  }
}
