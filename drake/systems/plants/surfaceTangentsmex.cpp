#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include <Eigen/Dense>
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * Mex interface for computing friction cone surface tangent basis vectors given contact normals.
 *
 * MATLAB signature:
 * d = 
 *     surfaceTangents(mex_model_ptr, normals)
 */

inline mxArray* getTangentsArray(RigidBodyManipulator* const model, Map<Matrix3xd> const & normals)
{
  const int numContactPairs = normals.cols();
  const mwSize cellDims[] = {1, BASIS_VECTOR_HALF_COUNT};
  mxArray* tangentCells = mxCreateCellArray(2, cellDims);
  
  vector< Map<Matrix3xd> > tangents;
  for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++)
  {
    mxArray *cell = mxCreateDoubleMatrix(3, numContactPairs, mxREAL );    
    tangents.push_back(Map<Matrix3xd>(mxGetPrSafe(cell), 3, numContactPairs));
    mxSetCell(tangentCells, k, cell);
  }

  model->surfaceTangents(normals, tangents);
  return tangentCells;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  if (nrhs != 2) {
    mexErrMsgIdAndTxt("Drake:surfaceTangentsmex:NotEnoughInputs","Usage: [d] = surfaceTangentsmex(mex_model_ptr, normals)");
  }
  
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  const unsigned int numNormals = mxGetN(prhs[1]);  //number of normal vectors
  const unsigned int dimNormals = mxGetM(prhs[1]);  //dimension of each normal vector
  

  if (dimNormals != 3) {
  	mexErrMsgIdAndTxt("Drake:surfaceTangentsmex:InvalidNormalDimensions","Normals must be 3-Dimensional");	
  }

  //Mapping Eigen Matrix to existing memory to avoid copy overhead
  Map<Matrix3xd> normals(mxGetPrSafe(prhs[1]), 3, numNormals); 

  if (nlhs > 0) {
    plhs[0] = getTangentsArray(model, normals);
  }
}
