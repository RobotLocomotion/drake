#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

/*
 * MATLAB signature: [d, n, D, dn, dD] = contactConstraintsmex(model_ptr, normals, idxA, idxB, xA, xB)
 * 
 * Inputs:
 *  model_ptr = mex_model_ptr to the RBM
 *  normals = (3xm) contact normals (from B to A) for each of m possible contacts in world space
 *  idxA = 1-based body indexes of body A for m possible contacts
 *  idxB = 1-based body indexes of body B for m possible contacts
 *  xA = (3xm) matrix of contact positions in body A space
 *  xB = (3xm) matrix of contact positions in body B space 
 * Outputs:
 *  d = {k}(3xm) surface tangent basis vectors for the friction cone approximation in world space
 *     **NOTE** k represents the half-count for basis vectors. (e.g. k = 2 means 4 basis vectors)
 *   n = m x (num_dof) dphi/dq normal vectors in joint coordinates
 *   D = {2k} m x (num_dof) dD/dq surface tangent basis vectors in joint coordinates
 *   dn = m*num_dof x num_dof dn/dq second derivative of contact distances with respect to state
 *   dD = {2k} m*num_dof x numdof dD/dq second derivate of basis vectors with respect to state
 */

inline void buildSparseMatrix(Matrix3xd const & pts, SparseMatrix<double> & sparse)
{
  const int m = pts.cols();
  const int numNonZero = 3*m;

  sparse.resize(m, numNonZero);
  sparse.reserve(VectorXi::Constant(numNonZero, 1));

  int j = 0;
  for (int i = 0 ; i < m ; i++) {
    for (int k = 0 ; k < 3 ; k++) {
     sparse.insert(i, j) =  pts(j);
     j++;
   }
 }
}

//forms a mex cell array and computes the surface tangents in-place using maps
inline mxArray* getTangentsArray(RigidBodyManipulator* const model, Map<Matrix3xd> const & normals)
{
  const int numContactPairs = normals.cols();
  const mwSize cellDims[] = {1, BASIS_VECTOR_HALF_COUNT};
  mxArray* tangentCells = mxCreateCellArray(2, cellDims);
  
  vector< Map<Matrix3xd> > tangents;
  for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) {
    mxArray *cell = mxCreateDoubleMatrix(3, numContactPairs, mxREAL );    
    tangents.push_back(Map<Matrix3xd>(mxGetPrSafe(cell), 3, numContactPairs));
    mxSetCell(tangentCells, k, cell);
  }

  model->surfaceTangents(normals, tangents);

  return tangentCells;
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  
  if (nrhs < 6) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:NotEnoughInputs","Usage: \n[d, n, D] = contactConstraintsmex(normal, model_ptr, idxA, idxB, xA, xB)\n[d, n, D, dn, dD] = contactConstraintsmex(normal, model_ptr, idxA, idxB, xA, xB)");
  }

  if (nrhs >= 4 && mxGetN(prhs[2]) != mxGetN(prhs[3])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyIndexes", "idxA and idxB must be the same size");
  }

  if (nrhs >= 6 && mxGetN(prhs[4]) != mxGetN(prhs[5])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPoints", "xA and xB must be the same size");
  } 

  if (nrhs >= 6 && (mxGetM(prhs[4]) != 3 || mxGetM(prhs[5]) != 3)) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "body points xA and xB must be 3 dimensional");
  }

  if (nrhs >= 5 && mxGetN(prhs[1]) != mxGetN(prhs[4])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "normals must match the number of contact points");
  }

  const int numContactPairs = mxGetN(prhs[1]);
  const Map<Matrix3xd> normals(mxGetPrSafe(prhs[1]), 3, numContactPairs); //contact normals in world space
  const bool compute_second_derivatives = nlhs > 3;

  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  if (model != NULL) {
    if (nlhs > 0) {
      plhs[0] = getTangentsArray(model, normals);
    }

    const Map<VectorXi> idxA((int*)mxGetData(prhs[2]), numContactPairs); //collision pairs index of body A
    const Map<VectorXi> idxB((int*)mxGetData(prhs[3]), numContactPairs); //collision pairs index of body B
    const Map<Matrix3xd> xA(mxGetPrSafe(prhs[4]), 3, numContactPairs); //contact point in body A space
    const Map<Matrix3xd> xB(mxGetPrSafe(prhs[5]), 3, numContactPairs); //contact point in body B space
    const int nq = model->num_positions;
    
    if (nlhs > 1) {
      MatrixXd J; 
      MatrixXd dJ;
      vector<int> bodyInds;
      SparseMatrix<double> sparseNormals;
      model->computeContactJacobians(idxA, idxB, xA, xB, compute_second_derivatives, J, dJ);      
      buildSparseMatrix(normals, sparseNormals);
      plhs[1] = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
      Map<MatrixXd> n(mxGetPrSafe(plhs[1]), numContactPairs, nq);
      n = sparseNormals * J; //dphi/dq
      if (nlhs > 2) {
        const mwSize cellDims[] = {1, 2*BASIS_VECTOR_HALF_COUNT};
        plhs[2] = mxCreateCellArray(2, cellDims);
        if (nlhs > 4) {
          plhs[4] = mxCreateCellArray(2, cellDims);
        }
        for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) { //for each friction cone basis vector
          Map<Matrix3xd> dk(mxGetPrSafe(mxGetCell(plhs[0], k)), 3, numContactPairs);
          SparseMatrix<double> sparseTangents;
          buildSparseMatrix(dk, sparseTangents);
          mxArray *D_cell = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
          mxArray *D_cell_reflected = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
          Map<MatrixXd> Dk(mxGetPrSafe(D_cell), numContactPairs, nq);
          Map<MatrixXd> Dk_reflected(mxGetPrSafe(D_cell_reflected), numContactPairs, nq);
          Dk = sparseTangents * J; //dd/dq
          Dk_reflected = -Dk;
          mxSetCell(plhs[2], k, D_cell);
          mxSetCell(plhs[2], k + BASIS_VECTOR_HALF_COUNT, D_cell_reflected);
          if (compute_second_derivatives) {
            if (nlhs > 4) {
              mxArray *dD_cell = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
              Map<MatrixXd> dD(mxGetPrSafe(dD_cell), numContactPairs , nq * nq);
              dD = sparseTangents * dJ;
              mxSetCell(plhs[4], k, dD_cell);
              mxArray *dD_cell_reflected = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
              Map<MatrixXd> dD_reflected(mxGetPrSafe(dD_cell_reflected), numContactPairs, nq*nq);
              dD_reflected = -dD;
              mxSetCell(plhs[4], k + BASIS_VECTOR_HALF_COUNT, dD_cell_reflected);
            }
          }
        }
      }
      
      if (nlhs > 3) {
       plhs[3] = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
       Map<MatrixXd> dn(mxGetPrSafe(plhs[3]), numContactPairs, nq*nq);
       dn = sparseNormals * dJ;
     }
   }
 }
}


