#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

/*
 * MATLAB signature: [n, D, dn, dD] = contactConstraintsmex(model_ptr, normals, idxA, idxB, xA, xB, d)
 * 
 * Inputs:
 *  model_ptr = mex_model_ptr to the RBM
 *  normals = (3xm) contact normals (from B to A) for each of m possible contacts in world space
 *  idxA = 1-based body indexes of body A for m possible contacts
 *  idxB = 1-based body indexes of body B for m possible contacts
 *  xA = (3xm) matrix of contact positions in body A space
 *  xB = (3xm) matrix of contact positions in body B space 
 *  d = {k}(3xm) surface tangent basis vectors for the friction cone approximation in world space
 *     **NOTE** k represents the half-count for basis vectors. (e.g. k = 2 means 4 basis vectors)
 * Outputs:
 *   n = m x (num_dof) dphi/dq normal vectors in joint coordinates
 *   D = {2k} m x (num_dof) dD/dq surface tangent basis vectors in joint coordinates
 *   dn = m*num_dof x num_dof dn/dq second derivative of contact distances with respect to state
 *   dD = {2k} m*num_dof x numdof dD/dq second derivate of basis vectors with respect to state
 */

inline void buildSparseMatrix(Matrix3Xd const & pts, SparseMatrix<double> & sparse)
{
	typedef SparseMatrix<double>::Index SparseIndex;
  const SparseIndex m = static_cast<SparseIndex>(pts.cols());
  const SparseIndex numNonZero = 3*m;

  sparse.resize(m, numNonZero);
  sparse.reserve(VectorXi::Constant(numNonZero, 1));

	SparseIndex j = 0;
  for (SparseIndex i = 0 ; i < m ; i++) {
    for (SparseIndex k = 0 ; k < 3 ; k++) {
      sparse.insert(i, j) =  pts(j);
      j++;
    }
  }
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
  
  if (nrhs < 8) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:NotEnoughInputs","Usage: \n[n, D] = contactConstraintsmex(model_ptr, cache_ptr, normal, idxA, idxB, xA, xB, d)\n[n, D, dn, dD] = contactConstraintsmex(model_ptr, normal, idxA, idxB, xA, xB, d)");
  }

  if (nrhs >= 6 && mxGetN(prhs[3]) != mxGetN(prhs[4])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyIndexes", "idxA and idxB must be the same size");
  }

  if (nrhs >= 8 && mxGetN(prhs[5]) != mxGetN(prhs[6])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPoints", "xA and xB must be the same size");
  } 

  if (nrhs >= 8 && (mxGetM(prhs[5]) != 3 || mxGetM(prhs[6]) != 3)) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "body points xA and xB must be 3 dimensional");
  }

  if (nrhs >= 7 && mxGetN(prhs[2]) != mxGetN(prhs[5])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "normals must match the number of contact points");
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  const size_t numContactPairs = mxGetN(prhs[arg_num]);
  const Map<Matrix3Xd> normals(mxGetPrSafe(prhs[arg_num]), 3, numContactPairs); //contact normals in world space
  arg_num++;

  const bool compute_second_derivatives = nlhs > 3;


  if (model != NULL) {

    const Map<VectorXi> idxA((int*)mxGetData(prhs[arg_num++]), numContactPairs); //collision pairs index of body A
    const Map<VectorXi> idxB((int*)mxGetData(prhs[arg_num++]), numContactPairs); //collision pairs index of body B
    const Map<Matrix3Xd> xA(mxGetPrSafe(prhs[arg_num++]), 3, numContactPairs); //contact point in body A space
    const Map<Matrix3Xd> xB(mxGetPrSafe(prhs[arg_num++]), 3, numContactPairs); //contact point in body B space
    const size_t nq = model->num_positions;
    
    VectorXi idxA_zero_indexed = idxA.array() - 1;
    VectorXi idxB_zero_indexed = idxB.array() - 1;

    if (nlhs > 0) {
      MatrixXd J; 
      MatrixXd dJ;
      vector<int> bodyInds;
      SparseMatrix<double> sparseNormals;
      model->computeContactJacobians(*cache, idxA_zero_indexed, idxB_zero_indexed, xA, xB, compute_second_derivatives, J, dJ);
      buildSparseMatrix(normals, sparseNormals);
      plhs[0] = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
      Map<MatrixXd> n(mxGetPrSafe(plhs[0]), numContactPairs, nq);
      n = sparseNormals * J; //dphi/dq
      int d_arg_num = arg_num;
      const size_t* dims = mxGetDimensions(prhs[d_arg_num]);
      const mwSize num_tangent_vectors = dims[1];

      if (nlhs > 1) {
        const mwSize cellDims[] = {1, 2*num_tangent_vectors};
        plhs[1] = mxCreateCellArray(2, cellDims);
        if (nlhs > 3) {
          plhs[3] = mxCreateCellArray(2, cellDims);
        }
        for (int k = 0 ; k < num_tangent_vectors ; k++) { //for each friction cone basis vector
          Map<Matrix3Xd> dk(mxGetPrSafe(mxGetCell(prhs[d_arg_num], k)), 3, numContactPairs);
          SparseMatrix<double> sparseTangents;
          buildSparseMatrix(dk, sparseTangents);
          mxArray *D_cell = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
          mxArray *D_cell_reflected = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
          Map<MatrixXd> Dk(mxGetPrSafe(D_cell), numContactPairs, nq);
          Map<MatrixXd> Dk_reflected(mxGetPrSafe(D_cell_reflected), numContactPairs, nq);
          Dk = sparseTangents * J; //dd/dq
          Dk_reflected = -Dk;
          mxSetCell(plhs[1], k, D_cell);
          mxSetCell(plhs[1], k + num_tangent_vectors, D_cell_reflected);
          if (compute_second_derivatives) {
            if (nlhs > 3) {
              mxArray *dD_cell = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
              Map<MatrixXd> dD(mxGetPrSafe(dD_cell), numContactPairs , nq * nq);
              dD = sparseTangents * dJ;
              mxSetCell(plhs[3], k, dD_cell);
              mxArray *dD_cell_reflected = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
              Map<MatrixXd> dD_reflected(mxGetPrSafe(dD_cell_reflected), numContactPairs, nq*nq);
              dD_reflected = -dD;
              mxSetCell(plhs[3], k + num_tangent_vectors, dD_cell_reflected);
            }
          }
        }
      }
      
      if (nlhs > 2) {
       plhs[2] = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
       Map<MatrixXd> dn(mxGetPrSafe(plhs[2]), numContactPairs, nq*nq);
       dn = sparseNormals * dJ;
     }
   }
 }
}


