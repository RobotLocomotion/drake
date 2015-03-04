#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

/*
 * Mex interface for contactConstraints
 * MATLAB signature: [n, D, dn, dD] = contactConstraintsmex(model_ptr, idxA, idxB, xA, xB, normal, d)
 * 
 * Inputs:
 *  model_ptr = mex_model_ptr to the RBM
 *  idxA = 1-based body indexes of body A for m possible contacts
 *  idxB = 1-based body indexes of body B for m possible contacts
 *  xA = (3xm) matrix of contact positions in body A space
 *  xB = (3xm) matrix of contact positions in body B space
 *  normal = (3xm) contact normals (from B to A) for each of m possible contacts in world space
 *  d = {k}(3xm) surface tangent basis vectors for the friction cone approximation in world space
 *     **NOTE** k represents the half-count for basis vectors. (e.g. k = 2 means 4 basis vectors)
 * Outputs:
 *   n = dphi/dq normal vectors in joint coordinates
 *   D = dD/dq surface tangent basis vectors in joint coordinates
 *   dn = dn/dq second derivative of contact distances with respect to state
 *   dD = dD/dq second derivate of basis vectors with respect to state
 */

typedef Matrix<double, 3, Dynamic> Matrix3xd;

inline void getUniqueBodiesSorted(VectorXi const & idxA, VectorXi const & idxB, vector<int> & bodyIndsSorted)
{
  size_t m = idxA.size();
  set<int> bodyInds;

  for(int i = 0 ; i < m ; i++)
  {  
    bodyInds.insert(idxA[i]);
    bodyInds.insert(idxB[i]);
  }

  bodyIndsSorted.clear();

  for(set<int>::const_iterator citer = bodyInds.begin() ; citer != bodyInds.end() ; citer++)
  {
    if( *citer > 1 )
    {
      bodyIndsSorted.push_back(*citer);
    }
  }

  sort(bodyIndsSorted.begin(), bodyIndsSorted.end());
}

inline void getBodyPoints(vector<int> const & cindA, vector<int> const & cindB, Matrix3xd const & xA, Matrix3xd const & xB, const int num_dof, MatrixXd & bodyPoints)
{
  int i = 0;
  int numPtsA = cindA.size();
  int numPtsB = cindB.size();

  bodyPoints.resize(4, numPtsA + numPtsB);

  for(i = 0 ; i < numPtsA ; i++ ) {
    bodyPoints.col(i) << xA.col(cindA[i]) , 1; //homogeneous coordinates
  }
  
  for(i = 0 ; i < numPtsB ; i++ ) {
    bodyPoints.col(numPtsA + i) << xB.col(cindB[i]), 1;
  }
}

inline void findContactIndexes(VectorXi const & idxList, const int bodyIdx, vector<int> & contactIdx)
{
  int m = idxList.size();
  contactIdx.clear();
  for(int i = 0 ; i < m ; i++) {
    if(idxList[i] == bodyIdx) {
      contactIdx.push_back(i); //zero-based index 
    }
  }
}

inline void accumulateJacobian(RigidBodyManipulator *model, const int bodyInd, MatrixXd const & bodyPoints, vector<int> const & cindA, vector<int> const & cindB, MatrixXd & J)
{
      const int nq = J.cols();
      const int numPts = bodyPoints.cols();
      const size_t numCA = cindA.size();
      const size_t numCB = cindB.size();
      const size_t offset = 3*numCA;

      MatrixXd J_tmp(3*numPts, nq);
      model->forwardJac(bodyInd - 1, bodyPoints, 0, J_tmp);

      //add contributions from points in xA
      for(int x = 0 ; x < numCA ; x++)
      {
        J.block(3*cindA[x], 0, 3, nq) += J_tmp.block(3*x, 0, 3, nq);
      }

      //subtract contributions from points in xB
      for(int x = 0 ; x < numCB ; x++)
      {
        J.block(3*cindB[x], 0, 3, nq) -= J_tmp.block(offset + 3*x, 0, 3, nq);
      }
}

inline void accumulateSecondOrderJacobian(RigidBodyManipulator *model, const int bodyInd, MatrixXd const & bodyPoints, vector<int> const & cindA, vector<int> const & cindB, MatrixXd & dJ)
{
      const int dJCols = dJ.cols(); //nq^2 instead of nq
      const int numPts = bodyPoints.cols();
      const size_t numCA = cindA.size();
      const size_t numCB = cindB.size();
      const size_t offset = 3*numCA;
      MatrixXd dJ_tmp(3*numPts, dJCols);
      model->forwarddJac(bodyInd - 1, bodyPoints, dJ_tmp); //dJac instead of Jac

      //add contributions from points in xA
      for(int x = 0 ; x < numCA ; x++)
      {
        dJ.block(3*cindA[x], 0, 3, dJCols) += dJ_tmp.block(3*x, 0, 3, dJCols);
      }

      //subtract contributions from points in xB
      for(int x = 0 ; x < numCB ; x++)
      {
        dJ.block(3*cindB[x], 0, 3, dJCols) -= dJ_tmp.block(offset + 3*x, 0, 3, dJCols);
      }
}

inline void buildSparseMatrix(Matrix3xd const & pts, SparseMatrix<double> & sparse)
{
  const int m = pts.cols();
  const int numNonZero = 3*m;
  
  sparse.resize(m, numNonZero);
  sparse.reserve(VectorXi::Constant(numNonZero, 1));

  int j = 0;
  for(int i = 0 ; i < m ; i++) {
    for(int k = 0 ; k < 3 ; k++) {
      sparse.insert(i, j) =  pts(j);
      j++;
    }
  }
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 7) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:NotEnoughInputs","Usage [n, D, dn, dD] = contactConstraintsmex(model_ptr, idxA, idxB, xA, xB, normal, d)");
  }

  if(!mxIsCell(prhs[6])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBasisVectors", "d must be a cell array");
  }

  if(mxGetN(prhs[1]) != mxGetN(prhs[2])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyIndexes", "idxA and idxB must be the same size");
  }

  if(mxGetN(prhs[3]) != mxGetN(prhs[4])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPoints", "xA and xB must be the same size");
  }

  if(mxGetM(prhs[3]) != 3 || mxGetM(prhs[4]) != 3) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "body points xA and xB must be 3 dimensional");
  }

  if(mxGetN(prhs[3]) != mxGetN(prhs[5])) {
    mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "normals must match the number of contact points");
  }

  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  if(model != NULL) {

    const int numContactPairs = mxGetNumberOfElements(prhs[1]);
    const int nq = model->num_positions;
    const int nk = mxGetN(prhs[6]); //half the number of friction cone basis vectors
    const Map<VectorXi> idxA((int*)mxGetData(prhs[1]), numContactPairs); //collision pairs index of body A
    const Map<VectorXi> idxB((int*)mxGetData(prhs[2]), numContactPairs); //collision pairs index of body B
    const Map<Matrix3xd> xA(mxGetPr(prhs[3]), 3, numContactPairs); //contact point in body A space
    const Map<Matrix3xd> xB(mxGetPr(prhs[4]), 3, numContactPairs); //contact point in body B space
    const Map<Matrix3xd> normals(mxGetPr(prhs[5]), 3, numContactPairs); //contact normals in world space
    const bool compute_first_derivatives = nlhs > 0;
    const bool compute_second_derivatives = nlhs > 2;
    
    if(!compute_first_derivatives) {
      return;
    }

    MatrixXd J = MatrixXd::Zero(3*numContactPairs, nq);
    MatrixXd dJ = MatrixXd::Zero(3*numContactPairs, nq*nq);
    vector<int> bodyInds;
    
    getUniqueBodiesSorted(idxA, idxB, bodyInds);

    for(vector<int>::const_iterator citer = bodyInds.begin() ; citer != bodyInds.end() ; citer++) {
        const int bodyInd = *citer;
        vector<int> cindA, cindB;
        MatrixXd bodyPoints;
        findContactIndexes(idxA, bodyInd, cindA);
        findContactIndexes(idxB, bodyInd, cindB);
        getBodyPoints(cindA, cindB, xA, xB, nq, bodyPoints);
        accumulateJacobian(model, bodyInd, bodyPoints, cindA, cindB, J);
        if(compute_second_derivatives)
        {
          accumulateSecondOrderJacobian(model, bodyInd, bodyPoints, cindA, cindB, dJ);
        }
    }

    SparseMatrix<double> sparseNormals;
    buildSparseMatrix(normals, sparseNormals);
    
    if(nlhs > 0) {
      plhs[0] = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
      Map<MatrixXd> n(mxGetPr(plhs[0]), numContactPairs, nq);
      n = sparseNormals * J; //dphi/dq
    }

    if(nlhs > 1) {
      const mwSize cellDims[] = {1, static_cast<mwSize>(2*nk)};
      plhs[1] = mxCreateCellArray(2, cellDims);
      
      if(nlhs > 3) {
        plhs[3] = mxCreateCellArray(2, cellDims);
      }
      
      for(int k = 0 ; k < nk ; k++) { //for each friction cone basis vector
        Map<Matrix3xd> dk(mxGetPr(mxGetCell(prhs[6], k)), 3, numContactPairs);
        SparseMatrix<double> sparseTangents;
        buildSparseMatrix(dk, sparseTangents);
        mxArray *D_cell = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
        mxArray *D_cell_reflected = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
        Map<MatrixXd> Dk(mxGetPr(D_cell), numContactPairs, nq);
        Map<MatrixXd> Dk_reflected(mxGetPr(D_cell_reflected), numContactPairs, nq);
        Dk = sparseTangents * J; //dd/dq
        Dk_reflected = -Dk;
        mxSetCell(plhs[1], k, D_cell);
        mxSetCell(plhs[1], k + nk, D_cell_reflected);
        if(compute_second_derivatives) {
          if(nlhs > 3){
            mxArray *dD_cell = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
            Map<MatrixXd> dD(mxGetPr(dD_cell), numContactPairs , nq * nq);
            dD = sparseTangents * dJ;
            mxSetCell(plhs[3], k, dD_cell);
            mxArray *dD_cell_reflected = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
            Map<MatrixXd> dD_reflected(mxGetPr(dD_cell_reflected), numContactPairs, nq*nq);
            dD_reflected = -dD;
            mxSetCell(plhs[3], k + nk, dD_cell_reflected);
          }
        }
      }
    }

    if(nlhs > 2) {
      plhs[2] = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
      Map<MatrixXd> dn(mxGetPr(plhs[2]), numContactPairs, nq*nq);
      dn = sparseNormals * dJ;
    }

  }
}


