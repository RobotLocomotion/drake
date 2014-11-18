#include <mex.h>
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "drakeGeometryUtil.h"
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the geometricJacobian function
 *
 * Call with [J, vIndices] = geometricJacobianmex(model_ptr, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind);
 */

// TODO: stop copying these functions everywhere and find a good place for them
template<int RowsAtCompileTime, int ColsAtCompileTime>
mxArray* eigenToMatlab(Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(), m.cols(), mxREAL);
  if (m.rows() * m.cols() > 0)
    memcpy(mxGetPr(pm), m.data(), sizeof(double) * m.rows() * m.cols());
  return pm;
}

mxArray* stdVectorToMatlab(const std::vector<int>& vec) {
//  mxArray* pm = mxCreateNumericMatrix(vec.size(), 1, mxINT32_CLASS, mxREAL);
//  if (vec.size() > 0) {
//    memcpy(mxGetPr(pm), vec.data(), sizeof(int) * vec.size());
//  }
//  return pm;

  mxArray* pm = mxCreateDoubleMatrix(vec.size(), 1, mxREAL);
  for (int i = 0; i < vec.size(); i++) {
    mxGetPr(pm)[i] = (double) vec[i];
  }
  return pm;
}

void baseZeroToBaseOne(std::vector<int>& vec) {
  for (auto& val : vec) {
    val++;
  }
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  std::string usage = "Usage [J, vIndices] = geometricJacobianmex(model_ptr, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind)";
  if (nrhs != 4) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int base_body_or_frame_ind = ((int) mxGetScalar(prhs[1])) - 1; // base 1 to base 0
  int end_effector_body_or_frame_ind = ((int) mxGetScalar(prhs[2])) - 1; // base 1 to base 0
  int expressed_in_body_or_frame_ind = ((int) mxGetScalar(prhs[3])) - 1; // base 1 to base 0

  Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> J(TWIST_SIZE, 1);

  if (nlhs > 1) {
    std::vector<int> v_indices;
    model->geometricJacobian(base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, J, &v_indices);

    baseZeroToBaseOne(v_indices);
    plhs[1] = stdVectorToMatlab(v_indices);
  }
  else if (nlhs > 0){
    model->geometricJacobian(base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, J, nullptr);
  }
  if (nlhs > 0) {
    plhs[0] = eigenToMatlab(J);
  }
}
