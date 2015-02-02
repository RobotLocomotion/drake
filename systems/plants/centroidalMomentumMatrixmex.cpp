#include <mex.h>
#include <iostream>
#include <memory>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"

using namespace Eigen;
using namespace std;

// TODO: stop copying these functions everywhere and find a good place for them
template <typename DerivedA>
mxArray* eigenToMatlab(const DerivedA &m)
{
 mxArray* pm = mxCreateDoubleMatrix(static_cast<int>(m.rows()),static_cast<int>(m.cols()),mxREAL);
 if (m.rows()*m.cols()>0)
   memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
 return pm;
}

template<int RowsAtCompileTime, int ColsAtCompileTime>
Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> matlabToEigen(const mxArray* matlab_array)
{
  const mwSize* size_array = mxGetDimensions(matlab_array);
  Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime> ret(size_array[0], size_array[1]);
  memcpy(ret.data(), mxGetPr(matlab_array), sizeof(double) * ret.size());
  return ret;
}

void mexFunction(int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]) {

  std::string usage = "Usage [A, dA] = centroidalMomentumMatrixmex()";
  if (nrhs != 1) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 2) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  int gradient_order = nlhs - 1;
  auto A = model->centroidalMomentumMatrix<double>(gradient_order);

  plhs[0] = eigenToMatlab(A.value());
  if (gradient_order > 0)
    plhs[1] = eigenToMatlab(A.gradient().value());
}
