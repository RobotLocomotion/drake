#include <mex.h>
#include <iostream>
#include "drakeMexUtil.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "drakeGeometryUtil.h"
#include "math.h"

using namespace Eigen;
using namespace std;

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  std::string usage = "Usage [J, vIndices] = geometricJacobianmex(model_ptr, cache_ptr, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, in_terms_of_qdot)";
  if (nrhs != 6) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 3) {
    mexErrMsgIdAndTxt("Drake:geometricJacobianmex:WrongNumberOfOutputs", usage.c_str());
  }

  int arg_num = 0;
  RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[arg_num++]));
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[arg_num++]));

  int base_body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  int end_effector_body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  int expressed_in_body_or_frame_ind = ((int) mxGetScalar(prhs[arg_num++])) - 1; // base 1 to base 0
  bool in_terms_of_qdot = (bool) (mxGetLogicals(prhs[arg_num++]))[0];

  Eigen::Matrix<double, TWIST_SIZE, Eigen::Dynamic> J(TWIST_SIZE, 1);

  int gradient_order = nlhs > 2 ? 1 : 0;
  if (nlhs > 1) {
    std::vector<int> v_indices;
    auto J_gradientVar = model->geometricJacobian<double>(*cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, &v_indices);
    plhs[0] = eigenToMatlab(J_gradientVar.value());

    baseZeroToBaseOne(v_indices);
    plhs[1] = stdVectorToMatlab(v_indices);
    if (nlhs > 2) {
      plhs[2] = eigenToMatlab(J_gradientVar.gradient().value());
    }
  }
  else if (nlhs > 0){
    auto J_gradientVar = model->geometricJacobian<double>(*cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, nullptr);
    plhs[0] = eigenToMatlab(J_gradientVar.value());
  }

}
