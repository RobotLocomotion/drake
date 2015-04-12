#include <mex.h>
#include <iostream>
#include <Eigen/Dense>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

/*
 * A C version of the getCMM function
 */

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {

  if (nrhs < 2) {
    mexErrMsgIdAndTxt("Drake:getCMMmex:NotEnoughInputs","Usage getCMMmex(model_ptr,q_cache) or getCMMmex(model_ptr,q_cache,qd)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  Map<VectorXd> q(mxGetPrSafe(prhs[1]), model->num_positions);
  for (int i = 0; i < model->num_positions; i++) {
    if (q[i] - model->cached_q[i] > 1e-8 || q[i] - model->cached_q[i] < -1e-8) {
      mexErrMsgIdAndTxt("Drake:getCMMmex:InvalidKinematics","This kinsol is no longer valid.  Somebody has called doKinematics with a different q since the solution was computed.");
    }
  }

  plhs[0] = mxCreateDoubleMatrix(6,model->num_positions,mxREAL);
  Map<MatrixXd> A(mxGetPrSafe(plhs[0]),6,model->num_positions);
  plhs[1] = mxCreateDoubleMatrix(6,model->num_positions,mxREAL);
  Map<MatrixXd> Adot(mxGetPrSafe(plhs[1]),6,model->num_positions);

  
  if (nrhs > 2) {
    Map<VectorXd> qd(mxGetPrSafe(prhs[2]), model->num_velocities);
    model->getCMM(q,qd,A,Adot);
  }
  else {
    VectorXd zeros = VectorXd::Zero(model->num_positions);
    Map<VectorXd> qd(zeros.data(), model->num_velocities);
    model->getCMM(q,qd,A,Adot);
  }
}
