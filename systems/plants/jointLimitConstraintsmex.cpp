#include "mex.h"
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  if (nrhs < 2 || nlhs < 2) {
    mexErrMsgIdAndTxt("Drake:jointLimitConstraintsmex:InvalidCall","Usage: [phi, J] = jointLimitConstraintsmex(mex_model_ptr, q) ");
  }

  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  if (mxGetNumberOfElements(prhs[1]) != model->num_positions) {
    mexErrMsgIdAndTxt("Drake:jointLimitConstraintsmex:InvalidPositionVectorLength", "q contains the wrong number of elements");
  }

  Map<VectorXd> q(mxGetPrSafe(prhs[1]),model->num_positions);
  
  size_t numJointConstraints = model->getNumJointLimitConstraints();

  plhs[0] = mxCreateDoubleMatrix(numJointConstraints, 1, mxREAL);
  plhs[1] = mxCreateDoubleMatrix(numJointConstraints, model->num_positions, mxREAL);

  Map<VectorXd> phi(mxGetPrSafe(plhs[0]), numJointConstraints);
  Map<MatrixXd> J(mxGetPrSafe(plhs[1]), numJointConstraints, model->num_positions);

  model->jointLimitConstraints(q, phi, J);
}

