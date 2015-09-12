#include "mex.h"
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  if (nrhs < 2 || nlhs < 2) {
    mexErrMsgIdAndTxt("Drake:positionConstraintsmex:InvalidCall","Usage: [phi, J] = positionConstraintsmex(mex_model_ptr, q) ");
  }

  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  
  const size_t nq = model->num_positions;
  
  if (mxGetNumberOfElements(prhs[1]) != nq) {
    mexErrMsgIdAndTxt("Drake:positionConstraintsmex:InvalidPositionVectorLength", "q contains the wrong number of elements");
  }

  Map<VectorXd> q(mxGetPrSafe(prhs[1]),nq);

  KinematicsCache<double> cache = model->doKinematics(q, 0); // FIXME: KinematicsCache should be passed in!

  auto phi = model->positionConstraints<double>(cache,1);
  plhs[0] = eigenToMatlab(phi.value());
  plhs[1] = eigenToMatlab(phi.gradient().value());
}

