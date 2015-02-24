#include "mex.h"
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage doKinematicsmex(model_ptr,q,compute_gradients,v,compute_JdotV)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  double *q, *v = nullptr;
  if (mxGetNumberOfElements(prhs[1]) != model->num_positions)
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "q must be size %d x 1", model->num_positions);
  q = mxGetPr(prhs[1]);
  bool compute_gradients = (bool) (mxGetLogicals(prhs[2]))[0];
  if (mxGetNumberOfElements(prhs[3]) > 0) {
    if (mxGetNumberOfElements(prhs[3]) != model->num_velocities)
      mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "v must be size %d x 1", model->num_velocities);
    v = mxGetPr(prhs[3]);
  }
  bool compute_Jdotv = (bool) (mxGetLogicals(prhs[4]))[0];

  model->doKinematicsNew(q, compute_gradients, v, compute_Jdotv);
}
