#include "mex.h"
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs == 1) {  // then assume the destructor is being called
    destroyDrakeMexPointer<KinematicsCache<double>*>(prhs[0]);
    return;
  }

  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage cache_ptr = doKinematicsmex(model_ptr,q,compute_gradients,v,compute_JdotV)");
  }

  // first get the model_ptr back from matlab
  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

  Map<VectorXd> q(mxGetPrSafe(prhs[1]), mxGetNumberOfElements(prhs[1]));
  if (q.rows() != model->num_positions)
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "q must be size %d x 1", model->num_positions);
  bool compute_gradients = (bool) (mxGetLogicals(prhs[2]))[0];
  bool compute_Jdotv = (bool) (mxGetLogicals(prhs[4]))[0];
  KinematicsCache<double>* cache = new KinematicsCache<double>(model->bodies, compute_gradients ? 1 : 0);
  if (mxGetNumberOfElements(prhs[3]) > 0) {
    auto v = Map<VectorXd>(mxGetPrSafe(prhs[3]), mxGetNumberOfElements(prhs[3]));
    if (v.rows() != model->num_velocities)
      mexErrMsgIdAndTxt("Drake:doKinematicsmex:BadInputs", "v must be size %d x 1", model->num_velocities);
    model->doKinematics(q, v, *cache, compute_Jdotv);
  }
  else {
    Map<VectorXd> v(nullptr, 0, 1);
    model->doKinematics(q, v, *cache, compute_Jdotv);
  }

  plhs[0] = createDrakeMexPointer((void*)cache, "KinematicsCache<double>");
}
