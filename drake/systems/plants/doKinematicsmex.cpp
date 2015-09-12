#include "mex.h"
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 5 || nlhs != 0) {
    mexErrMsgIdAndTxt("Drake:doKinematicsmex:NotEnoughInputs", "Usage doKinematicsmex(mex_model_ptr, kinematics_cache_ptr, q, v, compute_JdotV)");
  }

  RigidBodyManipulator *model = (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);
  KinematicsCache<double>* cache = static_cast<KinematicsCache<double>*>(getDrakeMexPointer(prhs[1]));
  auto q = matlabToEigenMap<Dynamic, 1>(prhs[2]);
  auto v = matlabToEigenMap<Dynamic, 1>(prhs[3]);
  bool compute_Jdotv = mxIsLogicalScalarTrue(prhs[4]);

  if (v.size() == 0 && model->num_velocities != 0)
    cache->initialize(q);
  else
    cache->initialize(q, v);
  model->doKinematics(*cache, compute_Jdotv);
}
