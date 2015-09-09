#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"

using namespace Eigen;
using namespace std;

/**
 * usage:
 * * createKinematicsCachemex(model_ptr, include_gradients) to construct
 * * createKinematicsCachemex(cache_ptr) to destruct
 */
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nlhs == 0 && nrhs == 1) {
    // if a KinematicsCache<double> is passed in, then assume the destructor is being called
    destroyDrakeMexPointer<KinematicsCache<double>*>(prhs[0]);
  }
  else if (nlhs == 1 && nrhs == 2) {
    RigidBodyManipulator *model = static_cast<RigidBodyManipulator*>(getDrakeMexPointer(prhs[0]));
    bool include_gradients = mxIsLogicalScalarTrue(prhs[1]);
    KinematicsCache<double>* cache = new KinematicsCache<double>(model->bodies, include_gradients ? 1 : 0);
    plhs[0] = createDrakeMexPointer((void*)cache, "KinematicsCache<double>");
  }
  else
    mexErrMsgTxt("couldn't parse input");
}
