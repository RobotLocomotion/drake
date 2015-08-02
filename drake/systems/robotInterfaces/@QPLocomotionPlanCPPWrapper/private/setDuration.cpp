#include "drakeMexUtil.h"
#include "QPLocomotionPlan.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nlhs != 0 || nrhs != 2) {
    mexErrMsgTxt("usage: setDuration(obj, duration);");
  }
  QPLocomotionPlan* plan = (QPLocomotionPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  double duration = mxGetScalar(prhs[1]);
  plan->setDuration(duration);
}
