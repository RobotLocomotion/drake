#include "drakeMexUtil.h"
#include "QPLocomotionPlan.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nlhs != 0 || nrhs != 2) {
    mexErrMsgTxt("usage: setStartTime(obj, t);");
  }
  QPLocomotionPlan* plan = (QPLocomotionPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  double start_time = mxGetScalar(prhs[1]);
  plan->setStartTime(start_time);
}
