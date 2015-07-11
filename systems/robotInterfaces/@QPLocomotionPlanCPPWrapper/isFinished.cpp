#include "drakeMexUtil.h"
#include "QPLocomotionPlan.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nlhs != 1 || nrhs < 2) {
    mexErrMsgTxt("usage: is_finished = isFinished(obj, t, x);");
  }
  QPLocomotionPlan* plan = (QPLocomotionPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));

  double t = mxGetScalar(prhs[1]);
  bool is_finished = plan->isFinished(t);
  plhs[0] = mxCreateLogicalScalar(is_finished);
}
