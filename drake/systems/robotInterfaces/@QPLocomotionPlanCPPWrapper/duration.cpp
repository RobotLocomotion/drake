#include "QPLocomotionPlan.h"
#include "drakeMexUtil.h"

using namespace std;
using namespace Eigen;

// TODO: rename to getDuration after QPLocomotionPlan interface changes
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 1 || nlhs != 1) {
    mexErrMsgTxt("usage: ret = duration(obj);");
  }

  QPLocomotionPlan* plan = (QPLocomotionPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  plhs[0] = mxCreateDoubleScalar(plan->getDuration());
}
