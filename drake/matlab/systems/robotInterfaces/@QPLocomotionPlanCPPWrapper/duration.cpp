#include "drake/systems/robotInterfaces/QPLocomotionPlan.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

// TODO(tkoolen): rename to getDuration after QPLocomotionPlan interface changes
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 1 || nlhs != 1) {
    mexErrMsgTxt("usage: ret = duration(obj);");
  }

  QPLocomotionPlan *plan = (QPLocomotionPlan *)getDrakeMexPointer(
      mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  plhs[0] = mxCreateDoubleScalar(plan->getDuration());
}
