#include "drake/systems/robotInterfaces/QPLocomotionPlan.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

// TODO(tkoolen): rename to getStartTime after QPLocomotionPlan interface
// changes
DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 1 || nlhs != 1) {
    mexErrMsgTxt("usage: time = start_time(obj);");
  }

  QPLocomotionPlan *plan = (QPLocomotionPlan *)getDrakeMexPointer(
      mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  double start_time = plan->getStartTime();
  if (std::isnan(start_time)) {
    plhs[0] = mxCreateDoubleMatrix(0, 0, mxREAL);
  } else {
    plhs[0] = mxCreateDoubleScalar(start_time);
  }
}
