#include "drake/systems/robotInterfaces/QPLocomotionPlan.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs < 1 || nlhs != 1) {
    mexErrMsgTxt("usage: lcm_msg_data = getLastQPInput(obj);");
  }

  QPLocomotionPlan *plan = (QPLocomotionPlan *)getDrakeMexPointer(
      mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  drake::lcmt_qp_controller_input qp_controller_input = plan->getLastQPInput();
  const size_t size = qp_controller_input.getEncodedSize();
  plhs[0] = mxCreateNumericMatrix(size, 1, mxUINT8_CLASS, mxREAL);
  qp_controller_input.encode(mxGetData(plhs[0]), 0, static_cast<int>(size));
}
