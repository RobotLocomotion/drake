#include "QPLocomotionPlan.h"
#include "drakeUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs != 5 || nlhs != 1) {
    mexErrMsgTxt("usage: lcm_msg_data = getQPControllerInput(obj, t_global, q, v, contact_force_detected);");
  }

  QPLocomotionPlan* plan = (QPLocomotionPlan*) getDrakeMexPointer(mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  double t_global = mxGetScalar(prhs[1]);
  auto q = matlabToEigenMap<Dynamic, 1>(prhs[2]);
  auto v = matlabToEigenMap<Dynamic, 1>(prhs[3]);
  auto contact_force_detected = matlabToStdVector<bool>(prhs[4]);
  drake::lcmt_qp_controller_input qp_controller_input = plan->createQPControllerInput(t_global, q, v, contact_force_detected);
  const size_t size = qp_controller_input.getEncodedSize();
  plhs[0] = mxCreateCharArray(1, &size);
  qp_controller_input.encode(mxGetChars(plhs[0]), 0, static_cast<int>(size));
}
