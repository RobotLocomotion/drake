#include "drake/systems/robotInterfaces/QPLocomotionPlan.h"
#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 4 || nlhs != 1) {
    mexErrMsgTxt(
        "usage: lcm_msg_data = getQPControllerInput(obj, t_global, x, "
        "contact_force_detected);");
  }

  QPLocomotionPlan *plan = (QPLocomotionPlan *)getDrakeMexPointer(
      mxGetPropertySafe(prhs[0], "qp_locomotion_plan_ptr"));
  double t_global = mxGetScalar(prhs[1]);
  int nq = plan->getRobot().get_num_positions();
  int nv = plan->getRobot().get_num_velocities();
  auto q = Map<const VectorXd>(mxGetPrSafe(prhs[2]), nq);
  auto v = Map<const VectorXd>(mxGetPrSafe(prhs[2]) + nq, nv);
  auto contact_force_detected = matlabToStdVector<bool>(prhs[3]);
  drake::lcmt_qp_controller_input qp_controller_input =
      plan->createQPControllerInput(t_global, q, v, contact_force_detected);
  lcm::LCM lcm;
  lcm.publish("QP_CONTROLLER_INPUT", &qp_controller_input);
  const size_t size = qp_controller_input.getEncodedSize();
  plhs[0] = mxCreateNumericMatrix(size, 1, mxUINT8_CLASS, mxREAL);
  qp_controller_input.encode(mxGetData(plhs[0]), 0, static_cast<int>(size));
}
