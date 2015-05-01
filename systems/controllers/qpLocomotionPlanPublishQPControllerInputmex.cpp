#include "QPLocomotionPlan.h"
#include "drakeUtil.h"

using namespace std;
using namespace Eigen;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  string usage = "usage: qpLocomotionPlanPublishQPControllerInputmex(mex_locomotion_plan_ptr, t_global, q, v, contact_force_detected);";

  if (nrhs != 5) {
    mexErrMsgIdAndTxt("Drake:qpLocomotionPlanPublishQPControllerInputmex:WrongNumberOfInputs", usage.c_str());
  }

  if (nlhs > 0) {
    mexErrMsgIdAndTxt("Drake:qpLocomotionPlanPublishQPControllerInputmex:WrongNumberOfOutputs", usage.c_str());
  }

  QPLocomotionPlan *plan = (QPLocomotionPlan*) getDrakeMexPointer(prhs[0]);
  double t_global = mxGetScalar(prhs[1]);
  auto q = matlabToEigenMap<Dynamic, 1>(prhs[2]);
  auto v = matlabToEigenMap<Dynamic, 1>(prhs[3]);
  auto contact_force_detected = matlabToStdVector<bool>(prhs[4]);
  plan->publishQPControllerInput(t_global, q, v, contact_force_detected);
}
