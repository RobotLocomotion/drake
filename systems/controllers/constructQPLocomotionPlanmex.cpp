#include "mex.h"
#include "drakeUtil.h"
#include "QPLocomotionPlan.h"

using namespace std;

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  string usage = "usage: ptr = constructQPLocomotionPlanmex(mex_model_ptr, qp_locomotion_settings, lcm_channel);";
  if (nrhs < 1 || nrhs > 3)
    mexErrMsgTxt(usage.c_str());

  if (nrhs == 1) {
    // By convention, calling the constructor with just one argument (the pointer) should delete the pointer
    // TODO: make this not depend on number of arguments
    if (isa(prhs[0],"DrakeMexPointer")) {
      destroyDrakeMexPointer<QPLocomotionPlan*>(prhs[0]);
      return;
    } else {
      mexErrMsgIdAndTxt("Drake:constructQPLocomotionPlanmex:BadInputs", "Expected a DrakeMexPointer (or a subclass)");
    }
  }

  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");


  const mxArray* mex_model = prhs[0];
  const mxArray* mex_settings = prhs[1];
  const mxArray* mex_lcm_channel = prhs[2];

  // robot
  RigidBodyManipulator *robot = (RigidBodyManipulator*) getDrakeMexPointer(mex_model);

  // settings
  QPLocomotionPlanSettings settings;

  // lcm
  string lcm_channel = mxGetStdString(mex_lcm_channel);

  QPLocomotionPlan* plan = new QPLocomotionPlan(*robot, settings, lcm_channel);

  plhs[0] = createDrakeMexPointer((void*) plan, "QPLocomotionPlan");

  return;
}
