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

  mxGetFieldByNumber(mex_settings, 0, 0);

  settings.duration = mxGetScalar(mxGetPropertySafe(mex_settings, "duration"));
//  std::vector<RigidBodySupportState> supports;
//  std::vector<double> support_times; // length: supports.size() + 1
//  typedef std::map<std::string, Eigen::Matrix3Xd> ContactGroupNameToContactPointsMap;
//  std::vector<ContactGroupNameToContactPointsMap> contact_groups; // one for each support

//  std::vector<BodyMotionData> body_motions;
//  PiecewisePolynomial<double> zmp_trajectory;
  settings.zmp_final = matlabToEigen<2, 1>(mxGetPropertySafe(mex_settings, "zmp_final"));
  settings.lipm_height = mxGetScalar(mxGetPropertySafe(mex_settings, "lipm_height"));
//  QuadraticLyapunovFunction V;
//  PiecewisePolynomial<double> q_traj;
//  ExponentialPlusPiecewisePolynomial<double> com_traj;
//  drake::lcmt_qp_controller_input default_qp_input;

  settings.gain_set = mxGetStdString(mxGetPropertySafe(mex_settings, "gain_set"));
  settings.mu = mxGetScalar(mxGetPropertySafe(mex_settings, "mu"));
//  settings.plan_shift_zmp_indices = matlabToStdVector(mxGetPropertySafe(mex_settings, "plan_shift_zmp_indices")); // TODO enable after making QPLocomotionPlan changes
//  settings.plan_shift_body_motion_indices = matlabToStdVector(mxGetPropertySafe(mex_settings, "plan_shift_body_motion_indices"));  // TODO enable after making QPLocomotionPlan changes
  settings.g = mxGetScalar(mxGetPropertySafe(mex_settings, "g"));
  settings.is_quasistatic = mxGetLogicals(mxGetPropertySafe(mex_settings, "is_quasistatic"))[0];
  settings.knee_settings.min_knee_angle = mxGetScalar(mxGetPropertySafe(mex_settings, "min_knee_angle"));
  settings.knee_settings.knee_kp = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_kp"));
  settings.knee_settings.knee_kd = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_kd"));
  settings.knee_settings.knee_weight = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_weight"));
  settings.pelvis_name = mxGetStdString(mxGetPropertySafe(mex_settings, "pelvis_name"));
  settings.foot_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "l_foot_name"));
  settings.foot_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "r_foot_name"));
//  std::vector<std::string> constrained_joint_name_parts = createDefaultConstrainedJointNameParts();







  // lcm
  string lcm_channel = mxGetStdString(mex_lcm_channel);

  QPLocomotionPlan* plan = new QPLocomotionPlan(*robot, settings, lcm_channel);

  plhs[0] = createDrakeMexPointer((void*) plan, "QPLocomotionPlan");

  return;
}
