#include "drakeUtil.h"
#include "drakeMexUtil.h"
#include "QPLocomotionPlan.h"

using namespace std;
using namespace Eigen;

/*
 * see Matlab's mkpp and ppval
 */
PiecewisePolynomial<double> matlabPPFormToPiecewisePolynomial(const mxArray* pp)
{
  vector<double> breaks = matlabToStdVector<double>(mxGetFieldSafe(pp, "breaks"));
  size_t num_segments = breaks.size() - 1; // l

  const mxArray* coefs_mex = mxGetFieldSafe(pp, "coefs"); // a d*l x k matrix
  const size_t* coefs_mex_dims = mxGetDimensions(coefs_mex);
  int num_coefs_mex_dims = mxGetNumberOfDimensions(coefs_mex);

  size_t number_of_elements = mxGetNumberOfElements(coefs_mex);
  if (!mxIsDouble(coefs_mex))
    mexErrMsgIdAndTxt("Drake:matlabPPFormToPiecewisePolynomial:BadInputs", "coefs should have type double");

  const mxArray* dim_mex = mxGetFieldSafe(pp, "dim");
  int num_dims_mex = mxGetNumberOfElements(dim_mex);
  if (num_dims_mex == 0 | num_dims_mex > 2)
    throw runtime_error("case not handled"); // because PiecewisePolynomial can't currently handle it
  const int num_dims = 2;
  mwSize dims[num_dims];
  if (!mxIsDouble(dim_mex))
    mexErrMsgIdAndTxt("Drake:matlabPPFormToPiecewisePolynomial:BadInputs", "dim should have type double");
  for (int i = 0; i < num_dims_mex; i++) {
    dims[i] = static_cast<mwSize>(mxGetPr(dim_mex)[i]);
  }
  for (int i = num_dims_mex; i < num_dims; i++)
    dims[i] = 1;

  size_t product_of_dimensions = dims[0]; // d
  for (int i = 1; i < num_dims; ++i) {
    product_of_dimensions *= dims[i];
  }

  size_t num_coefficients = number_of_elements / (num_segments * product_of_dimensions); // k

  vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomial_matrices;
  polynomial_matrices.reserve(num_segments);
  for (mwSize segment_index = 0; segment_index < num_segments; segment_index++) {
    PiecewisePolynomial<double>::PolynomialMatrix polynomial_matrix(dims[0], dims[1]);
    for (mwSize i = 0; i < product_of_dimensions; i++) {
      VectorXd coefficients(num_coefficients);
      mwSize row = segment_index * product_of_dimensions + i;
      for (mwSize coefficient_index = 0; coefficient_index < num_coefficients; coefficient_index++) {
        mwSize sub[] = {row, num_coefficients - coefficient_index - 1}; // Matlab's reverse coefficient indexing...
        coefficients[coefficient_index] = *(mxGetPr(coefs_mex) + sub2ind(num_coefs_mex_dims, coefs_mex_dims, sub));
      }
      polynomial_matrix(i) = Polynomial<double>(coefficients);
    }
    polynomial_matrices.push_back(polynomial_matrix);
  }

  return PiecewisePolynomial<double>(polynomial_matrices, breaks);
}

PiecewisePolynomial<double> matlabPPTrajectoryOrMatrixToPiecewisePolynomial(const mxArray* array)
{
  if (mxIsNumeric(array)) {
    return PiecewisePolynomial<double>(matlabToEigenMap<Dynamic, Dynamic>(array));
  }
  else {
    return matlabPPFormToPiecewisePolynomial(mxGetPropertySafe(array, "pp"));
  }
}

PiecewisePolynomial<double> matlabCoefsAndBreaksToPiecewisePolynomial(const mxArray* mex_coefs, const mxArray* mex_breaks, bool flip_last_dimension)
{
  if (!mxIsDouble(mex_coefs))
    mexErrMsgIdAndTxt("Drake:matlabPPFormToPiecewisePolynomial:BadInputs", "coefs should have type double");

  int num_dims = 3;
  mwSize dims[num_dims];
  size_t num_dims_mex = mxGetNumberOfDimensions(mex_coefs);
  for (int i = 0; i < num_dims_mex; i++) {
    dims[i] = mxGetDimensions(mex_coefs)[i];
  }
  for (int i = num_dims_mex; i < num_dims; i++) {
    dims[i] = 1;
  }
  vector<double> breaks = matlabToStdVector<double>(mex_breaks);
  std::vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomial_matrices;
  polynomial_matrices.reserve(dims[1]);
  for (mwSize segment_index = 0; segment_index < dims[1]; segment_index++) {
    PiecewisePolynomial<double>::PolynomialMatrix polynomial_matrix(dims[0], 1);
    for (mwSize row = 0; row < dims[0]; row++) {
      VectorXd coefficients(dims[2]);
      for (mwSize coefficient_index = 0; coefficient_index < dims[2]; coefficient_index++) {
        mwSize third_dimension_index = flip_last_dimension ? dims[2] - coefficient_index - 1 : coefficient_index;
        mwSize sub[] = { row, segment_index, third_dimension_index };
        coefficients[coefficient_index] = *(mxGetPr(mex_coefs) + sub2ind(num_dims, dims, sub));
      }
      polynomial_matrix(row) = Polynomial<double>(coefficients);
    }
    polynomial_matrices.push_back(polynomial_matrix);
  }
  PiecewisePolynomial<double> piecewise_polynomial_part = PiecewisePolynomial<double>(polynomial_matrices, breaks);
  return piecewise_polynomial_part;
}

ExponentialPlusPiecewisePolynomial<double> matlabExpPlusPPToExponentialPlusPiecewisePolynomial(const mxArray* array)
{
  auto K = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldOrPropertySafe(array, "K"));
  auto A = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldOrPropertySafe(array, "A"));
  auto alpha = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldOrPropertySafe(array, "alpha"));
  
  const mxArray* mex_breaks = mxGetFieldOrPropertySafe(array, "breaks");
  const mxArray* mex_coefs = mxGetFieldOrPropertySafe(array, "gamma");
  PiecewisePolynomial<double> piecewise_polynomial_part = matlabCoefsAndBreaksToPiecewisePolynomial(mex_coefs, mex_breaks, false);
  return ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, piecewise_polynomial_part);
}

ExponentialPlusPiecewisePolynomial<double> matlabExpPlusPPOrPPTrajectoryOrVectorToExponentialPlusPiecewisePolynomial(const mxArray* array)
{
  bool is_exp_plus_pp = mxGetProperty(array, 0, "K") != nullptr;
  if (is_exp_plus_pp) {
    return matlabExpPlusPPToExponentialPlusPiecewisePolynomial(array);
  }
  else {
    return matlabPPTrajectoryOrMatrixToPiecewisePolynomial(array);
  }
}

std::vector<RigidBodySupportState> setUpSupports(const mxArray* mex_supports)
{
  std::vector<RigidBodySupportState> ret;
  int num_supports = mxGetNumberOfElements(mex_supports);
  ret.reserve(num_supports);
  for (int support_num = 0; support_num < num_supports; support_num++) {
    RigidBodySupportState support_state;
    auto body_ids = matlabToStdVector<int>(mxGetFieldOrPropertySafe(mex_supports, support_num, "bodies"));
    support_state.reserve(body_ids.size());
    for (int i = 0; i < body_ids.size(); ++i) {
      RigidBodySupportStateElement support_state_element;
      support_state_element.body = body_ids[i] - 1; // base 1 to base zero
      support_state_element.contact_points = matlabToEigenMap<3, Dynamic>(mxGetCell(mxGetFieldOrPropertySafe(mex_supports, support_num, "contact_pts"), i));
      support_state_element.support_surface = matlabToEigenMap<4, 1>(mxGetCell(mxGetFieldOrPropertySafe(mex_supports, support_num, "support_surface"), i));
      support_state.push_back(support_state_element);
    }
    ret.push_back(support_state);
  }
  return ret;
}

std::vector<QPLocomotionPlanSettings::ContactNameToContactPointsMap> setUpContactGroups(RigidBodyManipulator* robot, const mxArray* mex_contact_groups)
{
  assert(mxGetNumberOfElements(mex_contact_groups) == robot->bodies.size());
  std::vector<QPLocomotionPlanSettings::ContactNameToContactPointsMap> contact_groups;
  contact_groups.reserve(robot->bodies.size());
  for (int body_id = 0; body_id < robot->bodies.size(); body_id++) {
    const mxArray* mex_contact_group = mxGetCell(mex_contact_groups, body_id);
    QPLocomotionPlanSettings::ContactNameToContactPointsMap contact_group;
    for (int field_number = 0; field_number < mxGetNumberOfFields(mex_contact_group); field_number++) {
      contact_group[mxGetFieldNameByNumber(mex_contact_group, field_number)] = matlabToEigenMap<3, Dynamic>(mxGetFieldByNumber(mex_contact_group, 0, field_number));
    }
    contact_groups.push_back(contact_group);
  }
  return contact_groups;
}

std::vector<BodyMotionData> setUpBodyMotions(const mxArray* mex_body_motions)
{
  vector<BodyMotionData> ret;
  int num_body_motions = mxGetNumberOfElements(mex_body_motions);
  ret.resize(num_body_motions);
  for (int i = 0; i < num_body_motions; ++i) {
    BodyMotionData& body_motion_data = ret[i];
    body_motion_data.body_or_frame_id = static_cast<int>(mxGetPrSafe(mxGetPropertySafe(mex_body_motions, i, "body_id"))[0]) - 1; // base 1 to base 0
    body_motion_data.trajectory = matlabCoefsAndBreaksToPiecewisePolynomial(mxGetPropertySafe(mex_body_motions, i, "coefs"), mxGetPropertySafe(mex_body_motions, i, "ts"), true);
    body_motion_data.toe_off_allowed = matlabToStdVector<bool>(mxGetPropertySafe(mex_body_motions, i, "toe_off_allowed"));
    body_motion_data.in_floating_base_nullspace = matlabToStdVector<bool>(mxGetPropertySafe(mex_body_motions, i, "in_floating_base_nullspace"));
    body_motion_data.control_pose_when_in_contact = matlabToStdVector<bool>(mxGetPropertySafe(mex_body_motions, i, "control_pose_when_in_contact"));
    auto quat_task_to_world = matlabToEigenMap<4, 1>(mxGetPropertySafe(mex_body_motions, i, "quat_task_to_world"));
    body_motion_data.transform_task_to_world.linear() = quat2rotmat(quat_task_to_world);
    body_motion_data.transform_task_to_world.translation() = matlabToEigenMap<3, 1>(mxGetPropertySafe(mex_body_motions, i, "translation_task_to_world"));
    body_motion_data.xyz_proportional_gain_multiplier = matlabToEigenMap<3, 1>(mxGetPropertySafe(mex_body_motions, i, "xyz_kp_multiplier"));
    body_motion_data.xyz_damping_ratio_multiplier = matlabToEigenMap<3, 1>(mxGetPropertySafe(mex_body_motions, i, "xyz_damping_ratio_multiplier"));
    body_motion_data.exponential_map_proportional_gain_multiplier = mxGetPrSafe(mxGetPropertySafe(mex_body_motions, i, "expmap_kp_multiplier"))[0];
    body_motion_data.exponential_map_damping_ratio_multiplier = mxGetPrSafe(mxGetPropertySafe(mex_body_motions, i, "expmap_damping_ratio_multiplier"))[0];
    body_motion_data.weight_multiplier = matlabToEigenMap<6, 1>(mxGetPropertySafe(mex_body_motions, i, "weight_multiplier"));
  }
  return ret;
}

TVLQRData setUpZMPData(const mxArray* mex_zmp_data)
{
  const int NUM_STATES = 4;
  const int NUM_INPUTS = 2;
  const int NUM_OUTPUTS = 2;

  TVLQRData ret;
  ret.A = matlabToEigenMap<NUM_STATES, NUM_STATES>(mxGetFieldSafe(mex_zmp_data, "A"));
  ret.B = matlabToEigenMap<NUM_STATES, NUM_INPUTS>(mxGetFieldSafe(mex_zmp_data, "B"));
  ret.C = matlabToEigenMap<NUM_OUTPUTS, NUM_STATES>(mxGetFieldSafe(mex_zmp_data, "C"));
  ret.D = matlabToEigenMap<NUM_OUTPUTS, NUM_INPUTS>(mxGetFieldSafe(mex_zmp_data, "D"));
  ret.u0 = matlabToEigenMap<NUM_INPUTS, 1>(mxGetFieldSafe(mex_zmp_data, "u0"));
  ret.R = matlabToEigenMap<NUM_INPUTS, NUM_INPUTS>(mxGetFieldSafe(mex_zmp_data, "R"));
  ret.Qy = matlabToEigenMap<NUM_OUTPUTS, NUM_OUTPUTS>(mxGetFieldSafe(mex_zmp_data, "Qy"));
  ret.Q1 = ret.C.transpose() * ret.Qy * ret.C;
  ret.R1 = ret.R + ret.D.transpose() * ret.Qy * ret.D;
  ret.N = ret.C.transpose() * ret.Qy * ret.D;
  return ret;
}

QuadraticLyapunovFunction setUpLyapunovFunction(const mxArray* mex_V)
{
  auto S = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldSafe(mex_V, "S"));
  auto s1 = matlabExpPlusPPOrPPTrajectoryOrVectorToExponentialPlusPiecewisePolynomial(mxGetFieldSafe(mex_V, "s1"));
  QuadraticLyapunovFunction V(S, s1);
  return V;
}

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
  settings.duration = mxGetScalar(mxGetPropertySafe(mex_settings, "duration"));
  settings.supports = setUpSupports(mxGetPropertySafe(mex_settings, "supports"));
  settings.support_times = matlabToStdVector<double>(mxGetPropertySafe(mex_settings, "support_times"));
  settings.contact_groups = setUpContactGroups(robot, mxGetPropertySafe(mex_settings, "contact_groups"));
  settings.planned_support_command = matlabToStdVector<bool>(mxGetPropertySafe(mex_settings, "planned_support_command"));
  settings.early_contact_allowed_fraction = mxGetScalar(mxGetPropertySafe(mex_settings, "early_contact_allowed_fraction"));
  settings.body_motions = setUpBodyMotions(mxGetPropertySafe(mex_settings, "body_motions"));
  settings.zmp_trajectory = matlabPPTrajectoryOrMatrixToPiecewisePolynomial(mxGetPropertySafe(mex_settings, "zmptraj"));
  settings.zmp_data = setUpZMPData(mxGetPropertySafe(mex_settings, "zmp_data"));
  settings.D_control = matlabToEigenMap<2, 2>(mxGetPropertySafe(mex_settings, "D_control"));
  settings.V = setUpLyapunovFunction(mxGetPropertySafe(mex_settings, "V"));
  settings.q_traj = matlabPPTrajectoryOrMatrixToPiecewisePolynomial(mxGetPropertySafe(mex_settings, "qtraj"));
  settings.com_traj = matlabExpPlusPPOrPPTrajectoryOrVectorToExponentialPlusPiecewisePolynomial(mxGetPropertySafe(mex_settings, "comtraj"));
  settings.gain_set = mxGetStdString(mxGetPropertySafe(mex_settings, "gain_set"));
  settings.mu = mxGetScalar(mxGetPropertySafe(mex_settings, "mu"));
  settings.use_plan_shift = static_cast<bool>(mxGetScalar(mxGetPropertySafe(mex_settings, "use_plan_shift")));
  settings.plan_shift_body_motion_indices = matlabToStdVector<Eigen::DenseIndex>(mxGetPropertySafe(mex_settings, "plan_shift_body_motion_inds"));
  addOffset(settings.plan_shift_body_motion_indices, (Eigen::DenseIndex) -1); // base 1 to base 0
  settings.g = mxGetScalar(mxGetPropertySafe(mex_settings, "g"));
  settings.is_quasistatic = mxGetLogicals(mxGetPropertySafe(mex_settings, "is_quasistatic"))[0];
  settings.knee_settings.min_knee_angle = mxGetScalar(mxGetPropertySafe(mex_settings, "min_knee_angle"));
  settings.ankle_limits_tolerance = mxGetScalar(mxGetPropertySafe(mex_settings, "ankle_limits_tolerance"));
  settings.knee_settings.knee_kp = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_kp"));
  settings.knee_settings.knee_kd = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_kd"));
  settings.knee_settings.knee_weight = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_weight"));
  settings.zmp_safety_margin = mxGetScalar(mxGetPropertySafe(mex_settings, "zmp_safety_margin"));
  settings.pelvis_name = mxGetStdString(mxGetPropertySafe(mex_settings, "pelvis_name"));
  settings.foot_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "l_foot_name"));
  settings.foot_names[Side::RIGHT] = mxGetStdString(mxGetPropertySafe(mex_settings, "r_foot_name"));
  settings.knee_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "l_knee_name"));
  settings.knee_names[Side::RIGHT] = mxGetStdString(mxGetPropertySafe(mex_settings, "r_knee_name"));
  settings.akx_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "l_akx_name"));
  settings.akx_names[Side::RIGHT] = mxGetStdString(mxGetPropertySafe(mex_settings, "r_akx_name"));
  settings.aky_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "l_aky_name"));
  settings.aky_names[Side::RIGHT] = mxGetStdString(mxGetPropertySafe(mex_settings, "r_aky_name"));
  settings.constrained_position_indices = matlabToStdVector<int>(mxGetPropertySafe(mex_settings, "constrained_dofs"));
  addOffset(settings.constrained_position_indices, -1); // base 1 to base 0
  settings.untracked_position_indices = matlabToStdVector<int>(mxGetPropertySafe(mex_settings, "untracked_joint_inds"));
  addOffset(settings.untracked_position_indices, -1); // base 1 to base 0

  // lcm
  string lcm_channel = mxGetStdString(mex_lcm_channel);

  QPLocomotionPlan* plan = new QPLocomotionPlan(*robot, settings, lcm_channel);

  plhs[0] = createDrakeMexPointer((void*) plan, "QPLocomotionPlan");

  return;
}
