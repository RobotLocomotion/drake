#include "drakeUtil.h"
#include "QPLocomotionPlan.h"
#include <sstream>

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

  const mxArray* dim_mex = mxGetFieldSafe(pp, "dim");
  int num_dims_mex = mxGetNumberOfElements(dim_mex);
  if (num_dims_mex == 0 | num_dims_mex > 2)
    throw runtime_error("case not handled"); // because PiecewisePolynomial can't currently handle it
  const int num_dims = 2;
  mwSize dims[num_dims];
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

ExponentialPlusPiecewisePolynomial<double> matlabExpPlusPPToExponentialPlusPiecewisePolynomial(const mxArray* array)
{
  vector<double> breaks = matlabToStdVector<double>(mxGetFieldOrPropertySafe(array, "breaks"));

  auto K = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldOrPropertySafe(array, "K"));
  auto A = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldOrPropertySafe(array, "A"));
  auto alpha = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldOrPropertySafe(array, "alpha"));
  
  const mxArray* gamma_mex = mxGetFieldOrPropertySafe(array, "gamma");
  int num_dims = 3;
  mwSize dims[num_dims];
  size_t num_dims_mex = mxGetNumberOfDimensions(gamma_mex);
  for (int i = 0; i < num_dims_mex; i++) {
    dims[i] = mxGetDimensions(gamma_mex)[i];
  }
  for (int i = num_dims_mex; i < num_dims; i++) {
    dims[i] = 1;
  }

  std::vector<PiecewisePolynomial<double>::PolynomialMatrix> polynomial_matrices;
  polynomial_matrices.reserve(dims[1]);

  for (mwSize segment_index = 0; segment_index < dims[1]; segment_index++) {
    PiecewisePolynomial<double>::PolynomialMatrix polynomial_matrix(dims[0], 1);
    for (mwSize row = 0; row < dims[0]; row++) {
      VectorXd coefficients(dims[2]);
      for (mwSize coefficient_index = 0; coefficient_index < dims[2]; coefficient_index++) {
        mwSize sub[] = { row, segment_index, coefficient_index };
        coefficients[coefficient_index] = *(mxGetPr(gamma_mex) + sub2ind(num_dims, dims, sub));
      }
      polynomial_matrix(row) = Polynomial<double>(coefficients);
    }
    polynomial_matrices.push_back(polynomial_matrix);
  }
  PiecewisePolynomial<double> piecewise_polynomial_part = PiecewisePolynomial<double>(polynomial_matrices, breaks);

  return ExponentialPlusPiecewisePolynomial<double>(K, A, alpha, piecewise_polynomial_part);
}

ExponentialPlusPiecewisePolynomial<double> matlabExpPlusPPOrVectorToExponentialPlusPiecewisePolynomial(const mxArray* array)
{
  if (mxIsNumeric(array)) {
    return ExponentialPlusPiecewisePolynomial<double>(matlabToEigenMap<Dynamic, Dynamic>(array));
  }
  else {
    return matlabExpPlusPPToExponentialPlusPiecewisePolynomial(array);
  }
}

std::vector<QPLocomotionPlanSettings::ContactNameToContactPointsMap> setUpContactGroups(RigidBodyManipulator* robot, const mxArray* mex_settings)
{
  const mxArray* mex_contact_groups = mxGetPropertySafe(mex_settings, "contact_groups");
  assert(mxGetNumberOfElements(mex_contact_groups) == robot->num_bodies);
  std::vector<QPLocomotionPlanSettings::ContactNameToContactPointsMap> contact_groups;
  for (int body_id = 0; body_id < robot->num_bodies; body_id++) {
    const mxArray* mex_contact_group = mxGetCell(mex_contact_groups, body_id);
    QPLocomotionPlanSettings::ContactNameToContactPointsMap contact_group;
    for (int field_number = 0; field_number < mxGetNumberOfFields(mex_contact_group); field_number++) {
      contact_group[mxGetFieldNameByNumber(mex_contact_group, field_number)] = matlabToEigenMap<3, Dynamic>(mxGetFieldByNumber(mex_contact_group, 0, field_number));
    }
    contact_groups.push_back(contact_group);
  }
  return contact_groups;
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
  // TODO:
  // * supports
  // * body motions

  QPLocomotionPlanSettings settings;
  settings.duration = mxGetScalar(mxGetPropertySafe(mex_settings, "duration"));
//  std::vector<RigidBodySupportState> supports;
  settings.support_times = matlabToStdVector<double>(mxGetPropertySafe(mex_settings, "support_times"));
  settings.contact_groups = setUpContactGroups(robot, mex_settings);

//  std::vector<BodyMotionData> body_motions;
  settings.zmp_trajectory = matlabPPTrajectoryOrMatrixToPiecewisePolynomial(mxGetPropertySafe(mex_settings, "zmptraj"));
  settings.zmp_final = matlabToEigen<2, 1>(mxGetPropertySafe(mex_settings, "zmp_final"));
  settings.lipm_height = mxGetScalar(mxGetPropertySafe(mex_settings, "LIP_height"));
  const mxArray* mex_V = mxGetPropertySafe(mex_settings, "V");
  auto S = matlabToEigenMap<Dynamic, Dynamic>(mxGetFieldSafe(mex_V, "S"));
  auto s1 = matlabExpPlusPPOrVectorToExponentialPlusPiecewisePolynomial(mxGetFieldSafe(mex_V, "s1"));
  settings.V = QuadraticLyapunovFunction(S, s1);
  settings.q_traj = matlabPPTrajectoryOrMatrixToPiecewisePolynomial(mxGetPropertySafe(mex_settings, "qtraj"));
  settings.com_traj = matlabExpPlusPPOrVectorToExponentialPlusPiecewisePolynomial(mxGetPropertySafe(mex_settings, "comtraj"));
  settings.gain_set = mxGetStdString(mxGetPropertySafe(mex_settings, "gain_set"));
  settings.mu = mxGetScalar(mxGetPropertySafe(mex_settings, "mu"));
  settings.plan_shift_zmp_indices = matlabToStdVector<Eigen::DenseIndex>(mxGetPropertySafe(mex_settings, "plan_shift_zmp_inds")); // TODO enable after making QPLocomotionPlan changes
  settings.plan_shift_body_motion_indices = matlabToStdVector<Eigen::DenseIndex>(mxGetPropertySafe(mex_settings, "plan_shift_body_motion_inds"));  // TODO enable after making QPLocomotionPlan changes
  settings.g = mxGetScalar(mxGetPropertySafe(mex_settings, "g"));
  settings.is_quasistatic = mxGetLogicals(mxGetPropertySafe(mex_settings, "is_quasistatic"))[0];
  settings.knee_settings.min_knee_angle = mxGetScalar(mxGetPropertySafe(mex_settings, "min_knee_angle"));
  settings.knee_settings.knee_kp = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_kp"));
  settings.knee_settings.knee_kd = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_kd"));
  settings.knee_settings.knee_weight = mxGetScalar(mxGetPropertySafe(mex_settings, "knee_weight"));
  settings.pelvis_name = mxGetStdString(mxGetPropertySafe(mex_settings, "pelvis_name"));
  settings.foot_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "l_foot_name"));
  settings.foot_names[Side::LEFT] = mxGetStdString(mxGetPropertySafe(mex_settings, "r_foot_name"));
  settings.constrained_position_indices = matlabToStdVector<int>(mxGetPropertySafe(mex_settings, "constrained_dofs"));

  // lcm
  string lcm_channel = mxGetStdString(mex_lcm_channel);

  QPLocomotionPlan* plan = new QPLocomotionPlan(*robot, settings, lcm_channel);

  plhs[0] = createDrakeMexPointer((void*) plan, "QPLocomotionPlan");

  return;
}
