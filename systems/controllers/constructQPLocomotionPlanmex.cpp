#include "drakeUtil.h"
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
  settings.support_times = matlabToStdVector<double>(mxGetPropertySafe(mex_settings, "support_times"));
//  typedef std::map<std::string, Eigen::Matrix3Xd> ContactGroupNameToContactPointsMap;
//  std::vector<ContactGroupNameToContactPointsMap> contact_groups; // one for each support

//  std::vector<BodyMotionData> body_motions;
  settings.zmp_trajectory = matlabPPFormToPiecewisePolynomial(mxGetPropertySafe(mxGetPropertySafe(mex_settings, "zmptraj"), "pp"));
  settings.zmp_final = matlabToEigen<2, 1>(mxGetPropertySafe(mex_settings, "zmp_final"));
  settings.lipm_height = mxGetScalar(mxGetPropertySafe(mex_settings, "LIP_height"));

//  QuadraticLyapunovFunction V;
//  PiecewisePolynomial<double> q_traj;
//  settings.com_traj = matlabPPFormToPiecewisePolynomial(mxGetProperty(mex_settings, "comtraj");
//  drake::lcmt_qp_controller_input default_qp_input;

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
