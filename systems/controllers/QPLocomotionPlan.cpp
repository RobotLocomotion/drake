#include "QPLocomotionPlan.h"
#include "drakeFloatingPointUtil.h"
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <limits>
#include "drakeGeometryUtil.h"
#include "splineGeneration.h"
#include "drakeUtil.h"
#include "lcmUtil.h"
#include <string>

// TODO: go through everything and make it match the updated Matlab code
// TODO: discuss possibility of chatter in knee control
// TODO: make body_motions a map from RigidBody* to BodyMotionData, remove body_id from BodyMotionData?
// TODO: undo plan shift?

using namespace std;
using namespace Eigen;

QPLocomotionPlan::QPLocomotionPlan(RigidBodyManipulator& robot, const QPLocomotionPlanSettings& settings, const std::string& lcm_channel) :
    robot(robot),
    settings(settings),
    start_time(std::numeric_limits<double>::quiet_NaN()),
    pelvis_id(robot.findLinkId(settings.pelvis_name)),
    foot_body_ids(createFootBodyIdMap(robot, settings.foot_names)),
    knee_indices(createKneeIndicesMap(robot, foot_body_ids)),
    plan_shift(Eigen::Vector3d::Zero())
{
  for (int i = 1; i < settings.support_times.size(); i++) {
    if (settings.support_times[i] < settings.support_times[i - 1])
      throw std::runtime_error("support times must be increasing");
  }

  for (auto it = Side::values.begin(); it != Side::values.end(); ++it) {
    toe_off_active[*it] = false;
  }
}


// TODO: delete?
template <size_t Rows, size_t Cols, typename Derived>
void polynomialVectorCoefficientsToCArrayOfArraysMatlabOrdering(const Eigen::MatrixBase<Derived>& source, typename Derived::Scalar::CoefficientType (&destination)[Rows][Cols]) {
  assert(source.cols() == 1);
  if (Rows != source.rows())
    throw std::runtime_error("Number of rows of source doesn't match destination");

  typedef typename Derived::Scalar PolynomialType;
  typedef typename PolynomialType::CoefficientType CoefficientType;
  typedef typename PolynomialType::CoefficientsType CoefficientsType;

  for (size_t row = 0; row < Rows; ++row) {
    const PolynomialType& polynomial = source(row);
    const CoefficientsType& coefficients = polynomial.getCoefficients();
    for (size_t col = 0; col < Cols; ++col) {
      size_t coefficient_index = Cols - col - 1;
      if (coefficient_index < coefficients.size())
        destination[row][col] = coefficients[coefficient_index];
      else
        destination[row][col] = (CoefficientType) 0;
    }
  }
}

const std::map<SupportLogicType, std::vector<bool> > QPLocomotionPlan::support_logic_maps = QPLocomotionPlan::createSupportLogicMaps();

template <typename DerivedQ, typename DerivedV>
void QPLocomotionPlan::publishQPControllerInput(
    double t_global, const Eigen::MatrixBase<DerivedQ>& q, const Eigen::MatrixBase<DerivedV>& v, const std::vector<bool>& contact_force_detected)
{
  if (isNaN(start_time))
    start_time = t_global;

  double t_plan = t_global - start_time;
  if (t_plan < 0)
    throw std::runtime_error("t_plan is negative!"); // TODO: decide how to handle this case; fail fast for now
  t_plan = std::min(t_plan, settings.duration);

  // find index into supports vector
  size_t support_index = 0;
  while (support_index < settings.support_times.size() - 1 && t_plan >= settings.support_times[support_index + 1])
    support_index++;

  RigidBodySupportState& support_state = settings.supports[support_index];
  bool is_last_support = support_index == settings.supports.size() - 1;
  const RigidBodySupportState& next_support = is_last_support ? settings.supports[support_index] : settings.supports[support_index + 1];
  updatePlanShift(t_plan, contact_force_detected, next_support);

  drake::lcmt_qp_controller_input qp_input;
  qp_input.be_silent = false;
  qp_input.timestamp = 0;
  qp_input.num_support_data = 0;
  qp_input.num_tracked_bodies = 0;
  qp_input.num_external_wrenches = 0;
  qp_input.num_joint_pd_overrides = 0;

  // zmp data: D
  Matrix2d D = -settings.lipm_height * Matrix2d::Identity();
  eigenToCArrayOfArrays(D, qp_input.zmp_data.D);

  // whole body data
  auto q_des = settings.q_traj.value(t_plan);
  eigenVectorToStdVector(q_des, qp_input.whole_body_data.q_des);
  qp_input.whole_body_data.constrained_dofs = settings.constrained_position_indices;

  // zmp data: x0, y0
  if (settings.is_quasistatic) {
    auto com_des = settings.com_traj.value(t_plan);
    auto comdot_des = settings.com_traj.derivative().value(t_plan);

    size_t x0_row = 0;
    for (DenseIndex i = 0; i < com_des.size(); ++i) {
      qp_input.zmp_data.x0[x0_row++][0] = com_des(i);
    }
    for (DenseIndex i = 0; i < comdot_des.size(); ++i) {
      qp_input.zmp_data.x0[x0_row++][0] = comdot_des(i);
    }
    eigenToCArrayOfArrays(com_des, qp_input.zmp_data.y0);
  }
  else {
    size_t x0_row = 0;
    for (DenseIndex i = 0; i < settings.zmp_final.size(); ++i) {
      qp_input.zmp_data.x0[x0_row++][0] = settings.zmp_final(i);
    }
    for (DenseIndex i = 0; i < settings.zmp_final.size(); ++i) {
      qp_input.zmp_data.x0[x0_row++][0] = 0.0;
    }
    auto zmp_des = settings.zmp_trajectory.value(t_plan);
    eigenToCArrayOfArrays(zmp_des, qp_input.zmp_data.y0);
  }

  // apply zmp plan shift
  for (auto it = settings.plan_shift_zmp_indices.begin(); it != settings.plan_shift_zmp_indices.end(); ++it) {
    qp_input.zmp_data.x0[*it][0] -= plan_shift[*it];
    qp_input.zmp_data.y0[*it][0] -= plan_shift[*it];
  }

  // zmp data: Lyapunov function
  eigenToCArrayOfArrays(settings.V.getS(), qp_input.zmp_data.S);
  auto s1_current = settings.V.getS1().value(t_plan);
  eigenToCArrayOfArrays(s1_current, qp_input.zmp_data.s1);

  VectorXd v_dummy(0, 1);
  robot.doKinematicsNew(q, v_dummy);

  bool pelvis_has_tracking = false;
  for (int j = 0; j < settings.body_motions.size(); ++j) {
    BodyMotionData& body_motion = settings.body_motions[j];
    int body_or_frame_id = body_motion.getBodyOrFrameId();
    int body_id = robot.parseBodyOrFrameID(body_or_frame_id);
    int body_motion_segment_index = body_motion.findSegmentIndex(t_plan);

    auto side_it = foot_body_ids.begin();
    for (; side_it != foot_body_ids.end(); side_it++) {
      if (side_it->second == body_id) {
        break;
      }
    }
    bool is_foot = side_it != foot_body_ids.end();

    if (is_foot) {
      // toe off active switching logic
      Side side = side_it->first;
      int knee_index = knee_indices.at(side);
      if (!toe_off_active[side]) {
        bool is_support_foot = isSupportingBody(body_id, support_state);
        bool knee_close_to_singularity = q[knee_index] < settings.knee_settings.min_knee_angle;
        if (is_support_foot && knee_close_to_singularity) {
          if (is_last_support)
            toe_off_active[side] = false;
          else {
            toe_off_active[side] = !isSupportingBody(body_id, next_support);
          }
        }
      }
      else {
        if (!isSupportingBody(body_id, support_state)) {
          toe_off_active[side] = false;
          updateSwingTrajectory(t_plan, body_motion, body_motion_segment_index - 1, v);
        }
      }

      if (toe_off_active[side]) {
        for (int i = 0; i < support_state.size(); ++i) {
          RigidBodySupportStateElement& support_state_element = support_state[i];
          if (support_state_element.body == body_id)
            support_state_element.contact_points = settings.contact_groups[body_id].at("toe");
        }
        drake::lcmt_joint_pd_override joint_pd_override_for_support;
        joint_pd_override_for_support.position_ind = static_cast<int32_t>(knee_index);
        joint_pd_override_for_support.qi_des = settings.knee_settings.min_knee_angle;
        joint_pd_override_for_support.qdi_des = 0.0;
        joint_pd_override_for_support.kp = settings.knee_settings.knee_kp;
        joint_pd_override_for_support.kd = settings.knee_settings.knee_kd;
        joint_pd_override_for_support.weight = settings.knee_settings.knee_weight;
        qp_input.joint_pd_override.push_back(joint_pd_override_for_support);
        qp_input.num_joint_pd_overrides++;

        if (body_motion.isToeOffAllowed(body_motion_segment_index)) {
          updateSwingTrajectory(t_plan, body_motion, body_motion_segment_index, v);
        }
      }
    }

    // extract out current and next polynomial segments
    PiecewisePolynomial<double> body_motion_trajectory_slice = body_motion.getTrajectory().slice(body_motion_segment_index, 2);

    // convert to global time
    body_motion_trajectory_slice.shiftRight(start_time);

    // apply plan shift
    PiecewisePolynomial<double>::CoefficientMatrix trajectory_shift(body_motion_trajectory_slice.rows(), body_motion_trajectory_slice.cols());
    trajectory_shift.setZero();
    for (auto direction_it = settings.plan_shift_body_motion_indices.begin(); direction_it != settings.plan_shift_body_motion_indices.end(); ++direction_it) {
      trajectory_shift(*direction_it) = plan_shift(*direction_it);
    }
    body_motion_trajectory_slice -= trajectory_shift;

    drake::lcmt_body_motion_data body_motion_data_for_support_lcm;
    body_motion_data_for_support_lcm.body_id = body_id;

    encodePiecewisePolynomial(body_motion_trajectory_slice, body_motion_data_for_support_lcm.spline);
    body_motion_data_for_support_lcm.in_floating_base_nullspace = body_motion.isInFloatingBaseNullSpace(body_motion_segment_index);
    body_motion_data_for_support_lcm.control_pose_when_in_contact = body_motion.isPoseControlledWhenInContact(body_motion_segment_index);
    const Isometry3d& transform_task_to_world = body_motion.getTransformTaskToWorld();
    Vector4d quat_task_to_world = rotmat2quat(transform_task_to_world.linear());
    Vector3d translation_task_to_world = transform_task_to_world.translation();
    eigenVectorToCArray(quat_task_to_world, body_motion_data_for_support_lcm.quat_task_to_world);
    eigenVectorToCArray(translation_task_to_world, body_motion_data_for_support_lcm.translation_task_to_world);
    eigenVectorToCArray(body_motion.getXyzProportionalGainMultiplier(), body_motion_data_for_support_lcm.xyz_kp_multiplier);
    eigenVectorToCArray(body_motion.getXyzDampingRatioMultiplier(), body_motion_data_for_support_lcm.xyz_damping_ratio_multiplier);
    body_motion_data_for_support_lcm.expmap_kp_multiplier = body_motion.getExponentialMapProportionalGainMultiplier();
    body_motion_data_for_support_lcm.expmap_damping_ratio_multiplier = body_motion.getExponentialMapDampingRatioMultiplier();
    eigenVectorToCArray(body_motion.getWeightMultiplier(), body_motion_data_for_support_lcm.weight_multiplier);

    qp_input.body_motion_data.push_back(body_motion_data_for_support_lcm);
    qp_input.num_tracked_bodies++;

    if (body_id == pelvis_id)
      pelvis_has_tracking = true;
  }

  if (!pelvis_has_tracking)
    throw runtime_error("Expecting a motion_motion_data element for the pelvis");

  // set support data
  for (auto it = support_state.begin(); it != support_state.end(); ++it) {
    drake::lcmt_support_data support_data_element_lcm;
    support_data_element_lcm.timestamp = 0;
    support_data_element_lcm.body_id = static_cast<int32_t>(it->body);
    support_data_element_lcm.num_contact_pts = it->contact_points.cols();
    eigenToStdVectorOfStdVectors(it->contact_points, support_data_element_lcm.contact_pts);
    for (int i = 0; i < planned_support_command.size(); i++)
      support_data_element_lcm.support_logic_map[i] = planned_support_command[i];
    support_data_element_lcm.mu = settings.mu;
    support_data_element_lcm.contact_surfaces = it->contact_surface;
    qp_input.support_data.push_back(support_data_element_lcm);
    qp_input.num_support_data++;
  }

  qp_input.param_set_name = settings.gain_set;

  qp_input.joint_pd_override = joint_pd_override_data;
  last_qp_input = qp_input;

  verifySubtypeSizes(qp_input);
  lcm.publish(lcm_channel, &qp_input);
}

void QPLocomotionPlan::updateSwingTrajectory(double t_plan, BodyMotionData& body_motion_data, int body_motion_segment_index, const Eigen::VectorXd& qd) {
  int takeoff_segment_index = body_motion_segment_index + 1; // this function is called before takeoff
  int num_swing_segments = 3;
  int landing_segment_index = takeoff_segment_index + num_swing_segments - 1;

  // last three knot points from spline
  PiecewisePolynomial<double>& trajectory = body_motion_data.getTrajectory();
  VectorXd x1 = trajectory.value(trajectory.getEndTime(takeoff_segment_index));
  VectorXd x2 = trajectory.value(trajectory.getEndTime(takeoff_segment_index + 1));
  VectorXd xf = trajectory.value(trajectory.getEndTime(takeoff_segment_index + 2));

  // first knot point from current position
  auto x0_xyzquat = robot.forwardKinNew((Vector3d::Zero()).eval(), body_motion_data.getBodyOrFrameId(), 0, 2, 1);
  auto& J = x0_xyzquat.gradient().value();
  auto xd0_xyzquat = (J * qd).eval();
  Vector4d x0_quat = x0_xyzquat.value().tail<4>(); // copying to Vector4d for quatRotateVec later on.
  auto x0_expmap = quat2expmap(x0_quat, 1);
  Vector3d xd0_expmap = x0_expmap.gradient().value() * xd0_xyzquat.tail<4>();

  auto x0_expmap_unwrapped = unwrapExpmap(x1.tail<3>(), x0_expmap.value(), 1);
  typedef Matrix<double, 6, 1> Vector6d;
  Vector6d x0;
  x0.head<3>() = x0_xyzquat.value().head<3>();
  x0.tail<3>() = x0_expmap_unwrapped.value().tail<3>();

  Vector6d xd0;
  xd0.head<3>().setZero();
  xd0.tail<3>() = x0_expmap_unwrapped.gradient().value() * xd0_expmap;

  // If the current pose is pitched down more than the first aerial knot point, adjust the knot point to match the current pose
  Vector3d unit_x = Vector3d::UnitX();
  auto quat1_gradientvar = expmap2quat(x1.tail<3>(), 0);
  Vector3d unit_x_rotated_0 = quatRotateVec(x0_quat, unit_x);
  Vector3d unit_x_rotated_1 = quatRotateVec(quat1_gradientvar.value(), unit_x);
  if (unit_x_rotated_0(2) < unit_x_rotated_1(2)) {
    auto quat2_gradientvar = expmap2quat(x2.tail<3>(), 0);
    x1.tail<3>() = quat2expmap(slerp(x0_quat, quat2_gradientvar.value(), 0.1), 0).value();
  }

  // FIXME: way too expensive
  MatrixXd xdf = trajectory.derivative().value(trajectory.getEndTime(landing_segment_index));

  auto start_it = trajectory.getSegmentTimes().begin() + takeoff_segment_index;
  int num_breaks = num_swing_segments + 1;
  std::vector<double> breaks(start_it, start_it + num_breaks);

  for (int dof_num = 0; dof_num < x0.rows(); ++dof_num) {
    PiecewisePolynomial<double> updated_spline_for_dof = twoWaypointCubicSpline(breaks, x0(dof_num), xd0(dof_num), xf(dof_num), xdf(dof_num), x1(dof_num), x2(dof_num));
    for (int j = 0; j < updated_spline_for_dof.getNumberOfSegments(); j++) {
      body_motion_data.getTrajectory().setPolynomialMatrixBlock(updated_spline_for_dof.getPolynomialMatrix(j), takeoff_segment_index + j, dof_num, 0);
    }
  }
  // NOTE: not updating times here. If we use a spline generation method that also determines the times, remember to update those as well
}


bool QPLocomotionPlan::isSupportingBody(int body_index, const RigidBodySupportState& support_state) const {
  for (auto it = support_state.begin(); it != support_state.end(); ++it) {
    if (it->body == body_index)
      return true;
  }
  return false;
}

template <typename T>
class AddressEqual
{
private:
  const T& t;
public:
  AddressEqual(const T& t) : t(t) {}
  bool operator()(const T& other) const { return &t == &other; }
};

void QPLocomotionPlan::updatePlanShift(double t_plan, const std::vector<bool>& contact_force_detected, const RigidBodySupportState& next_support) {
  // determine indices corresponding to support feet in next support
  std::vector<int> support_foot_indices;
  for (auto it = next_support.begin(); it != next_support.end(); ++it) {
    RigidBodySupportStateElement support_state_element = *it;
    int support_body_id = support_state_element.body;
    bool is_foot = false;
    for (auto side_it = foot_body_ids.begin(); side_it != foot_body_ids.end(); side_it++) {
      if (side_it->second == support_body_id) {
        is_foot = true;
      }
    }

    bool is_loading = contact_force_detected[support_body_id];
    if (is_foot && is_loading) {
      support_foot_indices.push_back(support_body_id);
    }
  }

  // find where the next support is in our planned list of supports
  vector<RigidBodySupportState>::const_iterator next_support_it = std::find_if(settings.supports.begin(), settings.supports.end(), AddressEqual<const RigidBodySupportState&>(next_support));

  // reverse iterate to find which one came into contact last
  reverse_iterator<vector<RigidBodySupportState>::const_iterator> supports_reverse_it(next_support_it);
  for (; supports_reverse_it != settings.supports.rend(); ++supports_reverse_it) {
    const RigidBodySupportState& support = *supports_reverse_it;
    for (auto support_foot_it = support_foot_indices.begin(); support_foot_it != support_foot_indices.end(); ++support_foot_it) {
      if (!isSupportingBody(*support_foot_it, support)) {
        int support_foot_that_came_into_contact_last = *support_foot_it;

        // find corresponding body_motion and compute new plan shift
        for (auto body_motion_it = settings.body_motions.begin(); body_motion_it != settings.body_motions.end(); ++body_motion_it) {
          int body_motion_body_id = robot.parseBodyOrFrameID(body_motion_it->getBodyOrFrameId());
          if (body_motion_body_id == support_foot_that_came_into_contact_last) {
            int world = 0;
            int rotation_type = 0;
            Vector3d foot_frame_origin_actual = robot.forwardKinNew(Vector3d::Zero().eval(), body_motion_it->getBodyOrFrameId(), world, rotation_type, 0).value();
            Vector3d foot_frame_origin_planned = body_motion_it->getTrajectory().value(t_plan).topRows<3>();

            /*
             * If more than one supporting foot came into contact at the same time (e.g. when jumping and landing with two feet at the same time)
             * just use the foot that appears first in support_foot_indices
             */
            plan_shift = foot_frame_origin_planned - foot_frame_origin_actual;
          }
        }
      }
    }
  }
}

const std::map<SupportLogicType, std::vector<bool> > QPLocomotionPlan::createSupportLogicMaps()
{
  std::map<SupportLogicType, std::vector<bool> > ret;
  ret[REQUIRE_SUPPORT] = { {true, true, true, true} };
  ret[ONLY_IF_FORCE_SENSED] = { {false, false, true, true} };
  ret[KINEMATIC_OR_SENSED] = { {false, true, true, true} };
  ret[PREVENT_SUPPORT] = { {false, false, false, false} };
  return ret;
}

const std::map<Side, int> QPLocomotionPlan::createFootBodyIdMap(RigidBodyManipulator& robot, const std::map<Side, std::string>& foot_names)
{
  std::map<Side, int> foot_body_ids;
  for (auto it = Side::values.begin(); it != Side::values.end(); ++it) {
    foot_body_ids[*it] = robot.findLinkId(foot_names.at(*it));
  }
  return foot_body_ids;
}

const std::map<Side, int> QPLocomotionPlan::createKneeIndicesMap(RigidBodyManipulator& robot, const std::map<Side, int>& foot_body_ids)
{
  std::map<Side, int> knee_indices;
  for (auto it = Side::values.begin(); it != Side::values.end(); ++it) {
    int foot_body_id = foot_body_ids.at(*it);
    knee_indices[*it] = robot.bodies[foot_body_id]->position_num_start;
  }
  return knee_indices;
}

template void QPLocomotionPlan::publishQPControllerInput<Eigen::Matrix<double, -1, 1, 0, -1, 1>, Eigen::Matrix<double, -1, 1, 0, -1, 1> >(double, Eigen::MatrixBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, 1, 0, -1, 1> > const&, std::__1::vector<bool, std::__1::allocator<bool> > const&);
template void QPLocomotionPlan::publishQPControllerInput<Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> >, Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> > >(double, MatrixBase<Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> > > const&, MatrixBase<Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> > > const&, vector<bool, allocator<bool> > const&);
