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

// TODO: discuss possibility of chatter in knee control
// TODO: make body_motions a map from RigidBody* to BodyMotionData, remove body_id from BodyMotionData?


using namespace std;
using namespace Eigen;

const std::map<SupportLogicType, std::vector<bool> > QPLocomotionPlan::support_logic_maps = QPLocomotionPlan::createSupportLogicMaps();

QPLocomotionPlan::QPLocomotionPlan(RigidBodyManipulator& robot, const QPLocomotionPlanSettings& settings, const std::string& lcm_channel) :
    robot(robot),
    settings(settings),
    start_time(std::numeric_limits<double>::quiet_NaN()),
    pelvis_id(robot.findLinkId(settings.pelvis_name)),
    foot_body_ids(createFootBodyIdMap(robot, settings.foot_names)),
    knee_indices(createKneeIndicesMap(robot, settings.knee_names)),
    plan_shift(Vector3d::Zero())
{
  for (int i = 1; i < settings.support_times.size(); i++) {
    if (settings.support_times[i] < settings.support_times[i - 1])
      throw std::runtime_error("support times must be increasing");
  }

  for (auto it = Side::values.begin(); it != Side::values.end(); ++it) {
    toe_off_active[*it] = false;
    foot_shifts[*it] = Vector3d::Zero();
  }

  if (!lcm.good())
  {
    cerr << "ERROR: lcm is not good()" << endl;
  }
}

template <typename DerivedQ, typename DerivedV>
drake::lcmt_qp_controller_input QPLocomotionPlan::createQPControllerInput(
    double t_global, const MatrixBase<DerivedQ>& q, const MatrixBase<DerivedV>& v, const std::vector<bool>& contact_force_detected)
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

  // do kinematics
  VectorXd v_dummy(0, 1);
  robot.doKinematicsNew(q, v_dummy);

  RigidBodySupportState& support_state = settings.supports[support_index];
  bool is_last_support = support_index == settings.supports.size() - 1;
  const RigidBodySupportState& next_support = is_last_support ? settings.supports[support_index] : settings.supports[support_index + 1];
  updatePlanShift(t_plan, contact_force_detected, support_index);

  drake::lcmt_qp_controller_input qp_input;
  qp_input.be_silent = false;
  qp_input.timestamp = static_cast<int64_t>(t_global * 1e6);
  qp_input.num_support_data = 0;
  qp_input.num_tracked_bodies = 0;
  qp_input.num_external_wrenches = 0;
  qp_input.num_joint_pd_overrides = 0;

  // whole body data
  auto q_des = settings.q_traj.value(t_plan);
  qp_input.whole_body_data.timestamp = 0;
  qp_input.whole_body_data.num_positions = robot.num_positions;
  eigenVectorToStdVector(q_des, qp_input.whole_body_data.q_des);
  qp_input.whole_body_data.constrained_dofs = settings.constrained_position_indices;
  addOffset(qp_input.whole_body_data.constrained_dofs, 1); // use 1-indexing in LCM
  qp_input.whole_body_data.num_constrained_dofs = qp_input.whole_body_data.constrained_dofs.size();
  // apply plan shift
  for (auto direction_it = settings.plan_shift_body_motion_indices.begin(); direction_it != settings.plan_shift_body_motion_indices.end(); ++direction_it) {
    qp_input.whole_body_data.q_des[*direction_it] -= plan_shift[*direction_it];
  }

  // zmp data:
  qp_input.zmp_data = createZMPData(t_plan);

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
        joint_pd_override_for_support.timestamp = 0;
        joint_pd_override_for_support.position_ind = static_cast<int32_t>(knee_index) + 1; // use 1-indexing in LCM
        joint_pd_override_for_support.qi_des = settings.knee_settings.min_knee_angle;
        joint_pd_override_for_support.qdi_des = 0.0;
        joint_pd_override_for_support.kp = settings.knee_settings.knee_kp;
        joint_pd_override_for_support.kd = settings.knee_settings.knee_kd;
        joint_pd_override_for_support.weight = settings.knee_settings.knee_weight;
        qp_input.joint_pd_override.push_back(joint_pd_override_for_support);
        qp_input.num_joint_pd_overrides++;
      }
      if (body_motion.isToeOffAllowed(body_motion_segment_index)) {
        updateSwingTrajectory(t_plan, body_motion, body_motion_segment_index, v);
      }
    }

    // extract out current and next polynomial segments
    PiecewisePolynomial<double> body_motion_trajectory_slice = body_motion.getTrajectory().slice(body_motion_segment_index, std::min(2, body_motion.getTrajectory().getNumberOfSegments() - body_motion_segment_index));

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
    body_motion_data_for_support_lcm.timestamp = 0;
    body_motion_data_for_support_lcm.body_id = body_or_frame_id + 1; // use 1-indexing in LCM

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

    // If this body is not currently in support, but will be soon (within
    // early_contact_allowed_time), then generate a new support data for that
    // body which will only be active if that body senses force.
    bool last_support = support_index == settings.supports.size() - 1;
    if (!last_support) {
      if (settings.support_times[support_index + 1] - t_plan < settings.early_contact_allowed_time) {
        if (!isSupportingBody(body_id, support_state)) {
          for (auto it = next_support.begin(); it != next_support.end(); ++it) {
            if (it->body == body_id) {
              qp_input.support_data.push_back(createSupportDataElement(*it, support_logic_maps.at(ONLY_IF_FORCE_SENSED)));
              qp_input.num_support_data++;
              break;
            }
          }
        }
      }
    }
  }

  if (!pelvis_has_tracking)
    throw runtime_error("Expecting a motion_motion_data element for the pelvis");

  // set support data
  for (auto it = support_state.begin(); it != support_state.end(); ++it) {
    qp_input.support_data.push_back(createSupportDataElement(*it, settings.planned_support_command));
    qp_input.num_support_data++;
  }

  qp_input.param_set_name = settings.gain_set;

  for (auto it = settings.untracked_position_indices.begin(); it != settings.untracked_position_indices.end(); ++it) {
    drake::lcmt_joint_pd_override joint_pd_override;
    joint_pd_override.timestamp = 0;
    joint_pd_override.position_ind = *it + 1; // use 1-indexing in LCM
    joint_pd_override.qi_des = q[*it];
    joint_pd_override.qdi_des = v[*it];
    joint_pd_override.kp = 0.0;
    joint_pd_override.kd = 0.0;
    joint_pd_override.weight = 0.0;
    qp_input.joint_pd_override.push_back(joint_pd_override);
    qp_input.num_joint_pd_overrides++;

    qp_input.whole_body_data.q_des[*it] = q[*it];
    auto constrained_dofs_it = std::find(qp_input.whole_body_data.constrained_dofs.begin(), qp_input.whole_body_data.constrained_dofs.end(), *it + 1); // use 1-indexing in LCM
    if (constrained_dofs_it != qp_input.whole_body_data.constrained_dofs.end()) {
      qp_input.whole_body_data.constrained_dofs.erase(constrained_dofs_it);
      qp_input.whole_body_data.num_constrained_dofs = qp_input.whole_body_data.constrained_dofs.size();
    }
  }

  last_qp_input = qp_input;
  verifySubtypeSizes(qp_input);
  return qp_input;
}

void QPLocomotionPlan::setDuration(double duration)
{
  settings.duration = duration;
}

void QPLocomotionPlan::setStartTime(double start_time)
{
  this->start_time = start_time;
}

double QPLocomotionPlan::getStartTime() const
{
  return start_time;
}

double QPLocomotionPlan::getDuration() const
{
  return settings.duration;
}

drake::lcmt_qp_controller_input QPLocomotionPlan::getLastQPInput() const
{
  return this->last_qp_input;
}

bool QPLocomotionPlan::isFinished(double t) const
{
  if (isNaN(start_time)) {
    return false;
  } else {
    return (t - start_time) >= settings.duration;
  }
}

const RigidBodyManipulator& QPLocomotionPlan::getRobot() const
{
  return robot;
}

drake::lcmt_zmp_data QPLocomotionPlan::createZMPData(double t_plan) const
{
  drake::lcmt_zmp_data zmp_data_lcm;
  zmp_data_lcm.timestamp = 0;
  eigenToCArrayOfArrays(settings.zmp_data.A, zmp_data_lcm.A);
  eigenToCArrayOfArrays(settings.zmp_data.B, zmp_data_lcm.B);
  eigenToCArrayOfArrays(settings.zmp_data.C, zmp_data_lcm.C);
//  eigenToCArrayOfArrays(settings.zmp_data.D, zmp_data_lcm.D); // set later
//  eigenToCArrayOfArrays(settings.zmp_data.x0, zmp_data_lcm.x0); // set later
//  eigenToCArrayOfArrays(settings.zmp_data.y0, zmp_data_lcm.y0); // set later
  eigenToCArrayOfArrays(settings.zmp_data.u0, zmp_data_lcm.u0);
  eigenToCArrayOfArrays(settings.zmp_data.R, zmp_data_lcm.R);
  eigenToCArrayOfArrays(settings.zmp_data.Qy, zmp_data_lcm.Qy);
//  eigenToCArrayOfArrays(settings.zmp_data.S, zmp_data_lcm.S); // set later
//  eigenToCArrayOfArrays(settings.zmp_data.s1, zmp_data_lcm.s1); // set later
//  eigenToCArrayOfArrays(settings.zmp_data.s1dot, zmp_data_lcm.s1dot); // set later
  zmp_data_lcm.s2 = settings.zmp_data.s2;
  zmp_data_lcm.s2dot = settings.zmp_data.s2dot;

  // D
  Matrix2d D = -settings.lipm_height / settings.g * Matrix2d::Identity();
  eigenToCArrayOfArrays(D, zmp_data_lcm.D);

  // x0, y0
  if (settings.is_quasistatic) {
    auto com_des = settings.com_traj.value(t_plan);
    auto comdot_des = settings.com_traj.derivative().value(t_plan);

    size_t x0_row = 0;
    for (DenseIndex i = 0; i < com_des.size(); ++i) {
      zmp_data_lcm.x0[x0_row++][0] = com_des(i);
    }
    for (DenseIndex i = 0; i < comdot_des.size(); ++i) {
      zmp_data_lcm.x0[x0_row++][0] = comdot_des(i);
    }
    eigenToCArrayOfArrays(com_des, zmp_data_lcm.y0);
  } else {
    size_t x0_row = 0;
    for (DenseIndex i = 0; i < settings.zmp_final.size(); ++i) {
      zmp_data_lcm.x0[x0_row++][0] = settings.zmp_final(i);
    }
    for (DenseIndex i = 0; i < settings.zmp_final.size(); ++i) {
      zmp_data_lcm.x0[x0_row++][0] = 0.0;
    }
    auto zmp_des = settings.zmp_trajectory.value(t_plan);
    eigenToCArrayOfArrays(zmp_des, zmp_data_lcm.y0);
  }

  // apply zmp plan shift
  for (auto it = settings.plan_shift_zmp_indices.begin(); it != settings.plan_shift_zmp_indices.end(); ++it) {
    zmp_data_lcm.x0[*it][0] -= plan_shift[*it];
    zmp_data_lcm.y0[*it][0] -= plan_shift[*it];
  }

  // Lyapunov function
  eigenToCArrayOfArrays(settings.V.getS(), zmp_data_lcm.S);
  auto s1_current = settings.V.getS1().value(t_plan);
  eigenToCArrayOfArrays(s1_current, zmp_data_lcm.s1);
  Vector4d s1dot_current = Vector4d::Zero(); // NOTE: not using this; setting to zeros as was being done in the Matlab implementation
  eigenToCArrayOfArrays(s1dot_current, zmp_data_lcm.s1dot);

  return zmp_data_lcm;
}

drake::lcmt_support_data QPLocomotionPlan::createSupportDataElement(const RigidBodySupportStateElement& element, const std::vector<bool>& support_logic)
{
  drake::lcmt_support_data support_data_element_lcm;
  support_data_element_lcm.timestamp = 0;
  support_data_element_lcm.body_id = static_cast<int32_t>(element.body) + 1; // use 1-indexing in LCM
  support_data_element_lcm.num_contact_pts = element.contact_points.cols();
  eigenToStdVectorOfStdVectors(element.contact_points, support_data_element_lcm.contact_pts);
  for (int i = 0; i < support_logic.size(); i++)
    support_data_element_lcm.support_logic_map[i] = support_logic[i];
  support_data_element_lcm.mu = settings.mu;
  support_data_element_lcm.use_support_surface = element.use_contact_surface;
  Vector4f support_surface_float = element.support_surface.cast<float>();
  memcpy(support_data_element_lcm.support_surface, support_surface_float.data(), sizeof(float) * 4);
  return support_data_element_lcm;
}

void QPLocomotionPlan::updateSwingTrajectory(double t_plan, BodyMotionData& body_motion_data, int body_motion_segment_index, const VectorXd& qd) {
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
  for (auto it = settings.plan_shift_body_motion_indices.begin(); it != settings.plan_shift_body_motion_indices.end(); ++it) {
    x0(*it) += plan_shift(*it);
  }
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

  // TODO: find a less expensive way of doing this
  MatrixXd xdf = trajectory.derivative().value(trajectory.getEndTime(landing_segment_index));

  auto start_it = trajectory.getSegmentTimes().begin() + takeoff_segment_index;
  int num_breaks = num_swing_segments + 1;
  std::vector<double> breaks(start_it, start_it + num_breaks);

  for (int dof_num = 0; dof_num < x0.rows(); ++dof_num) {
    PiecewisePolynomial<double> updated_spline_for_dof = nWaypointCubicSpline(breaks, x0(dof_num), xd0(dof_num), xf(dof_num), xdf(dof_num), Vector2d(x1(dof_num), x2(dof_num)));
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

std::vector<Side> QPLocomotionPlan::getSupportSides(const RigidBodySupportState &support_state) const {
  std::vector<Side> support_sides;
  for (auto side_it = foot_body_ids.begin(); side_it != foot_body_ids.end(); ++side_it) {
    for (auto it = support_state.begin(); it != support_state.end(); ++it) {
      if (it->body == side_it->second) {
        support_sides.push_back(side_it->first);
      }
    }
  }
  return support_sides;
}

void QPLocomotionPlan::updatePlanShift(double t_plan, const std::vector<bool>& contact_force_detected, int support_index) {
  const bool is_last_support = support_index == settings.supports.size() - 1;
  const RigidBodySupportState& next_support = is_last_support ? settings.supports[support_index] : settings.supports[support_index + 1];


  if (t_plan - last_foot_shift_time > settings.min_foot_shift_delay) {
    // First, figure out the relative shifts for each of the feet which are in support and in contact
    for (auto side_it = foot_body_ids.begin(); side_it != foot_body_ids.end(); side_it++) {
      if (QPLocomotionPlan::isSupportingBody(side_it->second, next_support)) {
        if (contact_force_detected[side_it->second]) {
          for (auto body_motion_it = settings.body_motions.begin(); body_motion_it != settings.body_motions.end(); ++body_motion_it) {
            int body_motion_body_id = robot.parseBodyOrFrameID(body_motion_it->getBodyOrFrameId());
            if (body_motion_body_id == side_it->second) {
              int world = 0;
              int rotation_type = 0;
              Vector3d foot_frame_origin_actual = robot.forwardKinNew(Vector3d::Zero().eval(), body_motion_it->getBodyOrFrameId(), world, rotation_type, 0).value();
              Vector3d foot_frame_origin_planned = body_motion_it->getTrajectory().value(t_plan).topRows<3>();
              foot_shifts[side_it->first] = foot_frame_origin_planned - foot_frame_origin_actual;
              break;
            }
          }
        }
        break;
      }
    }
    last_foot_shift_time = t_plan;
  }

  // Now figure out how to combine the foot shifts into a single plan shift. The logic is:
  // If we're in single support, then use the shift for the support foot
  // If we're in double support, then interpolate linearly between the two shifts based on the time since one foot was single support and the time when the other foot will be single support
  std::vector<Side> support_sides = this->getSupportSides(settings.supports[support_index]);
  if (support_sides.size() == 1) {
    // single support
    plan_shift = foot_shifts[support_sides[0]];
  } else {
    // First, look backwards until we find the most recent single support
    bool found_past_single_support = false;
    double t_last_single_support_end;
    Vector3d past_shift;
    for (int i = support_index - 1; i >= 0; --i) {
      support_sides = this->getSupportSides(settings.supports[i]);
      if (support_sides.size() == 1) {
        found_past_single_support = true;
        t_last_single_support_end = settings.support_times[i+1];
        past_shift = foot_shifts[support_sides[0]];
        break;
      }
    }
    if (!found_past_single_support) {
      past_shift = 0.5 * (foot_shifts.at(Side(Side::LEFT)) + foot_shifts.at(Side(Side::RIGHT)));
      t_last_single_support_end = settings.support_times[0];
    }

    // Now look forwards until we find the next single support
    bool found_future_single_support = false;
    double t_next_single_support_begin;
    Vector3d future_shift;
    for (int i = support_index + 1; i < settings.supports.size(); ++i) {
      support_sides = this->getSupportSides(settings.supports[i]);
      if (support_sides.size() == 1) {
        found_future_single_support = true;
        t_next_single_support_begin = settings.support_times[i];
        future_shift = foot_shifts[support_sides[0]];
        break;
      }
    }
    if (!found_future_single_support) {
      future_shift = 0.5 * (foot_shifts[Side(Side::LEFT)] + foot_shifts[Side(Side::RIGHT)]);
      t_next_single_support_begin = settings.support_times[settings.support_times.size() - 1];
    }

    // Now interpolate between the past and future shift vectors using the current plan time
    // std::cout << "t_plan: " << t_plan << " t_last_single_support_end: " << t_last_single_support_end << " t_next_single_support_begin: " << t_next_single_support_begin << std::endl;
    const double fraction = (t_plan - t_last_single_support_end) / (t_next_single_support_begin - t_last_single_support_end);
    plan_shift = fraction * future_shift + (1 - fraction) * past_shift;
    // std::cout << "fraction: " << fraction << " past: " << past_shift.transpose() << " future: " << future_shift.transpose() << std::endl;
  }
  // std::cout << "plan shift: " << plan_shift.transpose() << std::endl;
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

const std::map<Side, int> QPLocomotionPlan::createKneeIndicesMap(RigidBodyManipulator& robot, const std::map<Side, std::string>& knee_names)
{
  std::map<Side, int> knee_indices;
  for (auto it = Side::values.begin(); it != Side::values.end(); ++it) {
    int joint_id = robot.findJointId(knee_names.at(*it));
    knee_indices[*it] = robot.bodies[joint_id]->position_num_start;
  }
  return knee_indices;
}

template drake::lcmt_qp_controller_input QPLocomotionPlan::createQPControllerInput<Matrix<double, -1, 1, 0, -1, 1>, Matrix<double, -1, 1, 0, -1, 1> >(double, MatrixBase<Matrix<double, -1, 1, 0, -1, 1> > const&, MatrixBase<Matrix<double, -1, 1, 0, -1, 1> > const&, std::vector<bool, std::allocator<bool> > const&);
template drake::lcmt_qp_controller_input QPLocomotionPlan::createQPControllerInput<Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> >, Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> > >(double, MatrixBase<Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> > > const&, MatrixBase<Map<Matrix<double, -1, 1, 0, -1, 1> const, 0, Stride<0, 0> > > const&, vector<bool, allocator<bool> > const&);
