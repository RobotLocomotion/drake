#include "QPLocomotionPlan.h"
#include "drakeFloatingPointUtil.h"
#include <stdexcept>
#include <algorithm>
#include "drakeGeometryUtil.h"

// TODO: default start_time to nan
// TODO: set default qp input
// TODO: com traj 2x1, differentiate
// TODO: check that support_times are in order
// TODO: don't forget about foot_body_ids
// TODO: timestamp? be_silent? num variables?
// TODO: discuss possibility of chatter in knee control
// TODO: drakePiecewisePolynomial_EXPORTS
// TODO: constructor:   planned_support_command = support_logic_maps[REQUIRE_SUPPORT];
// TODO: constructor: joint_pd_override_data

using namespace std;
using namespace Eigen;


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

void QPLocomotionPlan::publishQPControllerInput(
    double t_global, const Eigen::VectorXd& q, const VectorXd& v,
    const RobotPropertyCache& robot_property_cache, const std::vector<bool>& contact_force_detected)
{
//  Get the input structure which can be passed to the stateless QP control loop
//  @param t the current time
//  @param x the current robot state
//  @param rpc the robot property cache, which lets us quickly look up info about
//  @param contact_force_detected num_bodies vector indicating whether contact force
//  was detected on that body. Default: zeros(num_bodies,1)
//  the robot which would be expensive to compute (such as terrain contact points)

  if (isNaN(start_time))
    start_time = t_global;

  double t_plan = t_global - start_time;
  if (t_plan < 0)
    throw std::runtime_error("t_plan is negative!"); // TODO: decide how to handle this case; fail fast for now
  t_plan = std::min(t_plan, duration);

  // zmp data: D
  drake::lcmt_qp_controller_input qp_input = default_qp_input;
  Matrix2d D = -lipm_height * Matrix2d::Identity();
  eigenToCArrayOfArrays(D, qp_input.zmp_data.D);

  // whole body data
  auto q_des = q_traj.value(t_plan);
  eigenVectorToStdVector(q_des, qp_input.whole_body_data.q_des);
  qp_input.whole_body_data.constrained_dofs = constrained_position_indices;

  // zmp data: x0, y0
  if (is_quasistatic) {
    auto com_des = com_traj.value(t_plan);
    auto comdot_des = com_traj.derivative().value(t_plan);

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
    for (DenseIndex i = 0; i < zmp_final.size(); ++i) {
      qp_input.zmp_data.x0[x0_row++][0] = zmp_final(i);
    }
    for (DenseIndex i = 0; i < zmp_final.size(); ++i) {
      qp_input.zmp_data.x0[x0_row++][0] = 0.0;
    }
    auto zmp_des = zmp_trajectory.value(t_plan);
    eigenToCArrayOfArrays(zmp_des, qp_input.zmp_data.y0);
  }

  // zmp data: Lyapunov function
  eigenToCArrayOfArrays(V.S, qp_input.zmp_data.S);
  auto s1_current = V.s1.value(t_plan);
  eigenToCArrayOfArrays(s1_current, qp_input.zmp_data.s1);

  robot->doKinematicsNew(q, v);

  // TODO: something smarter than this linear search
  size_t support_index = 0;
  while (support_index < support_times.size() - 1 && t_plan >= support_times[support_index + 1])
    support_index++;

  std::map<int, Side> foot_body_id_to_side;
  foot_body_id_to_side[robot_property_cache.body_ids.l_foot] = Side::LEFT;
  foot_body_id_to_side[robot_property_cache.body_ids.r_foot] = Side::RIGHT;

  std::map<Side, int> knee_indices;
  knee_indices[Side::LEFT] = robot_property_cache.position_indices.l_leg_kny[0];
  knee_indices[Side::RIGHT] = robot_property_cache.position_indices.r_leg_kny[0];

  RigidBodySupportState& support_state = supports[support_index];
  bool is_last_support = support_index == supports.size() - 1;
  const RigidBodySupportState& next_support = is_last_support ? supports[support_index] : supports[support_index + 1];

  bool pelvis_has_tracking = false;
  for (int j = 0; j < body_motions.size(); ++j) {
    BodyMotionData& body_motion = body_motions[j];
    int body_or_frame_id = body_motion.getBodyOrFrameId();
    int body_id = robot->parseBodyOrFrameID(body_or_frame_id);
    int body_motion_segment_index = body_motion.findSegmentIndex(t_plan);

    auto it = foot_body_id_to_side.find(body_id);
    bool is_foot = it != foot_body_id_to_side.end();
    if (is_foot) {
      // toe off active switching logic
      Side side = it->second;
      int knee_index = knee_indices[side];
      if (toe_off_active[side]) {
        bool is_support_foot = isSupportingBody(body_id, support_state);
        bool knee_close_to_singularity = q[knee_index] < knee_settings.min_knee_angle;
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
          // TODO:
//          updateSwingTrajectory(t_plan, support_state, );
                //              obj = obj.updateSwingTrajectory(t_plan, j, body_t_ind-1, kinsol, qd);
        }
      }

      if (toe_off_active[side]) {
        for (int i = 0; i < support_state.size(); ++i) {
          RigidBodySupportStateElement& support_state_element = support_state[i];
          if (support_state_element.body == body_id)
            support_state_element.contact_points = robot_property_cache.contact_groups[body_id].at("toe");
        }
        drake::lcmt_joint_pd_override joint_pd_override_for_support;
        joint_pd_override_for_support.position_ind = static_cast<int32_t>(knee_index);
        joint_pd_override_for_support.qi_des = knee_settings.min_knee_angle;
        joint_pd_override_for_support.qdi_des = 0.0;
        joint_pd_override_for_support.kp = knee_settings.knee_kp;
        joint_pd_override_for_support.kd = knee_settings.knee_kd;
        joint_pd_override_for_support.weight = knee_settings.knee_weight;
        qp_input.joint_pd_override.push_back(joint_pd_override_for_support);
        qp_input.num_joint_pd_overrides++;

        if (body_motion.isToeOffAllowed(body_motion_segment_index)) {
          // TODO:
//          updateSwingTrajectory();
//              obj = obj.updateSwingTrajectory(t_plan, j, body_t_ind, kinsol, qd);

        }
      }
    }

    // TODO:
//        qp_input.body_motion_data(j) = obj.body_motions(j).slice(body_t_ind);
    const PiecewisePolynomial<> body_motion_trajectory = body_motion.getTrajectory();
    drake::lcmt_body_motion_data body_motion_data_for_support_lcm;
    body_motion_data_for_support_lcm.body_id = body_id;
    body_motion_data_for_support_lcm.ts[0] = body_motion_trajectory.getStartTime(body_motion_segment_index);
    body_motion_data_for_support_lcm.ts[1] = body_motion_trajectory.getEndTime(body_motion_segment_index);
    const PiecewisePolynomial<>::PolynomialMatrix& bodyMotionSegmentIndex = body_motion_trajectory.getPolynomialMatrix(body_motion_segment_index);
    polynomialVectorCoefficientsToCArrayOfArraysMatlabOrdering(bodyMotionSegmentIndex, body_motion_data_for_support_lcm.coefs);
    body_motion_data_for_support_lcm.in_floating_base_nullspace = body_motion.isInFloatingBaseNullSpace(body_motion_segment_index);
    body_motion_data_for_support_lcm.control_pose_when_in_contact = body_motion.isPoseControlledWhenInContact(body_motion_segment_index);
    const Isometry3d& transform_task_to_world = body_motion.getTransformTaskToWorld();
    Vector4d quat_task_to_world = rotmat2quat(transform_task_to_world.linear());
    Vector3d translation_task_to_world = transform_task_to_world.translation();

//    memcpy()

//        'quat_task_to_world',obj.quat_task_to_world,...
//        'translation_task_to_world',obj.translation_task_to_world,...
//        'xyz_kp_multiplier',obj.xyz_kp_multiplier,...
//        'xyz_damping_ratio_multiplier',obj.xyz_damping_ratio_multiplier,...
//        'expmap_kp_multiplier',obj.expmap_kp_multiplier,...
//        'expmap_damping_ratio_multiplier',obj.expmap_damping_ratio_multiplier,...
//        'weight_multiplier',obj.weight_multiplier);


    qp_input.body_motion_data.push_back(body_motion_data_for_support_lcm);


    body_motion.getTrajectory().shiftRight(start_time);
    if (body_id == robot_property_cache.body_ids.pelvis)
      pelvis_has_tracking = true;
  }

  if (!pelvis_has_tracking)
    throw runtime_error("Expecting a motion_motion_data element for the pelvis");

  for (auto it = support_state.begin(); it != support_state.end(); ++it) {
    drake::lcmt_support_data support_data_element_lcm;
    support_data_element_lcm.body_id = static_cast<int32_t>(it->body);
    eigenToStdVectorOfStdVectors(it->contact_points, support_data_element_lcm.contact_pts);
    for (int i = 0; i < planned_support_command.size(); i++)
      support_data_element_lcm.support_logic_map[i] = planned_support_command[i];
    support_data_element_lcm.mu = mu;
    support_data_element_lcm.contact_surfaces = 0;
    qp_input.support_data.push_back(support_data_element_lcm);
  }

  qp_input.param_set_name = gain_set;

  // TODO
//  updatePlanShift(t_global, qp_input, contact_force_detected, next_support);
//      obj = obj.updatePlanShift(t_global, kinsol, qp_input, contact_force_detected, next_support);

  // TODO
//  applyPlanShift(qp_input);
//      qp_input = obj.applyPlanShift(qp_input);

  qp_input.joint_pd_override = joint_pd_override_data;
  last_qp_input = qp_input;

  // TODO: publish LCM message
}

bool QPLocomotionPlan::isSupportingBody(int body_index, const RigidBodySupportState& support_state) {
  for (auto it = support_state.begin(); it != support_state.end(); ++it) {
    if (it->body == body_index)
      return true;
  }
  return false;
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

