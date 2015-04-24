#include "QPLocomotionPlan.h"
#include "drakeFloatingPointUtil.h"
#include <stdexcept>
#include <algorithm>

// TODO: default start_time to nan
// TODO: set default qp input
// TODO: com traj 2x1, differentiate
// TODO: check that support_times are in order
// TODO: don't forget about foot_body_ids
// TODO: timestamp? be_silent? num variables?
// TODO: discuss possibility of chatter in knee control
// TODO: drakePiecewisePolynomial_EXPORTS

using namespace std;
using namespace Eigen;

void QPLocomotionPlan::createQPControllerInput(
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

  bool pelvis_has_tracking = false;

  std::map<int, Side> foot_body_id_to_side;
  foot_body_id_to_side[robot_property_cache.body_ids.l_foot] = Side::LEFT;
  foot_body_id_to_side[robot_property_cache.body_ids.r_foot] = Side::RIGHT;

  std::map<Side, int> knee_indices;
  knee_indices[Side::LEFT] = robot_property_cache.position_indices.l_leg_kny[0];
  knee_indices[Side::RIGHT] = robot_property_cache.position_indices.r_leg_kny[0];

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
      RigidBodySupportState& support_state = supports[support_index];
      if (toe_off_active[side]) {
        bool is_support_foot = isSupportingBody(body_id, support_state);
        bool knee_close_to_singularity = q[knee_index] < knee_settings.min_knee_angle;
        if (is_support_foot && knee_close_to_singularity) {
          bool is_last_support = support_index == supports.size() - 1;
          if (is_last_support)
            toe_off_active[side] = false;
          else {
            const RigidBodySupportState& next_support = supports[support_index + 1];
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


//    body_motion.setTrajectory()
//        qp_input.body_motion_data(j).ts = qp_input.body_motion_data(j).ts + obj.start_time_;
//
//        if qp_input.body_motion_data(j).body_id == rpc.body_ids.pelvis
//          pelvis_has_tracking = true;
//        end
//


  }

//
//      for j = 1:length(obj.body_motions)
//
//      end
//
//      assert(pelvis_has_tracking, 'Expecting a motion_motion_data element for the pelvis');
//
//      supp = obj.supports(supp_idx);
//
//      qp_input.support_data = struct('body_id', cell(1, length(supp.bodies)),...
//        'contact_pts', cell(1, length(supp.bodies)),...
//        'support_logic_map', cell(1, length(supp.bodies)),...
//        'mu', cell(1, length(supp.bodies)),...
//        'contact_surfaces', cell(1, length(supp.bodies)));
//      for j = 1:length(supp.bodies)
//        qp_input.support_data(j).body_id = supp.bodies(j);
//        qp_input.support_data(j).contact_pts = supp.contact_pts{j};
//        qp_input.support_data(j).support_logic_map = obj.planned_support_command;
//        qp_input.support_data(j).mu = obj.mu;
//        qp_input.support_data(j).contact_surfaces = 0;
//      end
//
//      qp_input.param_set_name = obj.gain_set_;
//
//      if supp_idx < length(obj.supports)
//        next_support = obj.supports(supp_idx + 1);
//      else
//        next_support = obj.supports(supp_idx);
//      end
//      obj = obj.updatePlanShift(t_global, kinsol, qp_input, contact_force_detected, next_support);
//      qp_input = obj.applyPlanShift(qp_input);
//
//      if(~isempty(obj.joint_pd_override_data))
//        for j = 1:length(obj.joint_pd_override_data.position_ind)
//          qp_input.joint_pd_override(end+1) = struct('position_ind',obj.joint_pd_override_data.position_ind(j),...
//            'qi_des',qp_input.whole_body_data.q_des(obj.joint_pd_override_data.position_ind(j)),...
//            'qdi_des',0,...
//            'kp',obj.joint_pd_override_data.kp(j),...
//            'kd',obj.joint_pd_override_data.kd(j),...
//            'weight',obj.joint_pd_override_data.weight(j));
//        end
//      end
//
//      obj.last_qp_input = qp_input;
}


QPLocomotionPlan::~QPLocomotionPlan()
{
  // TODO Auto-generated destructor stub
}

bool QPLocomotionPlan::isSupportingBody(int body_index, const RigidBodySupportState& support_state) {
  for (auto it = support_state.begin(); it != support_state.end(); ++it) {
    if (it->body == body_index)
      return true;
  }
  return false;
}
