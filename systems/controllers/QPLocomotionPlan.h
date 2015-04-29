#ifndef SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_
#define SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_

#include <vector>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "drake/PiecewisePolynomial.h"
#include "drake/RigidBodyManipulator.h"
#include "drake/lcmt_qp_controller_input.hpp"
#include "QPCommon.h"
#include "BodyMotionData.h"
#include "Side.h"
#include <lcm/lcm-cpp.hpp>

enum SupportLogicType {
  REQUIRE_SUPPORT, ONLY_IF_FORCE_SENSED, ONLY_IF_KINEMATIC, KINEMATIC_OR_SENSED, PREVENT_SUPPORT
};

struct KneeSettings {
  double min_knee_angle;
  double knee_kp;
  double knee_kd;
  double knee_weight;
};

struct QPLocomotionPlanSettings {
  double duration;
  std::vector<double> support_times;
  std::vector<RigidBodySupportState> supports;
  std::vector<BodyMotionData> body_motions;
  PiecewisePolynomial<double> zmp_trajectory;
  Eigen::Vector2d zmp_final;
  double lipm_height;
  QuadraticLyapunovFunction V;
  PiecewisePolynomial<double> q_traj;
  ExponentialPlusPiecewisePolynomial<double> com_traj;
  drake::lcmt_qp_controller_input default_qp_input;

  std::string gain_set = "standing";
  double mu = 0.5;
  std::vector<Eigen::DenseIndex> plan_shift_zmp_indices = { { 1, 2 } };
  std::vector<Eigen::DenseIndex> plan_shift_body_motion_indices  = { 3 };
  double g = 9.81;
  bool is_quasistatic = false;
  const KneeSettings knee_settings = createDefaultKneeSettings();
  std::map<Side, std::string> foot_names = createDefaultFootNames();
  std::vector<std::string> constrained_joint_name_parts = createDefaultConstrainedJointNameParts();

  static KneeSettings createDefaultKneeSettings() {
    KneeSettings knee_settings;
    knee_settings.min_knee_angle = 0.7;
    knee_settings.knee_kp = 40.0;
    knee_settings.knee_kd = 4.0;
    knee_settings.knee_weight = 1.0;
    return knee_settings;
  }

  static std::map<Side, std::string> createDefaultFootNames() {
    std::map<Side, std::string> ret;
    ret[Side::LEFT] = "l_foot";
    ret[Side::RIGHT] = "r_foot";
    return ret;
  }

  static std::vector<std::string> createDefaultConstrainedJointNameParts() {
    std::vector<std::string> ret;
    ret.push_back("arm");
    ret.push_back("neck");
    ret.push_back("back_bkz");
    ret.push_back("back_bky");
    return ret;
  }
};

class QPLocomotionPlan
{
private:
  RigidBodyManipulator& robot; // TODO: const correctness
  QPLocomotionPlanSettings settings;
  const std::map<Side, int> foot_body_ids;
  const std::map<Side, int> knee_indices;

  lcm::LCM lcm;
  std::string lcm_channel;

  double start_time;
  Eigen::Vector3d plan_shift;
  std::vector<int> constrained_position_indices;
  drake::lcmt_qp_controller_input last_qp_input;
  std::vector<drake::lcmt_joint_pd_override> joint_pd_override_data;
  std::map<Side, bool> toe_off_active;

  /*
   * when the plan says a given body is in support, require the controller to use that support.
   * To allow the controller to use that support only if it thinks the body is in contact with the terrain, try KINEMATIC_OR_SENSED
   */
  const static std::map<SupportLogicType, std::vector<bool> > support_logic_maps;
  const std::vector<bool>& planned_support_command = support_logic_maps.at(REQUIRE_SUPPORT);

public:
QPLocomotionPlan(RigidBodyManipulator& robot, const QPLocomotionPlanSettings& settings, const std::string& lcm_channel);

  /*
   * Get the input structure which can be passed to the stateless QP control loop
   * @param t the current time
   * @param x the current robot state
   * @param rpc the robot property cache, which lets us quickly look up info about
   * @param contact_force_detected num_bodies vector indicating whether contact force
   * was detected on that body. Default: zeros(num_bodies,1)
   * the robot which would be expensive to compute (such as terrain contact points)
   */
  void publishQPControllerInput(
      double t_global, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const RobotPropertyCache& robot_property_cache, const std::vector<bool>& contact_force_detected);

private:
  bool isSupportingBody(int body_index, const RigidBodySupportState& support_state) const;

  void updateSwingTrajectory(double t_plan, BodyMotionData& body_motion_data, int body_motion_segment_index, const Eigen::VectorXd& qd);

  void updatePlanShift(double t_global, const std::vector<bool>& contact_force_detected, const RigidBodySupportState& next_support);

  static const std::map<SupportLogicType, std::vector<bool> > createSupportLogicMaps();

  static const std::map<Side, int> createFootBodyIdMap(RigidBodyManipulator& robot, const std::map<Side, std::string>& foot_names);

  static const std::map<Side, int> createKneeIndicesMap(RigidBodyManipulator& robot, const std::map<Side, int>& foot_body_ids);
};

#endif /* SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_ */
