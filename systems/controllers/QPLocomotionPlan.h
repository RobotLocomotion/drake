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

enum SupportLogicType {
  REQUIRE_SUPPORT, ONLY_IF_FORCE_SENSED, ONLY_IF_KINEMATIC, KINEMATIC_OR_SENSED, PREVENT_SUPPORT
};

struct KneeSettings {
  double min_knee_angle = 0.7;
  double knee_kp = 40;
  double knee_kd = 4;
  double knee_weight = 1;
};

class QPLocomotionPlan
{
private:
  double duration;
  double start_time;
  drake::lcmt_qp_controller_input default_qp_input;
  std::string gain_set;

  RigidBodyManipulator* robot;
  std::vector<double> support_times;
  std::vector<RigidBodySupportState> supports;
  std::vector<BodyMotionData> body_motions;
  PiecewisePolynomial<double> zmp_trajectory;
  Eigen::Vector2d zmp_final;
  double lipm_height;
  QuadraticLyapunovFunction V;
  PiecewisePolynomial<double> q_traj;
  ExponentialPlusPiecewisePolynomial<double> com_traj;
  double mu;
  Eigen::Isometry3d plan_shift;
  PlanShiftMode plan_shift_mode;
  double g;
  bool is_quasistatic;
  std::vector<int> constrained_position_indices;
  drake::lcmt_qp_controller_input last_qp_input;
  std::vector<drake::lcmt_joint_pd_override> joint_pd_override_data;
  std::map<Side, bool> toe_off_active;

  const KneeSettings knee_settings;

  const static std::map<SupportLogicType, std::vector<bool> > support_logic_maps;

  /*
   * when the plan says a given body is in support, require the controller to use that support.
   * To allow the controller to use that support only if it thinks the body is in contact with the terrain, try KINEMATIC_OR_SENSED
   */
  const std::vector<bool>& planned_support_command = support_logic_maps.at(REQUIRE_SUPPORT);

public:

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
  bool isSupportingBody(int body_index, const RigidBodySupportState& support_state);

  void updateSwingTrajectory(double t_plan, BodyMotionData& body_motion_data, int body_motion_segment_index, const Eigen::VectorXd& qd);

  static const std::map<SupportLogicType, std::vector<bool> > createSupportLogicMaps();
};

#endif /* SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_ */
