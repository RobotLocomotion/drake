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
  drake::lcmt_joint_pd_override joint_pd_override_data;
  std::map<Side, bool> toe_off_active;

  const KneeSettings knee_settings;

public:
  void createQPControllerInput(
      double t_global, const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const RobotPropertyCache& robot_property_cache, const std::vector<bool>& contact_force_detected);

  virtual ~QPLocomotionPlan();

private:
  bool isSupportingBody(int body_index, const RigidBodySupportState& support_state);
};

#endif /* SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_ */
