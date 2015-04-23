#ifndef SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_
#define SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_

#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "drake/PiecewisePolynomial.h"
#include "drake/RigidBodyManipulator.h"
#include "lcmtypes/drake/lcmt_qp_controller_input.hpp"
#include "QPCommon.h"
#include "BodyMotionData.h"
#include "Side.h"

class QPLocomotionPlan
{
private:
  RigidBodyManipulator* robot;
  std::vector<double> support_times;
  std::vector<RigidBodySupportState> supports;
  std::vector<BodyMotionData> body_motions;
  PiecewisePolynomial<double> zmp_trajectory;
  Eigen::Vector2d zmp_final;
  double lipm_height;
  QuadraticLyapunovFunction V;
  PiecewisePolynomial<double> q_traj;
  PiecewisePolynomial<double> com_traj;
  double mu;
  Eigen::Isometry3d plan_shift;
  PlanShiftMode plan_shift_mode;
  double g;
  bool is_quasistatic;
  Eigen::Matrix<bool, Eigen::Dynamic, 1> position_constrained_indices;
  drake::lcmt_qp_controller_input last_qp_input;
  drake::lcmt_joint_pd_override joint_pd_override_data;
  std::map<Side, bool> toe_off_active;




public:
  virtual ~QPLocomotionPlan();
};

#endif /* SYSTEMS_ROBOTINTERFACES_QPLOCOMOTIONPLAN_H_ */
