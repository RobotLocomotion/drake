#pragma once

#include "humanoid_status.h"
#include <iostream>
#include <fstream>

/**
 * Input to the QP inverse dynamics controller
 */
struct QPInput {
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names;

  // Desired task space accelerations for various body parts.
  // Postfix _d indicates desired values.
  Vector3d comdd_d;
  Vector6d pelvdd_d;
  Vector6d torsodd_d;
  Vector6d footdd_d[2];

  VectorXd vd_d;         ///< Desired generalized coordinate accelerations
  Vector6d wrench_d[2];  ///< Desired contact wrench

  // These are weights for each cost term.
  // Prefix w_ indicates weights.
  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_vd;
  double w_wrench_reg;
};

void InitQPInput(const RigidBodyTree& r, QPInput* input);
inline bool is_qp_input_sane(const QPInput& input) {
  return ((int)input.coord_names.size() == input.vd_d.size()) &&
         (input.coord_names.size() != 0);
}

/**
 * Output of the QP inverse dynamics controller
 */
struct QPOutput {
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names;

  // Computed task space accelerations of various body parts.
  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d torsodd;
  Vector6d footdd[2];

  VectorXd vd;            ///< Computed generalized coordinate accelerations
  VectorXd joint_torque;  ///< Computed joint torque

  /// Computed contact wrench in the world frame
  Vector6d foot_wrench_in_world_frame[2];
  /// Computed contact wrench transformed to the sensor frame
  Vector6d foot_wrench_in_sensor_frame[2];
};

void InitQPOutput(const RigidBodyTree& r, QPOutput* output);
void PrintQPOutput(const QPOutput& output);
double ComputeQPCost(const HumanoidStatus& rs, const QPInput& input,
                     const QPOutput& output);
inline bool is_qp_output_sane(const QPOutput& output) {
  bool ret = (int)output.coord_names.size() == output.vd.size();
  ret &= output.vd.size() == output.joint_torque.size() + 6;
  return ret;
}

/**
 * Parameter for the QP inverse dynamics controller.
 */
struct QPParam {
  double mu;     // Friction approx for tangential force. Fx, Fy < |mu * Fz|
  double mu_Mz;  // Friction approx for normal torque. Mz < |mu * Mz|
  double x_max;  // Size of the foot in the x direction, in the foot frame
  double x_min;
  double y_max;  // Size of the foot in the y direction, in the foot frame
  double y_min;
};

/**
 * The QP inverse dynamics controller
 */
class QPController {
 public:
  QPParam param;

  /**
   * The current version explicitly uses SNOPT, but it is really solving a
   * quadratic program. It also instantiates
   * an MathematicalProgram every call. It should call a QP solver and maintain
   * a persistent "workspace" for the optimization problem in the future.
   *
   * @return 0 if success, < if error.
   */
  int Control(const HumanoidStatus& rs, const QPInput& input, QPOutput* output);

 private:
  // These are temporary matrices used by the controller.
  Eigen::MatrixXd torque_linear_;
  Eigen::VectorXd torque_constant_;
  Eigen::MatrixXd dynamics_linear_;
  Eigen::VectorXd dynamics_constant_;

  Eigen::MatrixXd inequality_linear_;
  Eigen::VectorXd inequality_upper_bound_;
  Eigen::VectorXd inequality_lower_bound_;
};
