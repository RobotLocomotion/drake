#pragma once

#include "humanoid_state.h"
#include <iostream>
#include <fstream>

/**
 * Input to the QP inverse dynamics controller
 */
class QPInput {
 public:
  std::vector<std::string> joint_names;

  Vector3d comdd_d;
  Vector6d pelvdd_d;
  Vector6d torsodd_d;
  Vector6d footdd_d[2];
  VectorXd qdd_d;
  Vector6d wrench_d[2];

  double w_com;
  double w_pelv;
  double w_torso;
  double w_foot;
  double w_hand;
  double w_qdd;
  double w_wrench_reg;

  QPInput() { inited_ = false; }

  explicit QPInput(const RigidBodyTree& r) {
    Init(r);
    qdd_d.resize(r.number_of_velocities());
    inited_ = true;
  }

  bool is_sane() const {
    bool ret = inited_;
    ret &= joint_names.size() == qdd_d.size();
    return ret;
  }

  void Init(const RigidBodyTree& r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      joint_names.push_back(r.getPositionName(i));
    inited_ = true;
  }

 private:
  bool inited_;
};

/**
 * Output of the QP inverse dynamics controller
 */
class QPOutput {
 public:
  std::vector<std::string> joint_names;

  Vector3d comdd;
  Vector6d pelvdd;
  Vector6d torsodd;
  Vector6d footdd[2];
  VectorXd qdd;
  VectorXd trq;

  Vector6d foot_wrench_w[2];
  Vector6d foot_wrench_in_sensor_frame[2];

  QPOutput() { inited_ = false; }

  explicit QPOutput(const RigidBodyTree& r) {
    Init(r);
    inited_ = true;
  }

  void Init(const RigidBodyTree& r) {
    for (int i = 0; i < r.number_of_positions(); i++)
      joint_names.push_back(r.getPositionName(i));
    inited_ = true;
  }

  bool is_sane() const {
    bool ret = inited_;
    ret &= joint_names.size() == qdd.size();
    ret &= (qdd.size() == trq.size()) || (qdd.size() == trq.size() + 6);
    return ret;
  }

  void Print() const;
  double ComputeCost(const HumanoidState& rs, const QPInput& input) const;

 private:
  bool inited_;
};

/**
 * Parameter for the QP inverse dynamics controller.
 */
class QPParam {
 public:
  double mu;     // Friction approx for tangential force. Fx, Fy < |mu * Fz|
  double mu_Mz;  // Friction approx for normal torque. Mz < |mu * Mz|
  double x_max;  // Size of the foot in the x direction, in the foot frame
  double x_min;
  double y_max;  // Size of the foot in the y direction, in the foot frame
  double y_min;

  QPParam() {
    mu = 1;
    mu_Mz = 0.1;
    x_max = 0.2;
    x_min = -0.05;
    y_max = 0.05;
    y_min = -0.05;
  }
};

/**
 * The QP inverse dynamics controller
 */
class QPController {
 public:
  QPParam param;

  /**
   * @brief The current version uses SNOPT, and instantiates
   * an OptimizationProblem every call.
   *
   * @return 0 if success, < if error.
   */
  int Control(const HumanoidState& rs, const QPInput& input, QPOutput& output);
};
