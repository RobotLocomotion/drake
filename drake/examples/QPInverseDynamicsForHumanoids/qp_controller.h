#pragma once

#include "humanoid_status.h"
#include <iostream>
#include <fstream>

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

/**
 * This is used to compute target task space acceleration, which is the input
 * to the inverse dynamics controller. The target acceleration is computed by:
 * xdd_d = Kp*(x* - x) + Kd*(xd* - xd) + xdd*.
 * * are the set points. Kp, Kd are position and velocity gains.
 * First 3 are angular acceleration, and last 3 are linear acceleration.
 */
class CartesianSetPoint {
 public:
  CartesianSetPoint() {
    pose_d_.setIdentity();
    vel_d_.setZero();
    acc_d_.setZero();
    Kp_.setZero();
    Kd_.setZero();
  }

  CartesianSetPoint(const Isometry3d& p_d, const Vector6d& v_d,
                    const Vector6d& vd_d, const Vector6d& Kp,
                    const Vector6d& Kd) {
    pose_d_ = p_d;
    vel_d_ = v_d;
    acc_d_ = vd_d;
    Kp_ = Kp;
    Kd_ = Kd;
  }

  /**
   * Computes target acceleration using PD feedback + feedfoward acceleration.
   * xdd_d = Kp*(x* - x) + Kd*(xd* - xd) + xdd*.
   * First 3 are angular acceleration, and last 3 are linear acceleration.
   */
  Vector6d ComputeAccelerationTarget(const Isometry3d& pose,
                                     const Vector6d& vel) const {
    // feedforward acc + velocity feedback
    Vector6d qdd = acc_d_;
    qdd += (Kd_.array() * (vel_d_ - vel).array()).matrix();

    // pose feedback
    Matrix3d R_err = pose_d_.linear() * pose.linear().transpose();
    AngleAxisd angle_axis_err(R_err);

    Vector3d pos_err = pose_d_.translation() - pose.translation();
    Vector3d rot_err = angle_axis_err.axis() * angle_axis_err.angle();

    // orientation
    qdd.segment<3>(0) += (Kp_.segment<3>(0).array() * rot_err.array()).matrix();

    // position
    qdd.segment<3>(3) += (Kp_.segment<3>(3).array() * pos_err.array()).matrix();

    return qdd;
  }

 private:
  Isometry3d pose_d_;  ///< Desired pose
  Vector6d vel_d_;     ///< Desired velocity
  Vector6d acc_d_;     ///< Desired acceleration

  Vector6d Kp_;  ///< Position gains
  Vector6d Kd_;  ///< Velocity gains
};

/**
 * This class describes contact related information for each contact link.
 */
class SupportElement {
 private:
  const RigidBody& body_;  ///< Link in contact
  // Offsets of the contact point specified in the link's body frame.
  std::vector<Eigen::Vector3d> contact_points_;
  // This is also in the body frame.
  // TODO(siyuan.feng@tri.global): normal is currently assumed to be the same
  // for all the contact points.
  Eigen::Vector3d normal_;
  double mu_;  ///< Friction coeff

 public:
  explicit SupportElement(const RigidBody& b) : body_(b) {
    normal_ = Vector3d(0, 0, 1);
    mu_ = 1;
  }

  /**
   * Computes a matrix (Basis) that converts a vector of scalars (Beta) to
   * stacked point forces (F).
   * All the point forces are in the world frame, and are applied at
   * contact_points_.
   * Basis also approximates all the friction cones at contact_points_.
   * Basis is 3 * contact_points_.size() by num_basis_per_contact_pt  *
   * contact_points_.size()
   * @param robot model
   * @param cache that stores the kinematics information, needs to be
   * initialized first.
   * @param num_basis_per_contact_pt number of basis per contact point
   * @return Basis matrix
   */
  Eigen::MatrixXd ComputeBasisMatrix(const RigidBodyTree& robot,
                                     const KinematicsCache<double>& cache,
                                     int num_basis_per_contact_pt) const {
    if (num_basis_per_contact_pt < 3)
      throw std::runtime_error(
          "Number of basis per contact point must be >= 3.");

    Eigen::MatrixXd basis(3 * contact_points_.size(),
                          num_basis_per_contact_pt * contact_points_.size());
    Eigen::Matrix3d body_rot =
        robot.relativeTransform(cache, 0, body_.get_body_index()).linear();
    basis.setZero();

    Eigen::Vector3d t1, t2, tangent_vec, base;
    double theta;

    // Computes the tangent vectors that are perpendicular to normal_
    if (fabs(1 - normal_[2]) < EPSILON) {
      t1 << 1, 0, 0;
    }
    // same for the reflected case
    else if (fabs(1 + normal_[2]) < EPSILON) {
      t1 << -1, 0, 0;
    } else {
      t1 << normal_[1], -normal_[0], 0;
      t1 /= sqrt(normal_[1] * normal_[1] + normal_[0] * normal_[0]);
    }
    t2 = t1.cross(normal_);

    for (size_t i = 0; i < contact_points_.size(); i++) {
      for (int k = 0; k < num_basis_per_contact_pt; k++) {
        theta = k * 2 * M_PI / num_basis_per_contact_pt;
        tangent_vec = cos(theta) * t1 + sin(theta) * t2;
        base = (normal_ + mu_ * tangent_vec).normalized();
        // rotate basis into world frame
        basis.block(3 * i, num_basis_per_contact_pt * i + k, 3, 1) =
            body_rot * base;
      }
    }
    return basis;
  }

  /**
   * Computes the stacked task space Jacobian matrix at each contact point
   * @param robot model
   * @param cache stores the kinematics information, needs to be initialized
   * first.
   * @return stacked Jacobian matrix
   */
  Eigen::MatrixXd ComputeJacobianAtContactPoints(
      const RigidBodyTree& robot, const KinematicsCache<double>& cache) const {
    Eigen::MatrixXd J(3 * contact_points_.size(), robot.number_of_velocities());
    for (size_t i = 0; i < contact_points_.size(); i++) {
      J.block(3 * i, 0, 3, robot.number_of_velocities()) =
          GetTaskSpaceJacobian(robot, cache, body_, contact_points_[i])
              .bottomRows(3);
    }
    return J;
  }

  /**
   * Computes the stacked task space Jacobian dot times v vector at each contact
   * point
   * @param robot model
   * @param cache stores the kinematics information, needs to be initialized
   * first.
   * @return stacked Jacobian dot times v vector
   */
  Eigen::VectorXd ComputeJacobianDotTimesVAtContactPoints(
      const RigidBodyTree& robot, const KinematicsCache<double>& cache) const {
    Eigen::VectorXd Jdv(3 * contact_points_.size());
    for (size_t i = 0; i < contact_points_.size(); i++) {
      Jdv.segment<3>(3 * i) =
          GetTaskSpaceJacobianDotTimesV(robot, cache, body_, contact_points_[i])
              .bottomRows(3);
    }
    return Jdv;
  }

  inline double mu() const { return mu_; }
  inline const std::vector<Eigen::Vector3d>& contact_points() const {
    return contact_points_;
  }
  inline const Eigen::Vector3d& normal() const { return normal_; }
  inline const RigidBody& body() const { return body_; }

  inline std::vector<Eigen::Vector3d>& get_mutable_contact_points() {
    return contact_points_;
  }
  inline void set_mu(double m) { mu_ = m; }
  inline void set_normal(const Eigen::Vector3d& n) { normal_ = n.normalized(); }
};

/**
 * Input to the QP inverse dynamics controller
 */
class QPInput {
 private:
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names_;
  // Support information
  std::vector<SupportElement> supports_;

  // Desired task space accelerations for various body parts.
  // Postfix _d indicates desired values.
  Vector3d comdd_d_;
  Vector6d pelvdd_d_;
  Vector6d torsodd_d_;
  Vector6d footdd_d_[2];
  VectorXd vd_d_;  ///< Desired generalized coordinate accelerations

  // These are weights for each cost term.
  // Prefix w_ indicates weights.
  double w_com_;
  double w_pelv_;
  double w_torso_;
  double w_foot_;
  double w_vd_;
  double w_basis_reg_;

 public:
  explicit QPInput(const RigidBodyTree& r) {
    coord_names_.resize(r.number_of_velocities());
    for (int i = 0; i < r.number_of_velocities(); i++) {
      // strip out the "dot" part from name
      coord_names_[i] =
          r.getVelocityName(i).substr(0, r.getVelocityName(i).size() - 3);
    }
    vd_d_.resize(r.number_of_velocities());
  }

  inline bool is_valid() const {
    return ((int)coord_names_.size() == vd_d_.size()) &&
           (coord_names_.size() != 0);
  }

  // Getters
  inline double w_com() const { return w_com_; }
  inline double w_pelv() const { return w_pelv_; }
  inline double w_torso() const { return w_torso_; }
  inline double w_foot() const { return w_foot_; }
  inline double w_vd() const { return w_vd_; }
  inline double w_basis_reg() const { return w_basis_reg_; }

  inline const std::string& coord_name(int idx) const {
    return coord_names_.at(idx);
  }
  inline const SupportElement& support(int idx) const {
    return supports_.at(idx);
  }
  inline const std::vector<SupportElement>& supports() const {
    return supports_;
  }
  inline const Vector3d& comdd_d() const { return comdd_d_; }
  inline const Vector6d& pelvdd_d() const { return pelvdd_d_; }
  inline const Vector6d& torsodd_d() const { return torsodd_d_; }
  inline const Vector6d& footdd_d(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return footdd_d_[0];
    else
      return footdd_d_[1];
  }
  inline const Vector6d& footdd_d(int s) const {
    return footdd_d(Side::values.at(s));
  }
  inline const VectorXd& vd_d() const { return vd_d_; }

  // Setters
  inline double& mutable_w_com() { return w_com_; }
  inline double& mutable_w_pelv() { return w_pelv_; }
  inline double& mutable_w_torso() { return w_torso_; }
  inline double& mutable_w_foot() { return w_foot_; }
  inline double& mutable_w_vd() { return w_vd_; }
  inline double& mutable_w_basis_reg() { return w_basis_reg_; }

  inline std::vector<SupportElement>& mutable_supports() { return supports_; }
  inline SupportElement& mutable_support(int idx) { return supports_.at(idx); }
  inline Vector3d& mutable_comdd_d() { return comdd_d_; }
  inline Vector6d& mutable_pelvdd_d() { return pelvdd_d_; }
  inline Vector6d& mutable_torsodd_d() { return torsodd_d_; }
  inline Vector6d& mutable_footdd_d(Side::SideEnum s) {
    if (s == Side::LEFT)
      return footdd_d_[0];
    else
      return footdd_d_[1];
  }
  inline Vector6d& mutable_footdd_d(int s) {
    return mutable_footdd_d(Side::values.at(s));
  }
  inline VectorXd& mutable_vd_d() { return vd_d_; }
};
std::ostream& operator<<(std::ostream& out, const QPInput& input);

/**
 * Output of the QP inverse dynamics controller
 */
class QPOutput {
 private:
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names_;

  // Computed task space accelerations of various body parts.
  Vector3d comdd_;
  Vector6d pelvdd_;
  Vector6d torsodd_;
  Vector6d footdd_[2];

  VectorXd vd_;            ///< Computed generalized coordinate accelerations
  VectorXd joint_torque_;  ///< Computed joint torque

  // Computed contact wrench in the world frame
  Vector6d foot_wrench_in_world_frame_[2];
  // Computed contact wrench transformed to the sensor frame
  Vector6d foot_wrench_in_sensor_frame_[2];

  // Pair of cost term and cost (only the quadratic and linear part)
  std::vector<std::pair<std::string, double>> costs_;

 public:
  explicit QPOutput(const RigidBodyTree& r) {
    coord_names_.resize(r.number_of_velocities());
    for (int i = 0; i < r.number_of_velocities(); i++) {
      // strip out the "dot" part from name
      coord_names_[i] =
          r.getVelocityName(i).substr(0, r.getVelocityName(i).size() - 3);
    }
    vd_.resize(r.number_of_velocities());
    joint_torque_.resize(r.actuators.size());
  }

  inline bool is_valid() const {
    bool ret = (int)coord_names_.size() == vd_.size();
    ret &= vd_.size() == joint_torque_.size() + 6;
    return ret;
  }

  // Getters
  inline const std::string& coord_name(int idx) const {
    return coord_names_.at(idx);
  }
  inline const Vector3d& comdd() const { return comdd_; }
  inline const Vector6d& pelvdd() const { return pelvdd_; }
  inline const Vector6d& torsodd() const { return torsodd_; }
  inline const Vector6d& footdd(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return footdd_[0];
    else
      return footdd_[1];
  }
  inline const VectorXd& vd() const { return vd_; }
  inline const VectorXd& joint_torque() const { return joint_torque_; }
  inline const Vector6d& foot_wrench_in_world_frame(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return foot_wrench_in_world_frame_[0];
    else
      return foot_wrench_in_world_frame_[1];
  }
  inline const Vector6d& foot_wrench_in_sensor_frame(Side::SideEnum s) const {
    if (s == Side::LEFT)
      return foot_wrench_in_sensor_frame_[0];
    else
      return foot_wrench_in_sensor_frame_[1];
  }
  inline const Vector6d& footdd(int s) const {
    return footdd(Side::values.at(s));
  }
  inline const Vector6d& foot_wrench_in_world_frame(int s) const {
    return foot_wrench_in_world_frame(Side::values.at(s));
  }
  inline const Vector6d& foot_wrench_in_sensor_frame(int s) const {
    return foot_wrench_in_sensor_frame(Side::values.at(s));
  }
  inline const std::vector<std::pair<std::string, double>>& costs() const {
    return costs_;
  };
  inline const std::pair<std::string, double>& costs(size_t i) const {
    return costs_.at(i);
  };

  // Setters
  inline Vector3d& mutable_comdd() { return comdd_; }
  inline Vector6d& mutable_pelvdd() { return pelvdd_; }
  inline Vector6d& mutable_torsodd() { return torsodd_; }
  inline Vector6d& mutable_footdd(Side::SideEnum s) {
    if (s == Side::LEFT)
      return footdd_[0];
    else
      return footdd_[1];
  }
  inline VectorXd& mutable_vd() { return vd_; }
  inline VectorXd& mutable_joint_torque() { return joint_torque_; }
  inline Vector6d& mutable_foot_wrench_in_world_frame(Side::SideEnum s) {
    if (s == Side::LEFT)
      return foot_wrench_in_world_frame_[0];
    else
      return foot_wrench_in_world_frame_[1];
  }
  inline Vector6d& mutable_foot_wrench_in_sensor_frame(Side::SideEnum s) {
    if (s == Side::LEFT)
      return foot_wrench_in_sensor_frame_[0];
    else
      return foot_wrench_in_sensor_frame_[1];
  }
  inline Vector6d& mutable_footdd(int s) {
    return mutable_footdd(Side::values.at(s));
  }
  inline Vector6d& mutable_foot_wrench_in_world_frame(int s) {
    return mutable_foot_wrench_in_world_frame(Side::values.at(s));
  }
  inline Vector6d& mutable_foot_wrench_in_sensor_frame(int s) {
    return mutable_foot_wrench_in_sensor_frame(Side::values.at(s));
  }
  inline std::vector<std::pair<std::string, double>>& mutable_costs() {
    return costs_;
  };
  inline std::pair<std::string, double>& mutable_cost(size_t i) {
    return costs_.at(i);
  };
};
std::ostream& operator<<(std::ostream& out, const QPOutput& output);

class QPController {
 private:
  // These are temporary matrices used by the controller.
  Eigen::MatrixXd stacked_contact_jacobians_;
  Eigen::VectorXd stacked_contact_jacobians_dot_times_v_;
  Eigen::MatrixXd basis_to_force_matrix_;

  Eigen::MatrixXd torque_linear_;
  Eigen::VectorXd torque_constant_;
  Eigen::MatrixXd dynamics_linear_;
  Eigen::VectorXd dynamics_constant_;

  Eigen::MatrixXd inequality_linear_;
  Eigen::VectorXd inequality_upper_bound_;
  Eigen::VectorXd inequality_lower_bound_;

  Eigen::MatrixXd JB_;
  Eigen::MatrixXd point_force_to_wrench_;
  Eigen::VectorXd contact_wrenches_;

  // Number of basis per contact_point, this needs to be > 3
  // The bigger this number the close to approximate a real friction cone.
  // Typically set to 4.
  const int num_basis_per_contact_point_;

  // These determines the size of the QP. These are set in ResizeQP
  int num_supports_;
  int num_vd_;
  int num_point_forces_;
  int num_basis_;
  int num_torque_;
  int num_variable_;

  // prog_ is only allocated in ResizeQP, Control only updates the appropriate
  // matrices / vectors.
  drake::solvers::MathematicalProgram prog_;
  // TODO(siyuan.feng@tri.global): switch to other faster solvers when they are
  // ready: gurobi / fastQP
  drake::solvers::SnoptSolver solver_;

  // pointers to different cost / constraint terms inside prog_
  std::shared_ptr<drake::solvers::LinearEqualityConstraint> eq_dynamics_;
  std::vector<std::shared_ptr<drake::solvers::LinearEqualityConstraint>>
      eq_contacts_;
  std::shared_ptr<drake::solvers::LinearConstraint> ineq_contact_wrench_;
  std::shared_ptr<drake::solvers::LinearConstraint> ineq_torque_limit_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_comdd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_pelvdd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_torsodd_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_footdd_[2];
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_vd_reg_;

  /**
   * Resize the QP. This resizes the temporary matrices. It also
   * reinitialize prog_ to the correct size, so that Control only updates the
   * matrices and vectors in prog_ instead of making a new one on every call.
   * Size change typically happens when contact state changes (making / breaking
   * contacts).
   */
  void ResizeQP(const HumanoidStatus& rs,
                const std::vector<SupportElement>& all_contacts);

  /**
   * Zeros out the temporary matrices.
   * Only necessary for those that are updated by block operations.
   */
  void SetTempMatricesToZero() {
    basis_to_force_matrix_.setZero();
    torque_linear_.setZero();
    dynamics_linear_.setZero();
    inequality_linear_.setZero();
    point_force_to_wrench_.setZero();

    JB_.setZero();
    point_force_to_wrench_.setZero();
  }

 public:
  /**
   * @param rs robot configuration
   * @param n number of basis vector per contact point
   */
  explicit QPController(const HumanoidStatus& rs, int n)
      : num_basis_per_contact_point_(n) {}

  /**
   * Computes the generalized acceleration, joint torque and contact wrenches
   * that best tracks the input given the current robot configuration.
   * @param rs, robot configuration
   * @param input specified by a higher level controller
   * @param output stores the output
   * @return 0 if successful. < 1 if error.
   */
  int Control(const HumanoidStatus& rs, const QPInput& input, QPOutput* output);
};
