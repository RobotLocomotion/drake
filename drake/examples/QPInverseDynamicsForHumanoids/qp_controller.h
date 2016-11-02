#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
*This is used to compute target task space acceleration, which is the input
*to the inverse dynamics controller.
*The target acceleration is computed by:
*acceleration_d = Kp*(x* - x) + Kd*(xd* - xd) + xdd*,
*where x is pose, xd is velocity, and xdd is acceleration.
*Variables with superscript * are the set points, and Kp and Kd are the
*position and velocity gains.
*The first terms 3 are angular accelerations, and the last 3 are linear
*accelerations.
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

  /**
   * @param pose_d Desired pose
   * @param vel_d Desired velocity
   * @param acc_d Desired feedforward acceleration
   * @param Kp Position gain
   * @param Kd Velocity gain
   */
  CartesianSetPoint(const Eigen::Isometry3d& pose_d,
                    const Eigen::Vector6d& vel_d, const Eigen::Vector6d& acc_d,
                    const Eigen::Vector6d& Kp, const Eigen::Vector6d& Kd) {
    pose_d_ = pose_d;
    vel_d_ = vel_d;
    acc_d_ = acc_d;
    Kp_ = Kp;
    Kd_ = Kd;
  }

  /**
   * Computes target acceleration using PD feedback + feedfoward acceleration.
   * @param pose Measured pose
   * @param vel Measured velocity
   * @return Computed spatial acceleration.
   */
  Eigen::Vector6d ComputeTargetAcceleration(const Eigen::Isometry3d& pose,
                                            const Eigen::Vector6d& vel) const {
    // feedforward acc + velocity feedback
    Eigen::Vector6d qdd = acc_d_;
    qdd += (Kd_.array() * (vel_d_ - vel).array()).matrix();

    // pose feedback
    // H^w_d = desired orientation in the world frame,
    // H^w_m = measured orientation in the world frame,
    // E = a small rotation in the world frame from measured to desired.
    // H^w_d = E * H^w_m, E = H^w_d * H^w_m.transpose()
    Eigen::Matrix3d R_err = pose_d_.linear() * pose.linear().transpose();
    Eigen::AngleAxisd angle_axis_err(R_err);

    Eigen::Vector3d pos_err = pose_d_.translation() - pose.translation();
    Eigen::Vector3d rot_err = angle_axis_err.axis() * angle_axis_err.angle();

    // orientation
    qdd.segment<3>(0) += (Kp_.segment<3>(0).array() * rot_err.array()).matrix();

    // position
    qdd.segment<3>(3) += (Kp_.segment<3>(3).array() * pos_err.array()).matrix();

    return qdd;
  }

  // Getters
  inline const Eigen::Isometry3d& desired_pose() const { return pose_d_; }
  inline const Eigen::Vector6d& desired_velocity() const { return vel_d_; }
  inline const Eigen::Vector6d& desired_acceleration() const { return acc_d_; }
  inline const Eigen::Vector6d& Kp() const { return Kp_; }
  inline const Eigen::Vector6d& Kd() const { return Kd_; }

  // Setters
  inline Eigen::Isometry3d& mutable_desired_pose() { return pose_d_; }
  inline Eigen::Vector6d& mutable_desired_velocity() { return vel_d_; }
  inline Eigen::Vector6d& mutable_desired_acceleration() { return acc_d_; }
  inline Eigen::Vector6d& mutable_Kp() { return Kp_; }
  inline Eigen::Vector6d& mutable_Kd() { return Kd_; }

 private:
  Eigen::Isometry3d pose_d_;  ///< Desired pose
  Eigen::Vector6d vel_d_;     ///< Desired velocity
  Eigen::Vector6d acc_d_;     ///< Desired acceleration

  Eigen::Vector6d Kp_;  ///< Position gains
  Eigen::Vector6d Kd_;  ///< Velocity gains
};

/**
 * This class describes contact related information for each body in contact
 * with the world.
 * Each contact body has a set of point contacts. For each contact point,
 * only point contact force can be applied, and the friction cone is
 * approximated by a set of basis vectors.
 */
class ContactInformation {
 private:
  const RigidBody& body_;  ///< Link in contact
  /// Offsets of the contact point specified in the body frame.
  std::vector<Eigen::Vector3d> contact_points_;

  // TODO(siyuan.feng@tri.global): normal is currently assumed to be the same
  // for all the contact points.
  /// Contact normal specified in the body frame.
  Eigen::Vector3d normal_;

  /// Number of basis vectors per contact point
  int num_basis_per_contact_point_;

  /// Friction coeff
  double mu_;

 public:
  /*
   * @param num_basis_per_contact_point number of basis per contact point
   */
  ContactInformation(const RigidBody& b, int num_basis_per_contact_point)
      : body_(b), num_basis_per_contact_point_(num_basis_per_contact_point) {
    normal_ = Eigen::Vector3d(0, 0, 1);
    mu_ = 1;
    if (num_basis_per_contact_point_ < 3)
      throw std::runtime_error(
          "Number of basis per contact point must be >= 3.");
  }

  /**
   * Computes a matrix (Basis) that converts a vector of scalars (Beta) to
   * the stacked point contact forces (F).
   * All the point forces are in the world frame, and are applied at
   * the contact points in the world frame.
   * Basis is 3 * number of contact points by
   * \param num_basis_per_contact_point * number of contact points.
   * @param robot model
   * @param cache that stores the kinematics information, needs to be
   * initialized first.
   * @return Basis matrix
   */
  Eigen::MatrixXd ComputeBasisMatrix(
      const RigidBodyTree& robot, const KinematicsCache<double>& cache) const {
    Eigen::MatrixXd basis(
        3 * contact_points_.size(),
        num_basis_per_contact_point_ * contact_points_.size());
    Eigen::Matrix3d body_rot =
        robot.relativeTransform(cache, 0, body_.get_body_index()).linear();
    basis.setZero();

    Eigen::Vector3d t1, t2, tangent_vec, base;
    double theta;

    // Computes the tangent vectors that are perpendicular to normal_.
    if (fabs(1 - normal_[2]) < EPSILON) {
      t1 << 1, 0, 0;
    } else if (fabs(1 + normal_[2]) < EPSILON) {
      // same for the reflected case
      t1 << -1, 0, 0;
    } else {
      t1 << normal_[1], -normal_[0], 0;
      t1 /= sqrt(normal_[1] * normal_[1] + normal_[0] * normal_[0]);
    }
    t2 = t1.cross(normal_);

    for (int i = 0; i < static_cast<int>(contact_points_.size()); i++) {
      for (int k = 0; k < num_basis_per_contact_point_; k++) {
        theta = k * 2 * M_PI / num_basis_per_contact_point_;
        tangent_vec = cos(theta) * t1 + sin(theta) * t2;
        base = (normal_ + mu_ * tangent_vec).normalized();
        // Rotate basis into world frame.
        basis.block(3 * i, num_basis_per_contact_point_ * i + k, 3, 1) =
            body_rot * base;
      }
    }
    return basis;
  }

  /**
   * Computes the contact points and reference point location in the world
   * frame.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @param offset Offset for the reference point expressed in body frame.
   * @param contact_points Output of the function. Holds the contact point
   * locations.
   * @param reference_point Output of the function. Holds the reference point
   * location.
   */
  void ComputeContactPointsAndWrenchReferencePoint(
      const RigidBodyTree& robot, const KinematicsCache<double>& cache,
      const Eigen::Vector3d& offset,
      std::vector<Eigen::Vector3d>* contact_points,
      Eigen::Vector3d* reference_point) const {
    *reference_point =
        robot.transformPoints(cache, offset, body_.get_body_index(), 0);
    contact_points->resize(contact_points_.size());
    for (size_t i = 0; i < contact_points_.size(); i++) {
      (*contact_points)[i] = robot.transformPoints(cache, contact_points_[i],
                                                   body_.get_body_index(), 0);
    }
  }

  /**
   * Computes a matrix that converts a vector of stacked point forces
   * to an equivalent wrench in a frame that has the same orientation as the
   * world frame, but located at \param reference_point. The stacked point
   * forces are assumed to have the same order of \param contact_points.
   * @param contact_points where the point forces are applied at. These are in
   * the world frame.
   * @param referece_point the reference point for the equivalent wrench.
   * @return The matrix that converts point forces to an equivalent wrench.
   */
  Eigen::MatrixXd ComputeWrenchMatrix(
      const std::vector<Eigen::Vector3d>& contact_points,
      const Eigen::Vector3d& reference_point) const {
    if (contact_points.size() != contact_points_.size())
      throw std::runtime_error("contact points size mismatch");

    Eigen::MatrixXd force_to_wrench =
        Eigen::MatrixXd::Zero(6, 3 * contact_points.size());
    int col_idx = 0;
    for (const Eigen::Vector3d& contact_point : contact_points) {
      // Force part: just sum up all the point forces, so these are I
      force_to_wrench.block<3, 3>(3, col_idx).setIdentity();
      // Torque part:
      force_to_wrench.block<3, 3>(0, col_idx) =
          drake::math::VectorToSkewSymmetric(contact_point - reference_point);
      col_idx += 3;
    }
    return force_to_wrench;
  }

  /**
   * Computes the stacked task space Jacobian matrix (only the position part)
   * for all the contact points.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @return The stacked Jacobian matrix
   */
  Eigen::MatrixXd ComputeJacobianAtContactPoints(
      const RigidBodyTree& robot, const KinematicsCache<double>& cache) const {
    Eigen::MatrixXd J(3 * contact_points_.size(), robot.get_num_velocities());
    for (size_t i = 0; i < contact_points_.size(); i++) {
      J.block(3 * i, 0, 3, robot.get_num_velocities()) =
          GetTaskSpaceJacobian(robot, cache, body_, contact_points_[i])
              .bottomRows<3>();
    }
    return J;
  }

  /**
   * Computes the stacked task space Jacobian dot times v vector (only the
   * position part) for all the contact points.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @return The stacked Jacobian dot times v vector
   */
  Eigen::VectorXd ComputeJacobianDotTimesVAtContactPoints(
      const RigidBodyTree& robot, const KinematicsCache<double>& cache) const {
    Eigen::VectorXd Jdv(3 * contact_points_.size());
    for (size_t i = 0; i < contact_points_.size(); i++) {
      Jdv.segment<3>(3 * i) =
          GetTaskSpaceJacobianDotTimesV(robot, cache, body_, contact_points_[i])
              .bottomRows<3>();
    }
    return Jdv;
  }

  inline const std::string& name() const { return body_.get_name(); }
  inline int num_contact_points() const {
    return static_cast<int>(contact_points_.size());
  }
  inline int num_basis() const {
    return num_contact_points() * num_basis_per_contact_point_;
  }

  // Getters
  inline double mu() const { return mu_; }
  inline const std::vector<Eigen::Vector3d>& contact_points() const {
    return contact_points_;
  }
  inline const Eigen::Vector3d& normal() const { return normal_; }
  inline const RigidBody& body() const { return body_; }
  inline int num_basis_per_contact_point() const {
    return num_basis_per_contact_point_;
  }

  // Setters
  inline std::vector<Eigen::Vector3d>& mutable_contact_points() {
    return contact_points_;
  }
  inline double& mutable_mu() { return mu_; }
  inline Eigen::Vector3d& mutable_normal() { return normal_; }
  inline int& mutable_num_basis_per_contact_point() {
    return num_basis_per_contact_point_;
  }
};

/**
 * This class holds the contact force / wrench related information, and works
 * closely with ContactInformation.
 */
class ResolvedContact {
 private:
  const RigidBody& body_;
  // Stacked scalars for all the basis vectors.
  Eigen::VectorXd basis_;

  // Point contact forces in the world frame.
  std::vector<Eigen::Vector3d> point_forces_;

  // Contact points in the world frame.
  std::vector<Eigen::Vector3d> contact_points_;

  // The equivalent wrench of all the point forces, w.r.t a frame that has
  // the same orientation as the world frame, but located at reference_point.
  Eigen::Vector6d equivalent_wrench_;

  // Reference point in the world frame for the equivalent wrench.
  Eigen::Vector3d reference_point_;

 public:
  explicit ResolvedContact(const RigidBody& body) : body_(body) {}

  // Getters
  inline const RigidBody& body() const { return body_; }
  inline const std::string& name() const { return body_.get_name(); }
  inline const Eigen::VectorXd& basis() const { return basis_; }
  inline const std::vector<Eigen::Vector3d>& point_forces() const {
    return point_forces_;
  }
  inline const Eigen::Vector3d& point_force(size_t i) const {
    return point_forces_.at(i);
  }
  inline const std::vector<Eigen::Vector3d>& contact_points() const {
    return contact_points_;
  }
  inline const Eigen::Vector3d& contact_point(size_t i) const {
    return contact_points_.at(i);
  }
  inline const Eigen::Vector6d& equivalent_wrench() const {
    return equivalent_wrench_;
  }
  inline const Eigen::Vector3d& reference_point() const {
    return reference_point_;
  }

  // Setters
  inline Eigen::VectorXd& mutable_basis() { return basis_; }
  inline std::vector<Eigen::Vector3d>& mutable_point_forces() {
    return point_forces_;
  }
  inline Eigen::Vector3d& mutable_point_force(size_t i) {
    return point_forces_.at(i);
  }
  inline std::vector<Eigen::Vector3d>& mutable_contact_points() {
    return contact_points_;
  }
  inline Eigen::Vector3d& mutable_contact_point(size_t i) {
    return contact_points_.at(i);
  }
  inline Eigen::Vector6d& mutable_equivalent_wrench() {
    return equivalent_wrench_;
  }
  inline Eigen::Vector3d& mutable_reference_point() { return reference_point_; }
};

/**
 * This class holds task space acceleration for a rigid body.
 */
class BodyAcceleration {
 protected:
  const RigidBody& body_;
  Eigen::Vector6d acceleration_;

 public:
  explicit BodyAcceleration(const RigidBody& body)
      : body_(body), acceleration_(Eigen::Vector6d::Zero()) {}

  inline bool is_valid() const { return acceleration_.allFinite(); }

  // Getters
  inline const RigidBody& body() const { return body_; }
  inline const std::string& name() const { return body_.get_name(); }
  inline const Eigen::Vector6d& acceleration() const { return acceleration_; }

  // Setter
  inline Eigen::Vector6d& mutable_acceleration() { return acceleration_; }
};

/**
 * In addition to the desired task space acceleration, this class also holds
 * a weighting scalar used in the cost function of some optimization.
 */
class DesiredBodyAcceleration : public BodyAcceleration {
 private:
  double weight_;

 public:
  explicit DesiredBodyAcceleration(const RigidBody& body)
      : BodyAcceleration(body), weight_(0) {}

  inline bool is_valid() const {
    return BodyAcceleration::is_valid() && std::isfinite(weight_) &&
           weight_ >= 0;
  }

  // Getters
  inline double weight() const { return weight_; }

  // Setters
  inline double& mutable_weight() { return weight_; }
};

/**
 * Input to the QP inverse dynamics controller
 */
class QPInput {
 private:
  // Names for each generalized coordinate
  std::vector<std::string> coord_names_;
  // Contact information
  std::vector<ContactInformation> contact_info_;

  // Desired task space accelerations for specific bodies
  std::vector<DesiredBodyAcceleration> desired_body_accelerations_;
  // Desired task space accelerations for the center of mass
  Eigen::Vector3d desired_comdd_;
  // Desired generalized coordinate accelerations
  Eigen::VectorXd desired_vd_;

  // These are weights for each cost term.
  // Prefix w_ indicates weights.
  double w_com_;
  double w_vd_;
  double w_basis_reg_;

 public:
  explicit QPInput(const RigidBodyTree& r) {
    coord_names_.resize(r.get_num_velocities());
    for (int i = 0; i < r.get_num_velocities(); i++) {
      // strip out the "dot" part from name
      coord_names_[i] =
          r.get_velocity_name(i).substr(0, r.get_velocity_name(i).size() - 3);
    }
    desired_vd_.resize(r.get_num_velocities());
  }

  /**
   * Checks validity of this QPInput.
   * @param size Dimension of acceleration in the generalized coordinates.
   * @return true if this QPInput is valid.
   */
  bool is_valid(int num_vd) const {
    int valid = true;
    valid &= static_cast<int>(coord_names_.size()) == num_vd;
    valid &= static_cast<int>(coord_names_.size()) == desired_vd_.size();

    valid &= desired_comdd_.allFinite();
    valid &= desired_vd_.allFinite();
    for (const DesiredBodyAcceleration& desired_body_acceleration :
         desired_body_accelerations_) {
      valid &= desired_body_acceleration.is_valid();
    }

    valid &= std::isfinite(w_com_);
    valid &= w_com_ >= 0;
    valid &= std::isfinite(w_vd_);
    valid &= w_vd_ >= 0;
    valid &= std::isfinite(w_basis_reg_);
    valid &= w_basis_reg_ >= 0;

    return valid;
  }

  // Getters
  inline double w_com() const { return w_com_; }
  inline double w_vd() const { return w_vd_; }
  inline double w_basis_reg() const { return w_basis_reg_; }

  inline const std::string& coord_name(size_t idx) const {
    return coord_names_.at(idx);
  }
  inline const ContactInformation& contact_info(size_t idx) const {
    return contact_info_.at(idx);
  }
  inline const std::vector<ContactInformation>& contact_info() const {
    return contact_info_;
  }
  inline const std::vector<DesiredBodyAcceleration>&
  desired_body_accelerations() const {
    return desired_body_accelerations_;
  }
  inline const DesiredBodyAcceleration& desired_body_acceleration(
      size_t idx) const {
    return desired_body_accelerations_.at(idx);
  }
  inline const Eigen::Vector3d& desired_comdd() const { return desired_comdd_; }
  inline const Eigen::VectorXd& desired_vd() const { return desired_vd_; }

  // Setters
  inline double& mutable_w_com() { return w_com_; }
  inline double& mutable_w_vd() { return w_vd_; }
  inline double& mutable_w_basis_reg() { return w_basis_reg_; }

  inline std::vector<ContactInformation>& mutable_contact_info() {
    return contact_info_;
  }
  inline ContactInformation& mutable_contact_info(size_t idx) {
    return contact_info_.at(idx);
  }
  inline std::vector<DesiredBodyAcceleration>&
  mutable_desired_body_accelerations() {
    return desired_body_accelerations_;
  }
  inline DesiredBodyAcceleration& mutable_desired_body_acceleration(
      size_t idx) {
    return desired_body_accelerations_.at(idx);
  }
  inline Eigen::Vector3d& mutable_desired_comdd() { return desired_comdd_; }
  inline Eigen::VectorXd& mutable_desired_vd() { return desired_vd_; }
};
std::ostream& operator<<(std::ostream& out, const QPInput& input);

/**
 * Output of the QP inverse dynamics controller
 */
class QPOutput {
 private:
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names_;

  // Tracked body motion
  std::vector<BodyAcceleration> body_accelerations_;

  // Computed task space accelerations for the center of mass
  Eigen::Vector3d comdd_;
  // Computed generalized coordinate accelerations
  Eigen::VectorXd vd_;
  // Computed joint torque
  Eigen::VectorXd joint_torque_;

  // Computed contact wrench in the world frame.
  // The first part of the pair is the reference point in the world frame,
  // and the second is the wrench.
  // std::vector<ComputedContactInformation> contact_
  std::vector<ResolvedContact> resolved_contacts_;

  // Pair of the name of cost term and cost value (only the quadratic and linear
  // term, no constant term).
  std::vector<std::pair<std::string, double>> costs_;

 public:
  explicit QPOutput(const RigidBodyTree& r) {
    coord_names_.resize(r.get_num_velocities());
    for (int i = 0; i < r.get_num_velocities(); i++) {
      // strip out the "dot" part from name
      coord_names_[i] =
          r.get_velocity_name(i).substr(0, r.get_velocity_name(i).size() - 3);
    }
    vd_.resize(r.get_num_velocities());
    joint_torque_.resize(r.actuators.size());
  }

  bool is_valid(int num_vd, int num_actuators) const {
    bool ret = static_cast<int>(coord_names_.size()) == vd_.size();
    ret &= vd_.size() == num_vd;
    ret &= joint_torque_.size() == num_actuators;

    ret &= comdd_.allFinite();
    ret &= vd_.allFinite();
    ret &= joint_torque_.allFinite();

    for (const BodyAcceleration& body_acceleration : body_accelerations_) {
      ret &= body_acceleration.is_valid();
    }

    return ret;
  }

  // Getters
  inline const std::string& coord_name(int idx) const {
    return coord_names_.at(idx);
  }
  inline const Eigen::Vector3d& comdd() const { return comdd_; }
  inline const Eigen::VectorXd& vd() const { return vd_; }
  inline const std::vector<BodyAcceleration>& body_accelerations() const {
    return body_accelerations_;
  }
  inline const BodyAcceleration& body_acceleration(size_t i) const {
    return body_accelerations_.at(i);
  }
  inline const Eigen::VectorXd& joint_torque() const { return joint_torque_; }
  inline const std::vector<ResolvedContact>& resolved_contacts() const {
    return resolved_contacts_;
  }
  inline const ResolvedContact& resolved_contact(size_t i) const {
    return resolved_contacts_.at(i);
  }
  inline const std::vector<std::pair<std::string, double>>& costs() const {
    return costs_;
  }
  inline const std::pair<std::string, double>& costs(size_t i) const {
    return costs_.at(i);
  }

  // Setters
  inline Eigen::Vector3d& mutable_comdd() { return comdd_; }
  inline Eigen::VectorXd& mutable_vd() { return vd_; }
  inline std::vector<BodyAcceleration>& mutable_body_accelerations() {
    return body_accelerations_;
  }
  inline BodyAcceleration& mutable_body_acceleration(size_t i) {
    return body_accelerations_.at(i);
  }
  inline std::vector<ResolvedContact>& mutable_resolved_contacts() {
    return resolved_contacts_;
  }
  inline ResolvedContact& mutable_resolved_contact(size_t i) {
    return resolved_contacts_.at(i);
  }
  inline Eigen::VectorXd& mutable_joint_torque() { return joint_torque_; }

  inline std::vector<std::pair<std::string, double>>& mutable_costs() {
    return costs_;
  }
  inline std::pair<std::string, double>& mutable_cost(size_t i) {
    return costs_.at(i);
  }
};
std::ostream& operator<<(std::ostream& out, const QPOutput& output);

class QPController {
 private:
  // These are temporary matrices and vectors used by the controller.
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
  Eigen::VectorXd point_forces_;

  Eigen::MatrixXd mass_matrix_;
  Eigen::VectorXd dynamics_bias_;

  Eigen::MatrixXd J_com_;
  Eigen::VectorXd J_dot_times_v_com_;
  Eigen::MatrixXd centroidal_momentum_matrix_;
  Eigen::VectorXd centroidal_momentum_matrix_dot_times_v_;

  std::vector<Eigen::MatrixXd> body_J_;
  std::vector<Eigen::VectorXd> body_Jdv_;

  // These determines the size of the QP. These are set in ResizeQP
  int num_contact_body_;
  int num_vd_;
  int num_point_force_;
  int num_basis_;
  int num_torque_;
  int num_variable_;
  int num_body_acceleration_;

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
  // TODO(siyuan.feng@tri.global): switch to cost for contact_constraints
  std::vector<std::shared_ptr<drake::solvers::QuadraticConstraint>>
      cost_body_accelerations_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_vd_reg_;
  std::shared_ptr<drake::solvers::QuadraticConstraint> cost_basis_reg_;

  /**
   * Resize the QP. This resizes the temporary matrices. It also reinitializes
   * prog_ to the correct size, so that Control only updates the
   * matrices and vectors in prog_ instead of making a new one on every call.
   * Size change typically happens when contact state changes (making / breaking
   * contacts).
   * @param robot Model
   * @param all_contacts Information about contacts
   * @param all_body_accelerations Desired body accelerations to be tracked
   */
  void ResizeQP(
      const RigidBodyTree& robot,
      const std::vector<ContactInformation>& all_contacts,
      const std::vector<DesiredBodyAcceleration>& all_body_accelerations);

  /**
   * Zeros out the temporary matrices.
   * Only necessary for those that are updated by block operations.
   */
  void SetTempMatricesToZero() {
    basis_to_force_matrix_.setZero();
    torque_linear_.setZero();
    dynamics_linear_.setZero();
    inequality_linear_.setZero();

    JB_.setZero();
  }

 public:
  /**
   * Computes the generalized acceleration, joint torque and contact wrenches
   * that best tracks the input given the current robot configuration.
   * @param rs Robot configuration
   * @param input Desired motions and objectives specified by a higher level
   * controller
   * @param output Container for outputs
   * @return 0 if successful. < 0 if error.
   */
  int Control(const HumanoidStatus& rs, const QPInput& input, QPOutput* output);

  static const double kUpperBoundForContactBasis;
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
