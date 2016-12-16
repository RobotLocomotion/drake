#pragma once

#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/math/cross_product.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Enum class for constraint types.
 * Hard: will be enforced by equality constraints.
 * Soft: will be enforced by cost functions.
 */
enum class ConstraintType { Hard = -1, Skip = 0, Soft = 1 };

std::ostream& operator<<(std::ostream& out, const ConstraintType& type);

/**
 * Base class for specifying various desired objectives.
 * The objectives can be ignored, set as equality constraints or optimized as
 * cost terms depending on the specified types.
 * For cost terms, a positive weight needs to be specified.
 */
class ConstrainedValues {
 private:
  std::vector<ConstraintType> constraint_types_;
  VectorX<double> weights_;
  VectorX<double> values_;

 public:
  ConstrainedValues() {}

  explicit ConstrainedValues(int dim)
      : constraint_types_(dim, ConstraintType::Skip),
        weights_(VectorX<double>::Zero(dim)),
        values_(VectorX<double>::Zero(dim)) {}

  void resize(int dim) {
    constraint_types_.resize(dim);
    weights_.resize(dim);
    values_.resize(dim);
  }

  /**
   * Get all the indices that has the specified constraint type.
   * @param type Matching constraint type
   * @return indices
   */
  std::list<int> GetConstraintTypeIndices(ConstraintType type) const {
    std::list<int> ret;
    for (int i = 0; i < static_cast<int>(constraint_types_.size()); ++i) {
      if (constraint_types_[i] == type) ret.push_back(i);
    }

    return ret;
  }

  /**
   * Set given indices' constraint types to the given type.
   * @param indices List of indices
   * @param type Desired type
   */
  void SetConstraintType(const std::list<int>& indices, ConstraintType type) {
    for (int i : indices) {
      if (i < static_cast<int>(constraint_types_.size()) && i >= 0)
        constraint_types_[i] = type;
    }
  }

  /**
   * Set all constraint types to the given type.
   * @param type Desired type
   */
  void SetAllConstraintType(ConstraintType type) {
    for (size_t i = 0; i < constraint_types_.size(); ++i) {
      constraint_types_[i] = type;
    }
  }

  inline bool is_valid() const { return is_valid(size()); }

  bool is_valid(int dim) const {
    if (weights_.size() != dim || weights_.size() != values_.size() ||
        weights_.size() != static_cast<int>(constraint_types_.size())) {
      return false;
    }
    for (int i = 0; i < dim; ++i) {
      if (constraint_types_[i] == ConstraintType::Soft && weights_[i] <= 0) {
        return false;
      }
    }
    if (!weights_.allFinite()) {
      return false;
    }
    if (!values_.allFinite()) {
      return false;
    }
    return true;
  }

  bool operator==(const ConstrainedValues& other) const {
    if (constraint_types_.size() != other.constraint_types_.size()) {
      return false;
    }

    for (size_t i = 0; i < constraint_types_.size(); ++i) {
      if (constraint_types_[i] != other.constraint_types_[i]) {
        return false;
      }
    }
    if (!weights_.isApprox(other.weights_)) {
      return false;
    }
    if (!values_.isApprox(other.values_)) {
      return false;
    }
    return true;
  }

  inline bool operator!=(const ConstrainedValues& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline int size() const { return values_.size(); }
  inline const VectorX<double>& weights() const { return weights_; }
  inline const VectorX<double>& values() const { return values_; }
  inline const std::vector<ConstraintType>& constraint_types() const {
    return constraint_types_;
  }

  inline double value(int i) const { return values_[i]; }
  inline double weight(int i) const { return weights_[i]; }
  inline ConstraintType constraint_type(int i) const {
    return constraint_types_.at(i);
  }

  // Setters
  inline VectorX<double>& mutable_weights() { return weights_; }
  inline VectorX<double>& mutable_values() { return values_; }
  inline std::vector<ConstraintType>& mutable_constraint_types() {
    return constraint_types_;
  }

  inline double& mutable_value(int i) { return values_[i]; }
  inline double& mutable_weight(int i) { return weights_[i]; }
  inline ConstraintType& mutable_constraint_type(int i) {
    return constraint_types_.at(i);
  }
};

/**
 * This class describes contact related information for each body in contact
 * with the world.
 * Each contact body has a set of point contacts. For each contact point,
 * only point contact force can be applied, and the friction cone is
 * approximated by a set of basis vectors.
 *
 * The stationary contact (small foot acceleration) condition can be described
 * as:
 * J * vd + J_dot_times_vd = Kd * (0 - v_contact_pt).
 * Only the linear velocities and accelerations are considered here.
 * Kd >= 0 is a stabilizing velocity gain to damp out contact velocity
 * This condition can be enforced either as an equality constraint or as a
 * cost term.
 */
class ContactInformation {
 public:
  static const int kDefaultNumBasisPerContactPoint = 4;

  /*
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   * @param num_basis_per_contact_point number of basis per contact point
   */
  ContactInformation(const RigidBody<double>& body,
                     int num_basis = kDefaultNumBasisPerContactPoint)
      : body_(&body),
        num_basis_per_contact_point_(num_basis),
        acceleration_constraint_type_(ConstraintType::Hard) {
    normal_ = Vector3<double>(0, 0, 1);
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
  MatrixX<double> ComputeBasisMatrix(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    MatrixX<double> basis(
        3 * contact_points_.cols(),
        num_basis_per_contact_point_ * contact_points_.cols());
    Matrix3<double> body_rot =
        robot.relativeTransform(cache, 0, body_->get_body_index()).linear();
    basis.setZero();

    Vector3<double> t1, t2, tangent_vec, base;
    double theta;

    // Computes the tangent vectors that are perpendicular to normal_.
    if (std::abs(1 - normal_[2]) < Eigen::NumTraits<double>::epsilon()) {
      t1 << 1, 0, 0;
    } else if (std::abs(1 + normal_[2]) < Eigen::NumTraits<double>::epsilon()) {
      // same for the reflected case
      t1 << -1, 0, 0;
    } else {
      t1 << normal_[1], -normal_[0], 0;
      t1 /= sqrt(normal_[1] * normal_[1] + normal_[0] * normal_[0]);
    }
    t2 = t1.cross(normal_);

    for (int i = 0; i < contact_points_.cols(); ++i) {
      for (int k = 0; k < num_basis_per_contact_point_; ++k) {
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
      const RigidBodyTree<double>& robot, const KinematicsCache<double>& cache,
      const Vector3<double>& offset, drake::Matrix3X<double>* contact_points,
      Vector3<double>* reference_point) const {
    *reference_point =
        robot.transformPoints(cache, offset, body_->get_body_index(), 0);
    contact_points->resize(3, contact_points_.cols());
    for (int i = 0; i < contact_points_.cols(); ++i) {
      contact_points->col(i) = robot.transformPoints(
          cache, contact_points_.col(i), body_->get_body_index(), 0);
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
  MatrixX<double> ComputeWrenchMatrix(
      const Matrix3X<double>& contact_points,
      const Vector3<double>& reference_point) const {
    if (contact_points.cols() != contact_points_.cols())
      throw std::runtime_error("contact points size mismatch");

    MatrixX<double> force_to_wrench =
        MatrixX<double>::Zero(6, 3 * contact_points.cols());
    int col_idx = 0;
    for (int i = 0; i < contact_points.cols(); ++i) {
      // Force part: just sum up all the point forces, so these are I
      force_to_wrench.block<3, 3>(3, col_idx).setIdentity();
      // Torque part:
      force_to_wrench.block<3, 3>(0, col_idx) =
          drake::math::VectorToSkewSymmetric(contact_points.col(i) -
                                             reference_point);
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
  MatrixX<double> ComputeJacobianAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    MatrixX<double> J(3 * contact_points_.cols(), robot.get_num_velocities());
    for (int i = 0; i < contact_points_.cols(); ++i) {
      J.block(3 * i, 0, 3, robot.get_num_velocities()) =
          GetTaskSpaceJacobian(robot, cache, *body_, contact_points_.col(i))
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
  VectorX<double> ComputeJacobianDotTimesVAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    VectorX<double> Jdv(3 * contact_points_.cols());
    for (int i = 0; i < contact_points_.cols(); ++i) {
      Jdv.segment<3>(3 * i) =
          GetTaskSpaceJacobianDotTimesV(robot, cache, *body_,
                                        contact_points_.col(i)).bottomRows<3>();
    }
    return Jdv;
  }

  /**
   * Computes the stacked velocities for all the contact points.
   * @param robot Robot model
   * @param cache Stores the kinematics information, needs to be initialized
   * first.
   * @return Stacked velocities.
   */
  VectorX<double> ComputeLinearVelocityAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    VectorX<double> vel(3 * contact_points_.cols());
    for (int i = 0; i < contact_points_.cols(); ++i) {
      vel.segment<3>(3 * i) =
          GetTaskSpaceVel(robot, cache, *body_, contact_points_.col(i))
              .bottomRows<3>();
    }
    return vel;
  }

  bool is_valid() const {
    if (std::abs(normal_.norm() - 1) >= Eigen::NumTraits<double>::epsilon()) {
      return false;
    }
    if (!contact_points_.allFinite()) {
      return false;
    }
    if (mu_ < 0) {
      return false;
    }
    if (num_basis_per_contact_point_ < 3) {
      return false;
    }
    if (acceleration_constraint_type_ == ConstraintType::Soft && weight_ <= 0) {
      return false;
    }
    // Can't skip contact constraints
    if (acceleration_constraint_type_ == ConstraintType::Skip) {
      return false;
    }
    // Can't have a minus stabilizing velocity gain.
    if (Kd_ < 0) {
      return false;
    }
    return true;
  }

  bool operator==(const ContactInformation& other) const {
    if (body_ != other.body_) {
      return false;
    }
    if (!contact_points_.isApprox(other.contact_points_)) {
      return false;
    }
    if (!normal_.isApprox(other.normal_)) {
      return false;
    }
    if (num_basis_per_contact_point_ != other.num_basis_per_contact_point_) {
      return false;
    }
    if (mu_ != other.mu_) {
      return false;
    }
    if (Kd_ != other.Kd_) {
      return false;
    }
    if (weight_ != other.weight_) {
      return false;
    }
    if (acceleration_constraint_type_ != other.acceleration_constraint_type_) {
      return false;
    }
    return true;
  }

  inline bool operator!=(const ContactInformation& other) const {
    return !(this->operator==(other));
  }

  inline const std::string& body_name() const { return body_->get_name(); }
  inline int num_contact_points() const { return contact_points_.cols(); }
  inline int num_basis() const {
    return num_contact_points() * num_basis_per_contact_point_;
  }

  // Getters
  inline double mu() const { return mu_; }
  inline double weight() const { return weight_; }
  inline double Kd() const { return Kd_; }
  inline ConstraintType acceleration_constraint_type() const {
    return acceleration_constraint_type_;
  }
  inline const Matrix3X<double>& contact_points() const {
    return contact_points_;
  }

  inline const Vector3<double>& normal() const { return normal_; }
  inline const RigidBody<double>& body() const { return *body_; }
  inline int num_basis_per_contact_point() const {
    return num_basis_per_contact_point_;
  }

  // Setters
  inline Matrix3X<double>& mutable_contact_points() { return contact_points_; }
  inline double& mutable_mu() { return mu_; }
  inline double& mutable_weight() { return weight_; }
  inline double& mutable_Kd() { return Kd_; }
  inline ConstraintType& mutable_acceleration_constraint_type() {
    return acceleration_constraint_type_;
  }
  inline Vector3<double>& mutable_normal() { return normal_; }
  inline int& mutable_num_basis_per_contact_point() {
    return num_basis_per_contact_point_;
  }
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBody<double>* body_;
  // Offsets of the contact point specified in the body frame.
  Matrix3X<double> contact_points_;

  // TODO(siyuan.feng): Normal is currently assumed to be the same for all
  // the contact points.
  // Contact normal specified in the body frame.
  Vector3<double> normal_;

  int num_basis_per_contact_point_;

  // Friction coeff
  double mu_;

  // Velocity gain for stabilizing contact
  double Kd_;

  // Weight for the cost function term, only useful when
  // acceleration_constraint_type_ == ConstraintType::Soft.
  double weight_;

  // Determines if all contact points' acceleration are enforced as equality
  // constraints or summed in a cost term.
  // This cannot be ConstraintType::Skip.
  ConstraintType acceleration_constraint_type_;
};

std::ostream& operator<<(std::ostream& out, const ContactInformation& contact);

/**
 * A wrapper class specifying desired body motion (acceleration) for a rigid
 * body and their corresponding weights for the QP.
 * The acceleration is expressed in a frame that has the same orientation as
 * the world, and located at the origin of the body.
 *
 * The first three elements for weights and accelerations are angular.
 *
 * The desired acceleration can be skipped, enforced as equality constraints
 * or optimized as a cost term depending on the constraint type.
 *
 * TODO: (siyuan.feng) Expand this to be expressed in other frame.
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredBodyMotion : public ConstrainedValues {
 public:
  /**
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   */
  explicit DesiredBodyMotion(const RigidBody<double>& body)
      : ConstrainedValues(6), body_(&body), control_during_contact_(false) {}

  inline bool is_valid() const {
    return this->ConstrainedValues::is_valid(kTwistSize);
  }

  inline std::string get_row_name(int i) const {
    static const std::string row_name[kTwistSize] = {"[WX]", "[WY]", "[WZ]",
                                                     "[X]",  "[Y]",  "[Z]"};
    if (i < 0 || i >= kTwistSize)
      throw std::runtime_error("index must be within [0, 5]");
    return row_name[i];
  }

  bool operator==(const DesiredBodyMotion& other) const {
    if (body_ != other.body_) {
      return false;
    }
    if (control_during_contact_ != other.control_during_contact_) {
      return false;
    }

    return this->ConstrainedValues::operator==(other);
  }

  inline bool operator!=(const DesiredBodyMotion& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline const RigidBody<double>& body() const { return *body_; }
  inline const std::string& body_name() const { return body_->get_name(); }
  inline bool control_during_contact() const { return control_during_contact_; }

  // Setters
  inline bool& mutable_control_during_contact() {
    return control_during_contact_;
  }
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }

 private:
  const RigidBody<double>* body_;

  // TODO(siyuan.feng) Actually implement this in the qp controller.
  bool control_during_contact_;
};

std::ostream& operator<<(std::ostream& out, const DesiredBodyMotion& input);

/**
 * A wrapper class specifying desired DoF (degree of freedom)
 * motions (acceleration) and their corresponding weights for the QP.
 *
 * The desired acceleration can be skipped, enforced as equality constraints
 * or optimized as a cost term depending on the constraint type.
 *
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredDoFMotions : public ConstrainedValues {
 public:
  DesiredDoFMotions() {}
  explicit DesiredDoFMotions(const std::vector<std::string>& names)
      : ConstrainedValues(static_cast<int>(names.size())), dof_names_(names) {}

  inline bool is_valid() const {
    return this->DesiredDoFMotions::is_valid(size());
  }

  bool is_valid(int dim) const {
    if (static_cast<int>(dof_names_.size()) != dim) {
      return false;
    }
    return this->ConstrainedValues::is_valid(dim);
  }

  bool operator==(const DesiredDoFMotions& other) const {
    if (dof_names_.size() != other.dof_names_.size()) {
      return false;
    }
    for (size_t i = 0; i < dof_names_.size(); ++i) {
      if (dof_names_[i].compare(other.dof_names_[i]) != 0) {
        return false;
      }
    }

    return this->ConstrainedValues::operator==(other);
  }

  inline bool operator!=(const DesiredDoFMotions& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline const std::vector<std::string>& dof_names() const {
    return dof_names_;
  }
  inline const std::string& dof_name(int i) const { return dof_names_.at(i); }

 private:
  std::vector<std::string> dof_names_;
};

std::ostream& operator<<(std::ostream& out, const DesiredDoFMotions& input);

/**
 * A wrapper class specifying desired centroidal momentum change and their
 * corresponding weights for the QP.
 * The change in momentum are expressed in the world frame.
 * The first three terms are angular.
 * Linear momentum change = com acceleration * mass.
 *
 * The desired centroidal momentum change can be skipped, enforced as
 * equality constraints or optimized as a cost term depending on the
 * constraint type.
 *
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredCentroidalMomentumDot : public ConstrainedValues {
 public:
  DesiredCentroidalMomentumDot() : ConstrainedValues(kTwistSize) {}

  inline bool is_valid() const {
    return this->ConstrainedValues::is_valid(kTwistSize);
  }

  inline std::string get_row_name(int i) const {
    static const std::string row_name[6] = {"AngMom[X]", "AngMom[Y]",
                                            "AngMom[Z]", "LinMom[X]",
                                            "LinMom[Y]", "LinMom[Z]"};
    if (i < 0 || i >= kTwistSize)
      throw std::runtime_error("index must be within [0, 5]");
    return row_name[i];
  }
};

std::ostream& operator<<(std::ostream& out,
                         const DesiredCentroidalMomentumDot& input);

/**
 * Input to the QP inverse dynamics controller
 */
class QPInput {
 public:
  explicit QPInput(const RigidBodyTree<double>& r) {
    std::vector<std::string> names(r.get_num_velocities());
    // strip out the "dot" part from name
    for (int i = 0; i < r.get_num_velocities(); ++i)
      names[i] =
          r.get_velocity_name(i).substr(0, r.get_velocity_name(i).size() - 3);
    desired_dof_motions_ = DesiredDoFMotions(names);
  }

  inline bool is_valid() const { return is_valid(desired_dof_motions_.size()); }

  /**
   * Checks validity of this QPInput.
   * @param num_vd Dimension of acceleration in the generalized coordinates.
   * @return true if this is valid.
   */
  bool is_valid(int num_vd) const {
    if (num_vd != desired_dof_motions_.size()) {
      return false;
    }
    if (!desired_dof_motions_.is_valid(num_vd)) {
      return false;
    }
    for (const auto& body_motion_pair : desired_body_motions_) {
      if (!body_motion_pair.second.is_valid()) {
        return false;
      }
    }
    for (const auto& contact_pair : contact_info_) {
      if (!contact_pair.second.is_valid()) {
        return false;
      }
    }
    if (!desired_centroidal_momentum_dot_.is_valid()) {
      return false;
    }
    // Regularization weight needs to be positive.
    if (!std::isfinite(w_basis_reg_) || w_basis_reg_ <= 0) {
      return false;
    }
    return true;
  }

  bool operator==(const QPInput& other) const {
    if (contact_info_.size() != other.contact_info_.size() ||
        desired_body_motions_.size() != other.desired_body_motions_.size()) {
      return false;
    }
    for (const auto& contact_pair : contact_info_) {
      auto it = other.contact_info_.find(contact_pair.first);
      if (it == other.contact_info_.end()) {
        return false;
      }
      if (!(contact_pair.second == it->second)) {
        return false;
      }
    }
    for (const auto& body_motion_pair : desired_body_motions_) {
      auto it = other.desired_body_motions_.find(body_motion_pair.first);
      if (it == other.desired_body_motions_.end()) {
        return false;
      }
      if (!(body_motion_pair.second == it->second)) {
        return false;
      }
    }
    if (!(desired_dof_motions_ == other.desired_dof_motions_)) {
      return false;
    }
    if (!(desired_centroidal_momentum_dot_ ==
          other.desired_centroidal_momentum_dot_)) {
      return false;
    }
    if (w_basis_reg_ != other.w_basis_reg_) {
      std::cout << w_basis_reg_ << " " << other.w_basis_reg_;
      return false;
    }

    return true;
  }

  inline bool operator!=(const QPInput& other) const {
    return !(this->operator==(other));
  }

  // Getters
  inline double w_basis_reg() const { return w_basis_reg_; }
  inline const std::string& dof_name(size_t idx) const {
    return desired_dof_motions_.dof_name(idx);
  }
  inline const std::map<std::string, ContactInformation>& contact_information()
      const {
    return contact_info_;
  }
  inline const std::map<std::string, DesiredBodyMotion>& desired_body_motions()
      const {
    return desired_body_motions_;
  }
  inline const DesiredDoFMotions& desired_dof_motions() const {
    return desired_dof_motions_;
  }
  inline const DesiredCentroidalMomentumDot& desired_centroidal_momentum_dot()
      const {
    return desired_centroidal_momentum_dot_;
  }

  // Setters
  inline double& mutable_w_basis_reg() { return w_basis_reg_; }
  inline std::map<std::string, ContactInformation>&
  mutable_contact_information() {
    return contact_info_;
  }
  inline std::map<std::string, DesiredBodyMotion>&
  mutable_desired_body_motions() {
    return desired_body_motions_;
  }
  inline DesiredDoFMotions& mutable_desired_dof_motions() {
    return desired_dof_motions_;
  }
  inline DesiredCentroidalMomentumDot&
  mutable_desired_centroidal_momentum_dot() {
    return desired_centroidal_momentum_dot_;
  }

 private:
  // Contact information
  std::map<std::string, ContactInformation> contact_info_;

  // Desired task space accelerations for specific bodies
  std::map<std::string, DesiredBodyMotion> desired_body_motions_;

  // Desired joint accelerations
  DesiredDoFMotions desired_dof_motions_;

  // Desired centroidal momentum change (change of overall linear and angular
  // momentum)
  DesiredCentroidalMomentumDot desired_centroidal_momentum_dot_;

  // Weight for regularizing basis vectors
  double w_basis_reg_;
};

std::ostream& operator<<(std::ostream& out, const QPInput& input);

/**
 * This class holds the contact force / wrench related information, and works
 * closely with ContactInformation.
 */
class ResolvedContact {
 public:
  /**
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   */
  explicit ResolvedContact(const RigidBody<double>& body) : body_(&body) {}

  bool is_valid() const {
    if (!basis_.allFinite() || basis_.minCoeff() < 0) {
      return false;
    }
    if (basis_.size() != num_basis_per_contact_point_ * num_contact_points()) {
      return false;
    }
    if (!point_forces_.allFinite()) {
      return false;
    }
    if (!contact_points_.allFinite()) {
      return false;
    }
    if (point_forces_.cols() != contact_points_.cols()) {
      return false;
    }
    if (!equivalent_wrench_.allFinite()) {
      return false;
    }
    if (!reference_point_.allFinite()) {
      return false;
    }
    return true;
  }

  bool operator==(const ResolvedContact& other) const {
    if (body_ != other.body_) {
      return false;
    }
    if (num_basis_per_contact_point_ != other.num_basis_per_contact_point_) {
      return false;
    }
    if (!basis_.isApprox(other.basis_)) {
      return false;
    }
    if (!point_forces_.isApprox(other.point_forces_)) {
      return false;
    }
    if (!contact_points_.isApprox(other.contact_points_)) {
      return false;
    }
    if (!equivalent_wrench_.isApprox(other.equivalent_wrench_)) {
      return false;
    }
    if (!reference_point_.isApprox(other.reference_point_)) {
      return false;
    }
    return true;
  }

  // Getters
  inline const RigidBody<double>& body() const { return *body_; }
  inline const std::string& body_name() const { return body_->get_name(); }
  inline const VectorX<double>& basis() const { return basis_; }
  inline const Matrix3X<double>& point_forces() const { return point_forces_; }
  inline const Matrix3X<double>& contact_points() const {
    return contact_points_;
  }
  inline const Vector6<double>& equivalent_wrench() const {
    return equivalent_wrench_;
  }
  inline const Vector3<double>& reference_point() const {
    return reference_point_;
  }
  inline const Vector6<double>& body_acceleration() const {
    return body_acceleration_;
  }
  inline int num_contact_points() const { return contact_points_.cols(); }
  inline int num_basis_per_contact_point() const {
    return num_basis_per_contact_point_;
  }

  // Setters
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }
  inline VectorX<double>& mutable_basis() { return basis_; }
  inline Matrix3X<double>& mutable_point_forces() { return point_forces_; }
  inline Matrix3X<double>& mutable_contact_points() { return contact_points_; }
  inline Vector6<double>& mutable_equivalent_wrench() {
    return equivalent_wrench_;
  }
  inline Vector3<double>& mutable_reference_point() { return reference_point_; }
  inline Vector6<double>& mutable_body_acceleration() {
    return body_acceleration_;
  }
  inline int& mutable_num_basis_per_contact_point() {
    return num_basis_per_contact_point_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBody<double>* body_;

  int num_basis_per_contact_point_;

  // Stacked scalars for all the basis vectors.
  VectorX<double> basis_;

  // Point contact forces in the world frame.
  Matrix3X<double> point_forces_;

  // Contact points in the world frame.
  Matrix3X<double> contact_points_;

  // The equivalent wrench of all the point forces, w.r.t a frame that has
  // the same orientation as the world frame, but located at reference_point.
  Vector6<double> equivalent_wrench_;

  // Body acceleration w.r.t a frame that has the same orientation as the
  // world frame, but located at body's origin.
  Vector6<double> body_acceleration_;

  // Reference point in the world frame for the equivalent wrench.
  Vector3<double> reference_point_;
};

std::ostream& operator<<(std::ostream& out, const ResolvedContact& contact);

/**
 * This class holds task space acceleration for a rigid body. The first three
 * are angular acceleration.
 */
class BodyAcceleration {
 public:
  /**
   * @param body Reference to a RigidBody, which must be valid through the
   * lifespan of this obejct.
   */
  explicit BodyAcceleration(const RigidBody<double>& body)
      : body_(&body), accelerations_(Vector6<double>::Zero()) {}

  inline bool is_valid() const { return accelerations_.allFinite(); }

  bool operator==(const BodyAcceleration& other) const {
    if (body_ != other.body_) {
      return false;
    }
    if (!accelerations_.isApprox(other.accelerations_)) {
      return false;
    }
    return true;
  }

  // Getters
  inline const RigidBody<double>& body() const { return *body_; }
  inline const std::string& body_name() const { return body_->get_name(); }
  inline const Vector6<double>& accelerations() const { return accelerations_; }

  // Setter
  inline Vector6<double>& mutable_accelerations() { return accelerations_; }
  inline void set_body(const RigidBody<double>& body) { body_ = &body; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  const RigidBody<double>* body_;
  Vector6<double> accelerations_;
};

std::ostream& operator<<(std::ostream& out, const BodyAcceleration& acc);

/**
 * Output of the QP inverse dynamics controller
 */
class QPOutput {
 public:
  explicit QPOutput(const RigidBodyTree<double>& r) {
    dof_names_.resize(r.get_num_velocities());
    for (int i = 0; i < r.get_num_velocities(); ++i) {
      // strip out the "dot" part from name
      dof_names_[i] =
          r.get_velocity_name(i).substr(0, r.get_velocity_name(i).size() - 3);
    }
    vd_.resize(r.get_num_velocities());
    dof_torques_.resize(r.get_num_velocities());
  }

  bool is_valid(int num_vd) const {
    if (vd_.size() != static_cast<int>(dof_names_.size()) ||
        vd_.size() != num_vd || vd_.size() != dof_torques_.size()) {
      return false;
    }
    if (!comdd_.allFinite() || !centroidal_momentum_dot_.allFinite() ||
        !vd_.allFinite()) {
      return false;
    }
    for (const auto& body_acceleration_pair : body_accelerations_) {
      if (!body_acceleration_pair.second.is_valid()) {
        return false;
      }
    }
    for (const auto& contact_pair : resolved_contacts_) {
      if (!contact_pair.second.is_valid()) {
        return false;
      }
    }
    return true;
  }

  // Getters
  inline const std::vector<std::string>& dof_names() const {
    return dof_names_;
  }
  inline const std::string& dof_name(int idx) const {
    return dof_names_.at(idx);
  }
  inline const Vector3<double>& comdd() const { return comdd_; }
  inline const Vector6<double>& centroidal_momentum_dot() const {
    return centroidal_momentum_dot_;
  }
  inline const VectorX<double>& vd() const { return vd_; }
  inline const std::map<std::string, BodyAcceleration>& body_accelerations()
      const {
    return body_accelerations_;
  }
  inline const VectorX<double>& dof_torques() const { return dof_torques_; }
  inline const std::map<std::string, ResolvedContact>& resolved_contacts()
      const {
    return resolved_contacts_;
  }
  inline const std::vector<std::pair<std::string, double>>& costs() const {
    return costs_;
  }
  inline const std::pair<std::string, double>& costs(size_t i) const {
    return costs_.at(i);
  }

  // Setters
  inline Vector3<double>& mutable_comdd() { return comdd_; }
  inline Vector6<double>& mutable_centroidal_momentum_dot() {
    return centroidal_momentum_dot_;
  }
  inline VectorX<double>& mutable_vd() { return vd_; }
  inline std::map<std::string, BodyAcceleration>& mutable_body_accelerations() {
    return body_accelerations_;
  }
  inline std::map<std::string, ResolvedContact>& mutable_resolved_contacts() {
    return resolved_contacts_;
  }
  inline VectorX<double>& mutable_dof_torques() { return dof_torques_; }

  inline std::vector<std::pair<std::string, double>>& mutable_costs() {
    return costs_;
  }
  inline std::pair<std::string, double>& mutable_cost(size_t i) {
    return costs_.at(i);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Names for each generalized coordinate.
  std::vector<std::string> dof_names_;

  // Computed accelerations for the center of mass in the world frame
  Vector3<double> comdd_;
  // Computed centroidal momentum dot in the world frame: [angular; linear]
  // The linear part equals comdd_ * mass.
  Vector6<double> centroidal_momentum_dot_;
  // Computed generalized coordinate accelerations
  VectorX<double> vd_;
  // Computed joint torque, in dof's order, not robot.actuator's order.
  VectorX<double> dof_torques_;

  // Computed contact related information such as point contact forces
  std::map<std::string, ResolvedContact> resolved_contacts_;

  // Tracked body motion
  std::map<std::string, BodyAcceleration> body_accelerations_;

  // Pair of the name of cost term and cost value (only the quadratic and linear
  // term, no constant term).
  std::vector<std::pair<std::string, double>> costs_;
};

std::ostream& operator<<(std::ostream& out, const QPOutput& output);

class QPController {
 public:
  /**
   * Computes the generalized acceleration, joint torque and contact wrenches
   * that best tracks the input given the current robot configuration.
   * @param robot_status Robot configuration
   * @param input Desired motions and objectives specified by a higher level
   * controller
   * @param output Container for outputs
   * @return 0 if successful. < 0 if error.
   */
  int Control(const HumanoidStatus& robot_status, const QPInput& input,
              QPOutput* output);

  static const double kUpperBoundForContactBasis;

 private:
  // These are temporary matrices and vectors used by the controller.
  MatrixX<double> tmp_vd_mat_;
  VectorX<double> tmp_vd_vec_;
  MatrixX<double> basis_reg_mat_;
  VectorX<double> basis_reg_vec_;

  MatrixX<double> stacked_contact_jacobians_;
  VectorX<double> stacked_contact_jacobians_dot_times_v_;
  VectorX<double> stacked_contact_velocities_;
  MatrixX<double> basis_to_force_matrix_;

  MatrixX<double> torque_linear_;
  VectorX<double> torque_constant_;
  MatrixX<double> dynamics_linear_;
  VectorX<double> dynamics_constant_;

  MatrixX<double> inequality_linear_;
  VectorX<double> inequality_upper_bound_;
  VectorX<double> inequality_lower_bound_;

  MatrixX<double> JB_;
  VectorX<double> point_forces_;

  MatrixX<double> mass_matrix_;
  VectorX<double> dynamics_bias_;

  MatrixX<double> J_com_;
  VectorX<double> J_dot_times_v_com_;
  MatrixX<double> centroidal_momentum_matrix_;
  VectorX<double> centroidal_momentum_matrix_dot_times_v_;

  VectorX<double> solution_;

  std::vector<MatrixX<double>> body_J_;
  std::vector<VectorX<double>> body_Jdv_;

  // These determines the size of the QP. These are set in ResizeQP
  int num_contact_body_{0};
  int num_vd_{0};
  int num_point_force_{0};
  int num_basis_{0};
  int num_torque_{0};
  int num_variable_{0};
  // One cost / eqaulity constraint term per body motion.
  // For each dimension (row) of the desired body motion, it can be treated
  // as a cost term (Soft), skipped (SKip) or as an equality constraint (Hard)
  // depending on the given constraint type. If it's a Soft constraint, the
  // corresponding weight needs to be positive.
  // E.g. if pelvis's desired body motion has type
  // (Soft, Soft, ,Skip, Skip, Hard, Hard),
  // num_body_motion_as_cost_ and num_body_motion_as_eq_ are both incremented
  // by 1.
  //
  // For soft constraints, the cost term is:
  // weight * (0.5 * vd^T * J(1:2,:)^T * J(1:2,:) * vd
  //           + vd^T * J(1:2,:)^T * (Jdv(1:2,:) - pelvdd_d(1:2)))
  // The equality constraint term is:
  // J(5:6,:) * vd + Jdv(5:6,:) = pelvdd_d(5:6)
  int num_body_motion_as_cost_{0};
  int num_body_motion_as_eq_{0};
  // Same as for body_motiom, replace J with the identity matrix.
  int num_dof_motion_as_cost_{0};
  int num_dof_motion_as_eq_{0};
  int num_cen_mom_dot_as_cost_{0};
  int num_cen_mom_dot_as_eq_{0};
  int num_contact_as_cost_{0};
  int num_contact_as_eq_{0};

  // prog_ is only allocated in ResizeQP, Control only updates the appropriate
  // matrices / vectors.
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;
  drake::solvers::GurobiSolver solver_;
  drake::solvers::DecisionVariableVectorX basis_;
  drake::solvers::DecisionVariableVectorX vd_;

  // pointers to different cost / constraint terms inside prog_
  drake::solvers::LinearEqualityConstraint* eq_dynamics_{nullptr};
  // TODO(siyuan.feng): Switch to cost for contact_constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> eq_contacts_;
  std::vector<drake::solvers::LinearEqualityConstraint*> eq_body_motion_;
  drake::solvers::LinearEqualityConstraint* eq_dof_motion_{nullptr};
  drake::solvers::LinearEqualityConstraint* eq_cen_mom_dot_{nullptr};

  drake::solvers::LinearConstraint* ineq_contact_wrench_{nullptr};
  drake::solvers::LinearConstraint* ineq_torque_limit_{nullptr};

  std::vector<drake::solvers::QuadraticConstraint*> cost_contacts_;
  drake::solvers::QuadraticConstraint* cost_cen_mom_dot_{nullptr};
  std::vector<drake::solvers::QuadraticConstraint*> cost_body_motion_;
  drake::solvers::QuadraticConstraint* cost_dof_motion_{nullptr};

  drake::solvers::QuadraticConstraint* cost_basis_reg_{nullptr};

  /**
   * Resize the QP. This resizes the temporary matrices. It also reinitializes
   * prog_ to the correct size, so that Control only updates the
   * matrices and vectors in prog_ instead of making a new one on every call.
   * Size change typically happens when contact state changes (making / breaking
   * contacts).
   * @param robot Model
   * @param input input to the QP
   */
  void ResizeQP(const RigidBodyTree<double>& robot, const QPInput& input);

  template <typename DerivedA, typename DerivedB>
  void AddAsConstraints(const Eigen::MatrixBase<DerivedA>& A,
                        const Eigen::MatrixBase<DerivedB>& b,
                        const std::list<int>& idx,
                        drake::solvers::LinearEqualityConstraint* eq) {
    if (idx.empty()) return;
    if (A.rows() != b.rows() || A.rows() > tmp_vd_mat_.rows() ||
        b.cols() != 1 || A.cols() != tmp_vd_mat_.cols()) {
      throw std::runtime_error("Invalid input dimension.");
    }

    int row_ctr = 0;
    for (int d : idx) {
      tmp_vd_mat_.row(row_ctr) = A.row(d);
      tmp_vd_vec_.row(row_ctr) = b.row(d);
      row_ctr++;
    }
    eq->UpdateConstraint(tmp_vd_mat_.topRows(row_ctr),
                         tmp_vd_vec_.head(row_ctr));
  }

  template <typename DerivedA, typename DerivedB, typename DerivedW>
  void AddAsCosts(const Eigen::MatrixBase<DerivedA>& A,
                  const Eigen::MatrixBase<DerivedB>& b,
                  const Eigen::MatrixBase<DerivedW>& weights,
                  const std::list<int>& idx,
                  drake::solvers::QuadraticConstraint* cost) {
    if (idx.empty()) return;
    if (A.rows() != b.rows() || A.rows() != weights.rows() ||
        A.rows() > tmp_vd_mat_.rows() || b.cols() != 1 || weights.cols() != 1 ||
        A.cols() != tmp_vd_mat_.cols()) {
      throw std::runtime_error("Invalid input dimension.");
    }

    tmp_vd_mat_.setZero();
    tmp_vd_vec_.setZero();
    for (int d : idx) {
      double weight = weights[d];
      tmp_vd_mat_ += weight * A.row(d).transpose() * A.row(d);
      tmp_vd_vec_ += weight * A.row(d).transpose() * b.row(d);
    }
    cost->UpdateQuadraticAndLinearTerms(tmp_vd_mat_, tmp_vd_vec_);
  }

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
};

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
