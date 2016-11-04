#pragma once

#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "drake/examples/QPInverseDynamicsForHumanoids/humanoid_status.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

/**
 * Enum class for constraint types.
 * Hard: will be enforced by equality constraints.
 * Soft: will be enforced by cost functions.
 */
enum class ConstraintType { Hard = -1, Skip = 0, Soft = 1 };

inline std::ostream& operator<<(std::ostream& out, const ConstraintType& type) {
  out << "constraint type: ";
  switch (type) {
    case ConstraintType::Hard:
      out << "Hard\n";
      break;
    case ConstraintType::Skip:
      out << "Skip\n";
      break;
    case ConstraintType::Soft:
      out << "Soft\n";
      break;
  }
  return out;
}

/**
 * Base class for specifying various desired objectives.
 * The objectives can be ignored, set as equality constraints or optimized as
 * cost terms depending on the specified types.
 * For cost terms, a positive weight needs to be specified.
 */
class ConstrainedValues {
 private:
  std::vector<ConstraintType> constraint_types_;
  Eigen::VectorXd weights_;
  Eigen::VectorXd values_;

 public:
  ConstrainedValues() {}

  explicit ConstrainedValues(int dim)
      : constraint_types_(dim, ConstraintType::Skip),
        weights_(Eigen::VectorXd::Zero(dim)),
        values_(Eigen::VectorXd::Zero(dim)) {}

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

  bool is_valid(int dim) const {
    bool ret = weights_.size() == values_.size();
    ret &= static_cast<int>(constraint_types_.size()) == weights_.size();
    ret &= weights_.size() == dim;
    if (ret) {
      for (int i = 0; i < dim; ++i) {
        if (constraint_types_[i] == ConstraintType::Soft)
          ret &= weights_[i] > 0;
      }
    }
    ret &= weights_.allFinite();
    ret &= values_.allFinite();
    return ret;
  }

  // Getters
  inline int size() const { return values_.size(); }
  inline const Eigen::VectorXd& weights() const { return weights_; }
  inline const Eigen::VectorXd& values() const { return values_; }
  inline const std::vector<ConstraintType>& constraint_types() const {
    return constraint_types_;
  }

  inline double value(int i) const { return values_[i]; }
  inline double weight(int i) const { return weights_[i]; }
  inline ConstraintType constraint_type(int i) const {
    return constraint_types_.at(i);
  }

  // Setters
  inline Eigen::VectorXd& mutable_weights() { return weights_; }
  inline Eigen::VectorXd& mutable_values() { return values_; }
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
  ContactInformation(const RigidBody& body,
                     int num_basis = kDefaultNumBasisPerContactPoint)
      : body_(&body),
        num_basis_per_contact_point_(num_basis),
        acceleration_constraint_type_(ConstraintType::Hard) {
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
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    Eigen::MatrixXd basis(
        3 * contact_points_.size(),
        num_basis_per_contact_point_ * contact_points_.size());
    Eigen::Matrix3d body_rot =
        robot.relativeTransform(cache, 0, body_->get_body_index()).linear();
    basis.setZero();

    Eigen::Vector3d t1, t2, tangent_vec, base;
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

    for (int i = 0; i < static_cast<int>(contact_points_.size()); ++i) {
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
      const Eigen::Vector3d& offset,
      std::vector<Eigen::Vector3d>* contact_points,
      Eigen::Vector3d* reference_point) const {
    *reference_point =
        robot.transformPoints(cache, offset, body_->get_body_index(), 0);
    contact_points->resize(contact_points_.size());
    for (size_t i = 0; i < contact_points_.size(); ++i) {
      (*contact_points)[i] = robot.transformPoints(cache, contact_points_[i],
                                                   body_->get_body_index(), 0);
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
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    Eigen::MatrixXd J(3 * contact_points_.size(), robot.get_num_velocities());
    for (size_t i = 0; i < contact_points_.size(); ++i) {
      J.block(3 * i, 0, 3, robot.get_num_velocities()) =
          GetTaskSpaceJacobian(robot, cache, *body_, contact_points_[i])
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
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    Eigen::VectorXd Jdv(3 * contact_points_.size());
    for (size_t i = 0; i < contact_points_.size(); ++i) {
      Jdv.segment<3>(3 * i) =
          GetTaskSpaceJacobianDotTimesV(robot, cache, *body_,
                                        contact_points_[i]).bottomRows<3>();
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
  Eigen::VectorXd ComputeLinearVelocityAtContactPoints(
      const RigidBodyTree<double>& robot,
      const KinematicsCache<double>& cache) const {
    Eigen::VectorXd vel(3 * contact_points_.size());
    for (size_t i = 0; i < contact_points_.size(); ++i) {
      vel.segment<3>(3 * i) =
          GetTaskSpaceVel(robot, cache, *body_, contact_points_[i])
              .bottomRows<3>();
    }
    return vel;
  }

  bool is_valid() const {
    bool ret =
        std::abs(normal_.norm() - 1) < Eigen::NumTraits<double>::epsilon();
    for (const Eigen::Vector3d& pt : contact_points_) ret &= pt.allFinite();
    ret &= mu_ >= 0;
    ret &= num_basis_per_contact_point_ >= 3;
    if (acceleration_constraint_type_ == ConstraintType::Soft)
      ret &= weight_ > 0;
    // Can't skip contact constraints
    ret &= acceleration_constraint_type_ != ConstraintType::Skip;
    // Can't have a minus stabilizing velocity gain.
    ret &= Kd_ >= 0;
    return ret;
  }

  inline const std::string& name() const { return body_->get_name(); }
  inline int num_contact_points() const {
    return static_cast<int>(contact_points_.size());
  }
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
  inline const std::vector<Eigen::Vector3d>& contact_points() const {
    return contact_points_;
  }
  inline const Eigen::Vector3d& normal() const { return normal_; }
  inline const RigidBody& body() const { return *body_; }
  inline int num_basis_per_contact_point() const {
    return num_basis_per_contact_point_;
  }

  // Setters
  inline std::vector<Eigen::Vector3d>& mutable_contact_points() {
    return contact_points_;
  }
  inline double& mutable_mu() { return mu_; }
  inline double& mutable_weight() { return weight_; }
  inline double& mutable_Kd() { return Kd_; }
  inline ConstraintType& mutable_acceleration_constraint_type() {
    return acceleration_constraint_type_;
  }
  inline Eigen::Vector3d& mutable_normal() { return normal_; }
  inline int& mutable_num_basis_per_contact_point() {
    return num_basis_per_contact_point_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBody* body_;
  // Offsets of the contact point specified in the body frame.
  std::vector<Eigen::Vector3d> contact_points_;

  // TODO(siyuan.feng): Normal is currently assumed to be the same for all
  // the contact points.
  // Contact normal specified in the body frame.
  Eigen::Vector3d normal_;

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

inline std::ostream& operator<<(std::ostream& out,
                                const ContactInformation& contact) {
  out << "contact: " << contact.name() << std::endl;
  out << "contact points in body frame: " << std::endl;
  for (size_t j = 0; j < contact.contact_points().size(); ++j)
    out << contact.contact_points()[j].transpose() << std::endl;
  out << "normal in body frame: " << contact.normal().transpose() << std::endl;
  out << "mu: " << contact.mu() << std::endl;
  out << contact.acceleration_constraint_type();
  out << "weight: " << contact.weight() << std::endl;
  out << "Kd: " << contact.Kd() << std::endl;
  return out;
}

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
  explicit DesiredBodyMotion(const RigidBody& body)
      : ConstrainedValues(6), body_(&body), control_during_contact_(false) {}

  inline bool is_valid() const {
    return ConstrainedValues::is_valid(kTwistSize);
  }

  inline std::string get_row_name(int i) const {
    static const std::string row_name[kTwistSize] = {"[WX]", "[WY]", "[WZ]",
                                                     "[X]",  "[Y]",  "[Z]"};
    if (i < 0 || i >= kTwistSize)
      throw std::runtime_error("index must be within [0, 5]");
    return row_name[i];
  }

  // Getters
  inline const RigidBody& body() const { return *body_; }
  inline const std::string& name() const { return body_->get_name(); }
  inline bool control_during_contact() const { return control_during_contact_; }

  // Setters
  inline bool& mutable_control_during_contact() {
    return control_during_contact_;
  }

 private:
  const RigidBody* body_;

  // TODO(siyuan.feng) Actually implement this in the qp controller.
  bool control_during_contact_;
};

inline std::ostream& operator<<(std::ostream& out,
                                const DesiredBodyMotion& input) {
  for (int i = 0; i < kTwistSize; ++i) {
    out << "desired " << input.name() << input.get_row_name(i)
        << " acc: " << input.values()[i] << " weight: " << input.weights()[i]
        << " " << input.constraint_types()[i];
  }
  return out;
}

/**
 * A wrapper class specifying desired joint motion (acceleration) and their
 * corresponding weights for the QP.
 *
 * The desired acceleration can be skipped, enforced as equality constraints
 * or optimized as a cost term depending on the constraint type.
 *
 * TODO: (siyuan.feng) Expand this to have policies (controllers).
 */
class DesiredJointMotions : public ConstrainedValues {
 public:
  DesiredJointMotions() {}
  explicit DesiredJointMotions(const std::vector<std::string>& names)
      : ConstrainedValues(static_cast<int>(names.size())), names_(names) {}

  bool is_valid(int dim) const {
    bool ret = static_cast<int>(names_.size()) == dim;
    ret &= ConstrainedValues::is_valid(dim);
    return ret;
  }

  // Getters
  inline const std::vector<std::string>& names() const { return names_; }
  inline const std::string& name(int i) const { return names_.at(i); }

 private:
  std::vector<std::string> names_;
};

inline std::ostream& operator<<(std::ostream& out,
                                const DesiredJointMotions& input) {
  for (int i = 0; i < input.size(); ++i) {
    out << "desired " << input.name(i) << " acc: " << input.value(i)
        << " weight: " << input.weight(i) << " " << input.constraint_type(i);
  }
  return out;
}

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
class DesiredCentroidalMomentumChange : public ConstrainedValues {
 public:
  DesiredCentroidalMomentumChange() : ConstrainedValues(kTwistSize) {}

  bool is_valid() const { return ConstrainedValues::is_valid(kTwistSize); }

  inline std::string get_row_name(int i) const {
    static const std::string row_name[6] = {"AngMom[X]", "AngMom[Y]",
                                            "AngMom[Z]", "LinMom[X]",
                                            "LinMom[Y]", "LinMom[Z]"};
    if (i < 0 || i >= kTwistSize)
      throw std::runtime_error("index must be within [0, 5]");
    return row_name[i];
  }
};

inline std::ostream& operator<<(std::ostream& out,
                                const DesiredCentroidalMomentumChange& input) {
  for (int i = 0; i < 6; ++i) {
    out << "desired " << input.get_row_name(i) << " change: " << input.value(i)
        << " weight: " << input.weight(i) << " " << input.constraint_type(i);
  }
  return out;
}

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
    desired_joint_motions_ = DesiredJointMotions(names);
  }

  /**
   * Checks validity of this QPInput.
   * @param num_vd Dimension of acceleration in the generalized coordinates.
   * @return true if this is valid.
   */
  bool is_valid(int num_vd) const {
    int valid = num_vd == desired_joint_motions_.size();
    valid &= desired_joint_motions_.is_valid(num_vd);
    for (auto it = desired_body_motions_.begin();
         it != desired_body_motions_.end(); ++it) {
      valid &= it->second.is_valid();
    }

    for (const ContactInformation& contact : contact_info_)
      valid &= contact.is_valid();

    valid &= desired_centroidal_momentum_dot_.is_valid();

    valid &= std::isfinite(w_basis_reg_);
    // Regularization weight needs to be positive.
    valid &= w_basis_reg_ > 0;

    return valid;
  }

  // Getters
  inline double w_basis_reg() const { return w_basis_reg_; }
  inline const std::string& coord_name(size_t idx) const {
    return desired_joint_motions_.name(idx);
  }
  inline const std::list<ContactInformation>& contact_info() const {
    return contact_info_;
  }
  inline const std::map<std::string, DesiredBodyMotion>& desired_body_motions()
      const {
    return desired_body_motions_;
  }
  inline const DesiredJointMotions& desired_joint_motions() const {
    return desired_joint_motions_;
  }
  inline const DesiredCentroidalMomentumChange&
  desired_centroidal_momentum_dot() const {
    return desired_centroidal_momentum_dot_;
  }

  // Setters
  inline double& mutable_w_basis_reg() { return w_basis_reg_; }
  inline std::list<ContactInformation>& mutable_contact_info() {
    return contact_info_;
  }
  inline std::map<std::string, DesiredBodyMotion>&
  mutable_desired_body_motions() {
    return desired_body_motions_;
  }
  inline DesiredJointMotions& mutable_desired_joint_motions() {
    return desired_joint_motions_;
  }
  inline DesiredCentroidalMomentumChange&
  mutable_desired_centroidal_momentum_dot() {
    return desired_centroidal_momentum_dot_;
  }

 private:
  // Contact information
  std::list<ContactInformation> contact_info_;

  // Desired task space accelerations for specific bodies
  std::map<std::string, DesiredBodyMotion> desired_body_motions_;

  // Desired joint accelerations
  DesiredJointMotions desired_joint_motions_;

  // Desired centroidal momentum change (change of overall linear and angular
  // momentum)
  DesiredCentroidalMomentumChange desired_centroidal_momentum_dot_;

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
  explicit ResolvedContact(const RigidBody& body) : body_(&body) {}

  bool is_valid() const {
    bool ret = basis_.allFinite() && basis_.minCoeff() >= 0;
    for (const Eigen::Vector3d& ptf : point_forces_) ret &= ptf.allFinite();
    for (const Eigen::Vector3d& pt : contact_points_) ret &= pt.allFinite();
    ret &= equivalent_wrench_.allFinite();
    ret &= reference_point_.allFinite();
    return ret;
  }

  // Getters
  inline const RigidBody& body() const { return *body_; }
  inline const std::string& name() const { return body_->get_name(); }
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  const RigidBody* body_;
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
};

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
  explicit BodyAcceleration(const RigidBody& body)
      : body_(&body), acceleration_(Eigen::Vector6d::Zero()) {}

  inline bool is_valid() const { return acceleration_.allFinite(); }

  // Getters
  inline const RigidBody& body() const { return *body_; }
  inline const std::string& name() const { return body_->get_name(); }
  inline const Eigen::Vector6d& acceleration() const { return acceleration_; }

  // Setter
  inline Eigen::Vector6d& mutable_acceleration() { return acceleration_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  const RigidBody* body_;
  Eigen::Vector6d acceleration_;
};

/**
 * Output of the QP inverse dynamics controller
 */
class QPOutput {
 public:
  explicit QPOutput(const RigidBodyTree<double>& r) {
    coord_names_.resize(r.get_num_velocities());
    for (int i = 0; i < r.get_num_velocities(); ++i) {
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
    ret &= centroidal_momentum_dot_.allFinite();
    ret &= vd_.allFinite();
    ret &= joint_torque_.allFinite();

    for (const BodyAcceleration& body_acceleration : body_accelerations_) {
      ret &= body_acceleration.is_valid();
    }

    for (const ResolvedContact& contact : resolved_contacts_) {
      ret &= contact.is_valid();
    }

    return ret;
  }

  // Getters
  inline const std::string& coord_name(int idx) const {
    return coord_names_.at(idx);
  }
  inline const Eigen::Vector3d& comdd() const { return comdd_; }
  inline const Eigen::Vector6d& centroidal_momentum_dot() const {
    return centroidal_momentum_dot_;
  }
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
  inline Eigen::Vector6d& mutable_centroidal_momentum_dot() {
    return centroidal_momentum_dot_;
  }
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

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Names for each generalized coordinate.
  std::vector<std::string> coord_names_;

  // Tracked body motion
  std::vector<BodyAcceleration> body_accelerations_;

  // Computed accelerations for the center of mass in the world frame
  Eigen::Vector3d comdd_;
  // Computed centroidal momentum dot in the world frame: [angular; linear]
  // The linear part equals comdd_ * mass.
  Eigen::Vector6d centroidal_momentum_dot_;
  // Computed generalized coordinate accelerations
  Eigen::VectorXd vd_;
  // Computed joint torque
  Eigen::VectorXd joint_torque_;

  // Computed contact related information such as point contact forces
  std::vector<ResolvedContact> resolved_contacts_;

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
  Eigen::MatrixXd tmp_vd_mat_;
  Eigen::VectorXd tmp_vd_vec_;
  Eigen::MatrixXd basis_reg_mat_;
  Eigen::VectorXd basis_reg_vec_;

  Eigen::MatrixXd stacked_contact_jacobians_;
  Eigen::VectorXd stacked_contact_jacobians_dot_times_v_;
  Eigen::VectorXd stacked_contact_velocities_;
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

  Eigen::VectorXd solution_;

  std::vector<Eigen::MatrixXd> body_J_;
  std::vector<Eigen::VectorXd> body_Jdv_;

  // These determines the size of the QP. These are set in ResizeQP
  int num_contact_body_;
  int num_vd_;
  int num_point_force_;
  int num_basis_;
  int num_torque_;
  int num_variable_;
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
  int num_body_motion_as_cost_;
  int num_body_motion_as_eq_;
  // Same as for body_motiom, replace J with the identity matrix.
  int num_joint_motion_as_cost_;
  int num_joint_motion_as_eq_;
  int num_cen_mom_dot_as_cost_;
  int num_cen_mom_dot_as_eq_;
  int num_contact_as_cost_;
  int num_contact_as_eq_;

  // prog_ is only allocated in ResizeQP, Control only updates the appropriate
  // matrices / vectors.
  drake::solvers::MathematicalProgram prog_;
  drake::solvers::GurobiSolver solver_;

  // pointers to different cost / constraint terms inside prog_
  drake::solvers::LinearEqualityConstraint* eq_dynamics_{nullptr};
  // TODO(siyuan.feng): Switch to cost for contact_constraints
  std::vector<drake::solvers::LinearEqualityConstraint*> eq_contacts_;
  std::vector<drake::solvers::LinearEqualityConstraint*> eq_body_motion_;
  drake::solvers::LinearEqualityConstraint* eq_joint_motion_{nullptr};
  drake::solvers::LinearEqualityConstraint* eq_cen_mom_dot_{nullptr};

  drake::solvers::LinearConstraint* ineq_contact_wrench_{nullptr};
  drake::solvers::LinearConstraint* ineq_torque_limit_{nullptr};

  std::vector<drake::solvers::QuadraticConstraint*> cost_contacts_;
  drake::solvers::QuadraticConstraint* cost_cen_mom_dot_{nullptr};
  std::vector<drake::solvers::QuadraticConstraint*> cost_body_motion_;
  drake::solvers::QuadraticConstraint* cost_joint_motion_{nullptr};

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
