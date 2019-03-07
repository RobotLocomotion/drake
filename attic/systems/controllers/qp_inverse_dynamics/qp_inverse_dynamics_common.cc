#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics_common.h"

#include "drake/math/cross_product.h"

namespace drake {
namespace systems {
namespace controllers {
namespace qp_inverse_dynamics {

std::ostream& operator<<(std::ostream& out, const ConstraintType& type) {
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

void ConstrainedValues::resize(int dim) {
  constraint_types_.resize(dim);
  weights_.resize(dim);
  values_.resize(dim);
}

void ConstrainedValues::SetAllConstraintTypesBasedOnWeights() {
  for (int i = 0; i < weights_.size(); ++i) {
    if (weights_[i] > 0) {
      constraint_types_.at(i) = ConstraintType::Soft;
    } else if (weights_[i] < 0) {
      constraint_types_.at(i) = ConstraintType::Hard;
    } else {
      constraint_types_.at(i) = ConstraintType::Skip;
    }
  }
}

std::list<int> ConstrainedValues::GetConstraintTypeIndices(
    ConstraintType type) const {
  std::list<int> ret;
  for (int i = 0; i < static_cast<int>(constraint_types_.size()); ++i) {
    if (constraint_types_[i] == type) ret.push_back(i);
  }

  return ret;
}

void ConstrainedValues::SetConstraintType(const std::list<int>& indices,
                                          ConstraintType type) {
  for (int i : indices) {
    if (i < static_cast<int>(constraint_types_.size()) && i >= 0)
      constraint_types_[i] = type;
  }
}

void ConstrainedValues::SetAllConstraintType(ConstraintType type) {
  for (size_t i = 0; i < constraint_types_.size(); ++i) {
    constraint_types_[i] = type;
  }
}

bool ConstrainedValues::is_valid(int dim) const {
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

bool ConstrainedValues::operator==(const ConstrainedValues& other) const {
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

ContactInformation::ContactInformation(const RigidBody<double>& body,
                                       int num_basis)
    : body_(&body),
      num_basis_per_contact_point_(num_basis),
      acceleration_constraint_type_(ConstraintType::Hard) {
  normal_ = Vector3<double>(0, 0, 1);
  mu_ = 1;
  if (num_basis_per_contact_point_ < 3)
    throw std::runtime_error("Number of basis per contact point must be >= 3.");
}

MatrixX<double> ContactInformation::ComputeBasisMatrix(
    const RigidBodyTree<double>& robot,
    const KinematicsCache<double>& cache) const {
  MatrixX<double> basis(3 * contact_points_.cols(),
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

void ContactInformation::ComputeContactPointsAndWrenchReferencePoint(
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

MatrixX<double> ContactInformation::ComputeWrenchMatrix(
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

MatrixX<double> ContactInformation::ComputeJacobianAtContactPoints(
    const RigidBodyTree<double>& robot,
    const KinematicsCache<double>& cache) const {
  MatrixX<double> J(3 * contact_points_.cols(), robot.get_num_velocities());
  Isometry3<double> offset(Isometry3<double>::Identity());
  for (int i = 0; i < contact_points_.cols(); ++i) {
    offset.translation() = contact_points_.col(i);
    J.block(3 * i, 0, 3, robot.get_num_velocities()) =
        robot
            .CalcFrameSpatialVelocityJacobianInWorldFrame(cache, *body_, offset)
            .bottomRows<3>();
  }
  return J;
}

VectorX<double> ContactInformation::ComputeJacobianDotTimesVAtContactPoints(
    const RigidBodyTree<double>& robot,
    const KinematicsCache<double>& cache) const {
  VectorX<double> Jdv(3 * contact_points_.cols());
  Isometry3<double> offset(Isometry3<double>::Identity());
  for (int i = 0; i < contact_points_.cols(); ++i) {
    offset.translation() = contact_points_.col(i);
    Jdv.segment<3>(3 * i) =
        robot
            .CalcFrameSpatialVelocityJacobianDotTimesVInWorldFrame(
                cache, *body_, offset)
            .bottomRows<3>();
  }
  return Jdv;
}

VectorX<double> ContactInformation::ComputeLinearVelocityAtContactPoints(
    const RigidBodyTree<double>& robot,
    const KinematicsCache<double>& cache) const {
  VectorX<double> vel(3 * contact_points_.cols());
  Isometry3<double> offset(Isometry3<double>::Identity());
  for (int i = 0; i < contact_points_.cols(); ++i) {
    offset.translation() = contact_points_.col(i);
    vel.segment<3>(3 * i) =
        robot.CalcFrameSpatialVelocityInWorldFrame(cache, *body_, offset)
            .bottomRows<3>();
  }
  return vel;
}

bool ContactInformation::is_valid() const {
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

bool ContactInformation::operator==(const ContactInformation& other) const {
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

std::ostream& operator<<(std::ostream& out, const ContactInformation& contact) {
  out << "contact: " << contact.body_name() << "\n";
  out << "contact points in body frame: "
      << "\n";
  for (int j = 0; j < contact.contact_points().cols(); ++j)
    out << contact.contact_points().col(j).transpose() << "\n";
  out << "normal in body frame: " << contact.normal().transpose() << "\n";
  out << "mu: " << contact.mu() << "\n";
  out << contact.acceleration_constraint_type();
  out << "weight: " << contact.weight() << "\n";
  out << "Kd: " << contact.Kd() << std::endl;
  return out;
}

std::string DesiredBodyMotion::get_row_name(int i) const {
  static constexpr const char* row_name[6] = {"[WX]", "[WY]", "[WZ]",
                                              "[X]",  "[Y]",  "[Z]"};
  if (i < 0 || i >= 6) throw std::runtime_error("index must be within [0, 5]");
  return std::string(row_name[i]);
}

bool DesiredBodyMotion::operator==(const DesiredBodyMotion& other) const {
  if (body_ != other.body_) {
    return false;
  }
  if (control_during_contact_ != other.control_during_contact_) {
    return false;
  }

  return this->ConstrainedValues::operator==(other);
}

std::ostream& operator<<(std::ostream& out, const DesiredBodyMotion& input) {
  for (int i = 0; i < 6; ++i) {
    out << "desired " << input.body_name() << input.get_row_name(i)
        << " acc: " << input.values()[i] << " weight: " << input.weights()[i]
        << " " << input.constraint_types()[i];
  }
  return out;
}

bool DesiredDofMotions::is_valid(int dim) const {
  if (static_cast<int>(dof_names_.size()) != dim) {
    return false;
  }
  return this->ConstrainedValues::is_valid(dim);
}

bool DesiredDofMotions::operator==(const DesiredDofMotions& other) const {
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

std::ostream& operator<<(std::ostream& out, const DesiredDofMotions& input) {
  for (int i = 0; i < input.size(); ++i) {
    out << "desired " << input.dof_name(i) << " acc: " << input.value(i)
        << " weight: " << input.weight(i) << " " << input.constraint_type(i);
  }
  return out;
}

std::string DesiredCentroidalMomentumDot::get_row_name(int i) const {
  static constexpr const char* row_name[6] = {"AngMom[X]", "AngMom[Y]",
                                              "AngMom[Z]", "LinMom[X]",
                                              "LinMom[Y]", "LinMom[Z]"};
  if (i < 0 || i >= 6) throw std::runtime_error("index must be within [0, 5]");
  return std::string(row_name[i]);
}

std::ostream& operator<<(std::ostream& out,
                         const DesiredCentroidalMomentumDot& input) {
  for (int i = 0; i < 6; ++i) {
    out << "desired " << input.get_row_name(i) << " change: " << input.value(i)
        << " weight: " << input.weight(i) << " " << input.constraint_type(i);
  }
  return out;
}

bool QpInput::is_valid(int num_vd) const {
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
  if (!std::isfinite(w_basis_reg_) || w_basis_reg_ < 0) {
    return false;
  }
  return true;
}

bool QpInput::operator==(const QpInput& other) const {
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
    return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& out, const QpInput& input) {
  out << "===============================================\n";
  out << "QpInput:\n";
  out << input.desired_centroidal_momentum_dot() << "\n";

  for (const auto& pair : input.desired_body_motions()) {
    out << pair.second << "\n";
  }

  out << input.desired_dof_motions() << "\n";

  out << "weight_basis_reg: " << input.w_basis_reg() << "\n";

  for (const auto& contact_pair : input.contact_information()) {
    out << contact_pair.second << std::endl;
  }

  return out;
}

bool ResolvedContact::is_valid() const {
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

bool ResolvedContact::operator==(const ResolvedContact& other) const {
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

std::ostream& operator<<(std::ostream& out, const ResolvedContact& contact) {
  out << "contact: " << contact.body_name() << "\n";
  out << "contact points in world frame: "
      << "\n";
  for (int j = 0; j < contact.contact_points().cols(); ++j)
    out << contact.contact_points().col(j).transpose() << "\n";
  out << "point forces in world frame: "
      << "\n";
  for (int j = 0; j < contact.point_forces().cols(); ++j)
    out << contact.point_forces().col(j).transpose() << "\n";
  out << "equivalent wrench in world aligned body frame: "
      << contact.equivalent_wrench().transpose() << "\n";
  out << "body acceleration: " << contact.body_acceleration().transpose()
      << "\n";
  out << "reference point in world frame: " << contact.reference_point()
      << std::endl;
  return out;
}

bool BodyAcceleration::operator==(const BodyAcceleration& other) const {
  if (body_ != other.body_) {
    return false;
  }
  if (!accelerations_.isApprox(other.accelerations_)) {
    return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& out, const BodyAcceleration& acc) {
  out << acc.body_name() << " acc: " << acc.accelerations().transpose()
      << std::endl;
  return out;
}

bool QpOutput::is_valid(int num_vd) const {
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

std::ostream& operator<<(std::ostream& out, const QpOutput& output) {
  out << "===============================================\n";
  out << "QpOutput:\n";
  out << "accelerations:\n";
  for (int i = 0; i < output.vd().size(); ++i) {
    out << output.dof_name(i) << ": " << output.vd()[i] << "\n";
  }

  out << "com acc: ";
  out << output.comdd().transpose() << "\n";

  for (const auto& body_motion_pair : output.body_accelerations()) {
    out << body_motion_pair.second;
  }

  out << "===============================================\n";
  for (const auto& contact_result_pair : output.resolved_contacts()) {
    out << contact_result_pair.second;
  }

  out << "===============================================\n";
  out << "torque:\n";
  for (int i = 0; i < output.dof_torques().size(); ++i) {
    out << output.dof_name(i) << ": " << output.dof_torques()[i] << "\n";
  }
  out << "===============================================\n";
  out << "costs:\n";
  for (const std::pair<std::string, double>& cost : output.costs()) {
    out << cost.first << ": " << cost.second << std::endl;
  }

  return out;
}

}  // namespace qp_inverse_dynamics
}  // namespace controllers
}  // namespace systems
}  // namespace drake
