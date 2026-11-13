#include "drake/multibody/inverse_kinematics/manipulability_cost.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>

#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/inverse_kinematics/kinematic_evaluator_utilities.h"

using drake::multibody::internal::PtrOrThrow;
using drake::multibody::internal::RefFromPtrOrThrow;
using drake::multibody::internal::UpdateContextConfiguration;

namespace drake {
namespace multibody {

ManipulabilityCost::ManipulabilityCost(
    const MultibodyPlant<double>* const plant, const Frame<double>& frameA,
    const Frame<double>& frameB, systems::Context<double>* plant_context,
    double c)
    : solvers::Cost(RefFromPtrOrThrow(plant).num_positions()),
      plant_(*plant),
      frameA_index_(frameA.index()),
      frameB_index_(frameB.index()),
      context_(PtrOrThrow(plant_context,
                          "ManipulabilityCost(): plant_context is nullptr")),
      c_(c) {
  DRAKE_THROW_UNLESS(c > 0, c);
  // A joint is on the path iff it moves exactly one of A's and B's bodies.
  const BodyIndex body_A = frameA.body().index();
  const BodyIndex body_B = frameB.body().index();
  for (const JointIndex& joint_index : plant_.GetJointIndices()) {
    const Joint<double>& joint = plant_.get_joint(joint_index);
    if (joint.num_velocities() == 0) continue;
    // TODO(cohnt) Replace std::find with std::binary_search once
    // GetBodiesKinematicallyAffectedBy() returns a set (see its TODO(sherm1)).
    const std::vector<BodyIndex> affected =
        plant_.GetBodiesKinematicallyAffectedBy({joint_index});
    const bool moves_A =
        std::find(affected.begin(), affected.end(), body_A) != affected.end();
    const bool moves_B =
        std::find(affected.begin(), affected.end(), body_B) != affected.end();
    if (moves_A != moves_B) {
      for (int k = 0; k < joint.num_velocities(); ++k) {
        path_velocities_.push_back(joint.velocity_start() + k);
      }
    }
  }
  std::sort(path_velocities_.begin(), path_velocities_.end());
  if (ssize(path_velocities_) < 6) {
    throw std::logic_error(fmt::format(
        "ManipulabilityCost(): only {} generalized velocities are on the "
        "kinematic path between frame A ('{}') and frame B ('{}'), but the "
        "manipulability index needs at least 6.",
        ssize(path_velocities_), frameA.name(), frameB.name()));
  }
}

ManipulabilityCost::~ManipulabilityCost() = default;

double ManipulabilityCost::CalcIndex(const Eigen::Ref<const Eigen::VectorXd>& q,
                                     Eigen::VectorXd* dw_dq) const {
  UpdateContextConfiguration(context_, plant_, q);
  const Frame<double>& frameA = plant_.get_frame(frameA_index_);
  const Frame<double>& frameB = plant_.get_frame(frameB_index_);
  const int n = ssize(path_velocities_);
  const int nv = plant_.num_velocities();
  // Neither the point (Bo here) nor the expressed-in frame affects w:
  // re-expressing multiplies J by diag(R, R) and moving the point by r
  // multiplies it by [I 0; -[r]ₓ I], both of determinant 1. (The translational
  // index alone would depend on the point.) Off-path columns are zero, so
  // dropping them leaves J Jᵀ unchanged.
  Eigen::MatrixXd J_full(6, nv);
  plant_.CalcJacobianSpatialVelocity(*context_, JacobianWrtVariable::kV, frameB,
                                     Eigen::Vector3d::Zero(), frameA, frameA,
                                     &J_full);
  Matrix6X<double> J(6, n);
  for (int k = 0; k < n; ++k) J.col(k) = J_full.col(path_velocities_[k]);

  const Eigen::SelfAdjointEigenSolver<Matrix6<double>> eigen(J * J.transpose());
  const Vector6<double>& lambda = eigen.eigenvalues();
  if (!(lambda.minCoeff() > 0)) {
    if (dw_dq != nullptr) dw_dq->setZero(plant_.num_positions());
    return 0.0;
  }
  const double w = std::sqrt(lambda.prod());
  if (dw_dq != nullptr) {
    // ∂w/∂vᵢ = w⋅⟨M⁻¹J, Dᵢ(J)⟩ (Jacobi's formula; Frobenius inner product),
    // M = J Jᵀ, then ∂w/∂q = N⁺(q)ᵀ⋅∂w/∂v. N⁺ handles non-unit quaternions.
    const Matrix6X<double> Minv_J = eigen.eigenvectors() *
                                    lambda.cwiseInverse().asDiagonal() *
                                    eigen.eigenvectors().transpose() * J;
    // Taking the derivatives in A (frame_E = A) matches J's expressed-in frame.
    std::vector<Matrix6X<double>> H;
    plant_.CalcHessianSpatialVelocity(*context_, JacobianWrtVariable::kV,
                                      frameB, Eigen::Vector3d::Zero(), frameA,
                                      frameA, &H);
    Eigen::VectorXd dw_dv(nv);
    for (int i = 0; i < nv; ++i) {
      double inner = 0;
      for (int k = 0; k < n; ++k) {
        inner += Minv_J.col(k).dot(H[i].col(path_velocities_[k]));
      }
      dw_dv(i) = w * inner;
    }
    *dw_dq = plant_.MakeQDotToVelocityMap(*context_).transpose() * dw_dv;
  }
  return w;
}

void ManipulabilityCost::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                Eigen::VectorXd* y) const {
  y->resize(1);
  (*y)[0] = -c_ * CalcIndex(x, nullptr);
}

void ManipulabilityCost::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                AutoDiffVecXd* y) const {
  y->resize(1);
  Eigen::VectorXd dw_dq;
  const double w = CalcIndex(math::ExtractValue(x), &dw_dq);
  (*y)[0] =
      AutoDiffXd(-c_ * w, -c_ * math::ExtractGradient(x).transpose() * dw_dq);
}

void ManipulabilityCost::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::logic_error(
      "ManipulabilityCost::DoEval() does not work for symbolic variables.");
}

}  // namespace multibody
}  // namespace drake
