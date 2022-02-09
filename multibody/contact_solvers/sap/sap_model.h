#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/partial_permutation.h"
#include "drake/multibody/contact_solvers/point_contact_data.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
class SapConstraintBundle {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraintBundle);

  // We keep a reference to `problem` and its data.
  SapConstraintBundle(
      BlockSparseMatrix<T>&& J, VectorX<T>&& vhat, VectorX<T>&& R,
      std::vector<const SapConstraint<T>*>&& constraits);

  int num_constraints() const;

  const BlockSparseMatrix<T>& J() const { return J_; }

  const VectorX<T>& R() const { return R_; }

  const VectorX<T>& Rinv() const { return Rinv_; }

  const VectorX<T>& vhat() const { return vhat_; }

  void MultiplyByJacobian(const VectorX<T>& v, VectorX<T>* vc) const;

  void MultiplyByJacobianTranspose(const VectorX<T>& gamma,
                                   VectorX<T>* jc) const;

  void CalcUnprojectedImpulses(const VectorX<T>& vc, VectorX<T>* y) const;

  // Computes the projection gamma = P(y) for all impulses and the gradient
  // dP/dy if dgamma_dy != nullptr.
  // TODO: no need to have R as an argument. Remove.
  void ProjectImpulses(const VectorX<T>& y, const VectorX<T>& R,
                       VectorX<T>* gamma,
                       std::vector<MatrixX<T>>* dPdy = nullptr) const;

  // TODO: no need to have R as an argument. Remove.
  void ProjectImpulsesAndCalcConstraintsHessian(
      const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
      std::vector<MatrixX<T>>* G) const;

 private:
  // Jacobian for the entire bundle, with graph_.num_constraint_groups() block rows and
  // graph_.num_cliques() block columns.
  BlockSparseMatrix<T> J_;
  VectorX<T> vhat_;
  VectorX<T> R_;
  VectorX<T> Rinv_;
  // problem_ constraint references in the order dictated by the
  // ContactProblemGraph.
  std::vector<const SapConstraint<T>*> constraints_;
};

template <typename T>
class SapModel {
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapModel);
 public:
  // Constructs an empty model. Needed?
  //SapModel() = default;


  // `problem` must outlive `this` model.  
  explicit SapModel(const SapContactProblem<T>* problem);

  SapModel(const T& time_step, const SystemDynamicsData<T>& dynamics_data,
           const PointContactData<T>& contact_data);

  const SapContactProblem<T>& sap_problem() const { 
      DRAKE_ASSERT(problem_!=nullptr);
      return *problem_; 
  }

  int num_cliques() const;
  int num_participating_cliques() const;  
  int num_velocities() const;
  int num_participating_velocities() const;
  int num_constraints() const;
  int num_impulses() const;

  // Returns permutation for participating cliques.  
  const PartialPermutation& cliques_permutation() const {
    return cliques_permutation_;
  }

  // Returns permutation for participating velocities.
  const PartialPermutation& velocities_permutation() const {
    return velocities_permutation_;
  }

  const PartialPermutation& impulses_permutation() const {
    return impulses_permutation_;
  }

  const T& time_step() const { return problem_->time_step(); }

  // Returns the system dynamics matrix A for the participating DOFs only.
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  const BlockSparseMatrix<T>& J() const { return constraints_bundle_->J(); }

  const VectorX<T>& R() const {
    return constraints_bundle_->R();
  }

  const VectorX<T>& Rinv() const {
    return constraints_bundle_->Rinv();
  }

  // Returns free-motion velocities for participating DOFs.
  const VectorX<T>& v_star() const;

  // Returns free-motion generalized momenta for participating DOFs.
  const VectorX<T>& p_star() const;

  // Returns diag(A)^{-1/2}. Used for scaling SAP equations and residuals.
  // Of size num_participating_velocities().
  const VectorX<T>& inv_sqrt_A() const { return inv_sqrt_A_; }

  // Performs multiplication p = A * v. Only participating DOFs are
  // considered.
  // @pre p must be a valid pointer.
  // @pre both v and p must be of size num_participating_velocities().
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* p) const;    

  void CalcConstraintVelocities(const VectorX<T>& v, VectorX<T>* vc) const;

  void CalcUnprojectedImpulses(const VectorX<T>& vc, VectorX<T>* y) const;

  void ProjectImpulses(const VectorX<T>& y, VectorX<T>* gamma) const;

  void ProjectImpulsesAndCalcConstraintsHessian(
      const VectorX<T>& y, VectorX<T>* gamma, std::vector<MatrixX<T>>* G) const;

 private:
  friend class SapModelTester;

  PartialPermutation MakeParticipatingCliquesPermutation(
      const ContactProblemGraph& graph) const;
  PartialPermutation MakeParticipatingVelocitiesPermutation(
      const SapContactProblem<T>& problem,
      const PartialPermutation& cliques_permutation) const;
  PartialPermutation MakeImpulsesPermutation(
      const ContactProblemGraph& graph) const;

  BlockSparseMatrix<T> MakeConstraintsBundleJacobian(
      const SapContactProblem<T>& problem, const ContactProblemGraph& graph,
      const PartialPermutation& cliques_permutation) const;  

  // Computes a diagonal approximation of the Delassus operator used to compute
  // a per constraint diagonal scaling into delassus_diagonal. Given an
  // approximation Wᵢᵢ of the block diagonal element corresponding to the i-th
  // constraint, the scaling is computed as delassus_diagonal[i] = ‖Wᵢᵢ‖ᵣₘₛ =
  // ‖Wᵢᵢ‖/nᵢ. See [Castro et al. 2021] for details.
  // @pre Matrix entries stored in `At` are SPD.
  //
  // @param[out] cliques_permutation On output an array of size nc (number of
  // constraints) where each entry stores the Delassus operator constraint
  // scaling. That is, wi = delassus_diagonal[i] corresponds to the scaling for
  // the i-th constraint and mi = 1./wi corresponds to the mass scaling for the
  // same constraint.
  void CalcDelassusDiagonalApproximation(
      const std::vector<MatrixX<T>>& At, const SapContactProblem<T>& problem,
      const ContactProblemGraph& graph,
      const PartialPermutation& cliques_permutation,
      VectorX<T>* delassus_diagonal) const;

  // Overload used when a BlockSparseMatrix of the contact jacobian is
  // available. Used when building a model from SystemDynamicsData and
  // PointContactData.
  void CalcDelassusDiagonalApproximation(int nc,
                                         const std::vector<MatrixX<T>>& At,
                                         const BlockSparseMatrix<T>& Jblock,
                                         VectorX<T>* delassus_diagonal) const;

  const SapContactProblem<T>* problem_{nullptr};
  PartialPermutation cliques_permutation_;
  PartialPermutation velocities_permutation_;
  PartialPermutation impulses_permutation_;

  // Per-clique blocks of the momentum matrix. Only participating cliques.
  // That is, the size of At is cliques_permutation_.domain_size().
  std::vector<MatrixX<T>> A_;

  // Inverse of the diagonal matrix formed with the square root of the
  // diagonal entries of the momentum matrix, i.e. inv_sqrt_A =
  // diag(A)^{-1/2}. This matrix is used to compute a scaled momentum
  // residual, see discussion on tolerances in SapSolverParameters for
  // details.
  VectorX<T> inv_sqrt_A_;

  VectorX<T> delassus_diagonal_;  // Delassus operator diagonal approximation.
  VectorX<T> v_star_;  // Free motion generalized velocity v*.
  VectorX<T> p_star_;  // Free motion generalized impulse, i.e. p* = M⋅v*.
  std::unique_ptr<SapConstraintBundle<T>> constraints_bundle_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

