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
class SapConstraintsBundle {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapConstraintsBundle);

  // We keep a reference to `problem` and its data.
  SapConstraintsBundle(
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
  void ProjectImpulses(const VectorX<T>& y, const VectorX<T>& R,
                       VectorX<T>* gamma,
                       std::vector<MatrixX<T>>* dPdy = nullptr) const;

  void CalcProjectImpulsesAndCalcConstraintsHessian(
      const VectorX<T>& y, const VectorX<T>& R, VectorX<T>* gamma,
      std::vector<MatrixX<T>>* G) const;

 private:
  // Jacobian for the entire bundle, with graph_.num_edges() block rows and
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

  int num_velocities() const;
  int num_constraints() const;
  int num_impulses() const;

  const VectorX<T>& v_star() const;
  const VectorX<T>& p_star() const;

  // Performs multiplication p = A * v. Only participating cliques are
  // considered.
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* p) const;

 private:
  PartialPermutation MakeParticipatingCliquesPermutation(
      const ContactProblemGraph& graph) const;

  BlockSparseMatrix<T> MakeConstraintsBundleJacobian(
      const SapContactProblem<T>& problem, const ContactProblemGraph& graph,
      const PartialPermutation& cliques_permutation) const;

  // Computes a diagonal approximation of the Delassus operator used to
  // compute
  // a per constraint diagonal scaling into delassus_diagonal. Given an
  // approximation Wₖₖ of the block diagonal element corresponding to the k-th
  // constraint, the scaling is computed as delassus_diagonal[k] = ‖Wₖₖ‖ᵣₘₛ =
  // ‖Wₖₖ‖/3. See [Castro et al. 2021] for details.
  // @pre Matrix entries stored in `At` are SPD.
  void CalcDelassusDiagonalApproximation(int nc,
                                         const std::vector<MatrixX<T>>& At,
                                         const BlockSparseMatrix<T>& Jblock,
                                         VectorX<T>* delassus_diagonal) const;

  const SapContactProblem<T>* problem_{nullptr};
  ContactProblemGraph graph_;  // The graph corresponding to problem_.
  PartialPermutation cliques_permutation_;

  // Per-clique blocks of the momentum matrix. Only participating cliques.
  // That is, the size of At is cliques_permutation_.domain_size().
  std::vector<MatrixX<T>> participating_cliques_dynamics_;

  // Inverse of the diagonal matrix formed with the square root of the
  // diagonal entries of the momentum matrix, i.e. inv_sqrt_A =
  // diag(A)^{-1/2}. This matrix is used to compute a scaled momentum
  // residual, see discussion on tolerances in SapSolverParameters for
  // details.
  VectorX<T> inv_sqrt_A_;

  VectorX<T> delassus_diagonal_;  // Delassus operator diagonal approximation.
  VectorX<T> p_star_;  // Free motion generalized impulse, i.e. p* = M⋅v*.
  std::unique_ptr<SapConstraintsBundle<T>> constraints_bundle_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

template class ::drake::multibody::contact_solvers::internal::SapModel<double>;
