#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/sap/partial_permutation.h"
#include "drake/multibody/contact_solvers/point_contact_data.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/system_dynamics_data.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint_bundle.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* This class represents the underlying computational model built by the SAP
 solver given a SapContactProblem.
 The model re-arranges constraints to exploit the block sparse structure of the
 problem and it only includes participating cliques, that is, cliques that
 connect to a cluster (edge) in the contact graph. The solution for
 non-participating cliques is trivial (v=v*) and therefore they are excluded
 from the main (expensive) SAP computation. 
 
 [Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An
 Unconstrained Convex Formulation of Compliant Contact. Available at
 https://arxiv.org/abs/2110.10107
*/
template <typename T>
class SapModel {
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapModel);
 public:
  /* Constructs a model of `problem` optimized to be used by the SAP solver. 
   The input `problem` must outlive `this` model. */
  explicit SapModel(const SapContactProblem<T>* problem);

  /* Returns a reference to the contact problem being modeled by this class. */
  const SapContactProblem<T>& problem() const { 
      DRAKE_ASSERT(problem_!=nullptr);
      return *problem_; 
  }

  /* Returns the number of (participating) cliques. */
  int num_cliques() const;

  /* Returns the number or (participating) generalized velocities.*/
  int num_velocities() const;

  int num_constraints() const;
  int num_constraint_equations() const;

  /* Returns permutation for participating cliques, allowing to map back and
   forth between cliques in `this` model with cliques in problem(). */
  const PartialPermutation& cliques_permutation() const {
    return cliques_permutation_;
  }

  /* Returns permutation for participating velocities, allowing to map back and
   forth between velocities in `this` model with velocities in problem(). */
  const PartialPermutation& velocities_permutation() const {
    return velocities_permutation_;
  }

  /* Returns permutation for impulses, allowing to map back and
   forth between impulses in `this` model with impulses in problem(). */
  const PartialPermutation& impulses_permutation() const {
    return impulses_permutation_;
  }

  /* The time step of problem(). */
  const T& time_step() const { return problem_->time_step(); }

  /* Returns the system dynamics matrix A for participating velocities only. See
   velocities_permutation(). */
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  /* Returns the constraints Jacobian. Only participating velocities are
   involved. */
  const BlockSparseMatrix<T>& J() const { return constraints_bundle_->J(); }

  /* Returns diagonal of the regularization matrix R. */
  const VectorX<T>& R() const {
    return constraints_bundle_->R();
  }

  /* Returns diagonal of the inverse of the regularization matrix R. */
  const VectorX<T>& Rinv() const {
    return constraints_bundle_->Rinv();
  }

  /* Returns free-motion velocities v*, Participating velocities only. */
  const VectorX<T>& v_star() const;

  /* Returns free-motion generalized momenta. Participating momenta only. */
  const VectorX<T>& p_star() const;

  /* Returns diag(A)^{-1/2}. Used for scaling SAP equations and residuals.
   of size num_velocities(). See [Castro et al., 2021]. */
  const VectorX<T>& inv_sqrt_A() const { return inv_sqrt_A_; }

  /* Performs multiplication p = A * v. Only participating velocities are
   considered.
   @pre p must be a valid pointer.
   @pre both v and p must be of size num_participating_velocities(). */
  void MultiplyByDynamicsMatrix(const VectorX<T>& v, VectorX<T>* p) const;    

  const SapConstraintBundle<T>& constraint_bundle() const {
    return *constraints_bundle_;
  }

 private:
  friend class SapModelTester;

  PartialPermutation MakeParticipatingCliquesPermutation(
      const ContactProblemGraph& graph) const;
  PartialPermutation MakeParticipatingVelocitiesPermutation(
      const SapContactProblem<T>& problem,
      const PartialPermutation& cliques_permutation) const;
  PartialPermutation MakeImpulsesPermutation(
      const ContactProblemGraph& graph) const;  

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
      const std::vector<MatrixX<T>>& At,
      const PartialPermutation& cliques_permutation,
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

