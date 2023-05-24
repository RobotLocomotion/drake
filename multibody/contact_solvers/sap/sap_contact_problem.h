#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Struct returned by SapContactProblem::MakeReduced() which stores the mapping
   between the original and reduced problems.*/
struct ReducedMapping {
  PartialPermutation clique_permutation;
  PartialPermutation constraint_permutation;
};

/* In the SAP formulation of contact the state of a mechanical system is
advanced from the previous state x₀ = [q₀,v₀] to the next state x = [q,v]. The
SAP formulation linearizes the discrete time dynamics of the mechanical system
of interest to obtain the SAP momentum equations:
  A(v*)⋅(v−v*) = J(q₀)ᵀ⋅γ                                                   (1)
where v denotes the generalized velocities of the system, of size nv. Matrix A
(of size nv x nv) is a symmetric positive definite (SPD) approximation of the
linearized discrete dynamics, v* (of size nv) are the free-motion generalized
velocities (i.e. the velocities of the system when constraint impulses are
zero), γ (of size nk) are the constraint impulses and J (of size nk x nv) the
constraint Jacobian. SAP's formulation solves a strictly convex optimization
problem such that at optimality:
  1. the linearized momentum equation in Eq. (1) is satisfied and,
  2. impulses γ are constrained to live in a convex set.
Please refer to [Castro et al., 2021] for details.

Therefore a SapContactProblem is described by:
  1. The linear dynamics matrix A,
  2. the free motion velocities v* and,
  3. the set of constraints for the problem (which defines the Jacobian J).

Often times matrix A is block diagonal. This block diagonal structure of A
induces a partition of the nv generalized velocities. A subset in this partition
is referred to as a "clique". The total number of cliques in this partition
equals the number of diagonal blocks in A.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapContactProblem {
 public:
  /* We allow move semantics and forbid copy semantics. */
  SapContactProblem(const SapContactProblem&) = delete;
  SapContactProblem& operator=(const SapContactProblem&) = delete;
  SapContactProblem(SapContactProblem&&) = default;
  SapContactProblem& operator=(SapContactProblem&&) = default;

  /* Constructs an empty contact problem. Typically used to cheaply instantiate
   a problem within a container. Since `this` problem does not yet store any
   data, it is in an invalid state until Reset() is called.

   @param[in] time_step
     The time step used to discretize the dynamics in time, it must be
     strictly positive. E.g. the time step used to write an implicit Euler
     scheme of the dynamics.

   @throws exception if time_step is not strictly positive. */
  explicit SapContactProblem(const T& time_step);

  /* Constructs a SAP contact problem for a system of equations discretized with
   a given `time_step` provided its linear dynamics matrix A and free motion
   velocities v*. See this class's documentation for details.

   @param[in] time_step
     The time step used to discretize the dynamics in time, it must be
     strictly positive. E.g. the time step used to write an implicit Euler
     scheme of the dynamics.
   @param[in] A
     SPD approximation of the linearized dynamics, [Castro et al., 2021]. This
     matrix is block diagonal with each block corresponding to a "clique". The
     number of cliques is A.size() and the number of DOFs in the c-th clique
     is A[c].rows() (or A[c].cols() since each block is square). The total
     number of generalized velocities of the system is nv = ∑A[c].rows().
     This class does not check for the positive definiteness of each block in
     A, it is the responsibility of the calling code to enforce this
     invariant. Ultimately, the SAP solver can fail to converge if this
     requirement is not satisfied.
   @param[in] v_star
     Free-motion velocities, of size nv. DOFs in v_star must match the
     ordering implicitly induced by A.

   @throws exception if time_step is not strictly positive.
   @throws exception if the blocks in A are not square.
   @throws exception if the size of v_star is not nv = ∑A[c].rows(). */
  SapContactProblem(const T& time_step, std::vector<MatrixX<T>> A,
                    VectorX<T> v_star);

  /* A call to this member function resets the contact problem and effectively
   removes any existing constraints. The problem will have a new number of
   cliques and generalized velocities according to the sizes of `A` and `v_star`
   and no constraints. The time step is preserved, see time_step().

   @param[in] A
     SPD approximation of the linearized dynamics, [Castro et al., 2021]. This
     matrix is block diagonal with each block corresponding to a "clique". The
     number of cliques is A.size() and the number of DOFs in the c-th clique
     is A[c].rows() (or A[c].cols() since each block is square). The total
     number of generalized velocities of the system is nv = ∑A[c].rows().
     This class does not check for the positive definiteness of each block in
     A, it is the responsibility of the calling code to enforce this
     invariant. Ultimately, the SAP solver can fail to converge if this
     requirement is not satisfied.
   @param[in] v_star
     Free-motion velocities, of size nv. DOFs in v_star must match the
     ordering implicitly induced by A.

   @throws exception if the blocks in A are not square.
   @throws exception if the size of v_star is not nv = ∑A[c].rows(). */
  void Reset(std::vector<MatrixX<T>> A, VectorX<T> v_star);

  /* Returns a deep-copy of `this` instance. */
  std::unique_ptr<SapContactProblem<T>> Clone() const;

  /* Makes a "reduced" contact problem given the DOFs specified in
    `known_free_motion_dofs` are known to equal the free-motion velocities.
    That is, for an i-th DOF in `known_free_motion_dofs`, we know that vᵢ = vᵢ*.
    `per_clique_known_free_motion_dofs` contains the same information stored in
    `known_free_motion_dofs`, but expressed per-clique and in clique local
    indices. e.g. If clique 0 corresponds to global velocity indices {5, 6, 7},
    then its local velocity indices are {0, 1, 2}. If `known_free_motion_dofs` =
    {6}, i.e. we know v₆ = v₆*, then `per_clique_known_free_motion_dofs[0] =
    {1}` containing the known clique local index 1, corresponding to the known
    global velocity index 6.

    @param[in] known_free_motion_dofs Specifies known DOFs to be eliminated.
    i ∈ known_free_motion_dofs specifies the i-th DOF.
    @param[in] per_clique_known_free_motion_dofs Specifies known DOFs to be
    eliminated, per clique. i ∈ per_clique_known_free_motion_dofs[c] specifies
    the i-th DOF of the c-th clique. This parameter should contain the same
    information in `known_free_motion_dofs`, but transformed to clique local
    indices.
    @param[out] mapping On output it will store information to map DOFs and
    constraints between the original and reduced problems in a ReducedMapping.

    @pre known_free_motion_dofs is a strict ordered subset of
         [0, ..., num_velocities()-1].
    @pre per_clique_known_free_motion_dofs.size() == num_cliques().
    @pre per_clique_known_free_motion_dofs[c] is a strict ordered subset of
         [0, ..., num_velocities(c)-1].
    @pre mapping != nullptr.
  */
  std::unique_ptr<SapContactProblem<T>> MakeReduced(
      const std::vector<int>& known_free_motion_dofs,
      const std::vector<std::vector<int>>& per_clique_known_free_motion_dofs,
      ReducedMapping* mapping) const;

  /* TODO(amcastro-tri): consider constructor API taking std::vector<VectorX<T>>
   for v_star. It could be useful for deformables. */

  /* Adds `constraint` to this problem.
   @throws exception if the clique indexes referenced by `constraint` are not in
   the range [0, num_cliques()).
   @throws exception if the number of columns of the Jacobian matrices in
   `constraint` is not consistent with the number of velocities for the cliques
   in this problem referenced by `constraint` or if they are both zero.
   @returns the index to the newly added constraint. */
  int AddConstraint(std::unique_ptr<SapConstraint<T>> constraint);

  /* Returns the number of cliques. */
  int num_cliques() const { return A_.size(); }

  /* Returns the total number of generalized velocities for this problem. */
  int num_velocities() const { return nv_; }

  /* Returns the number of generalized velocities for clique with index
   `clique_index`. clique_index must be in the interval [0, num_cliques()). */
  int num_velocities(int clique_index) const {
    DRAKE_THROW_UNLESS(0 <= clique_index && clique_index < num_cliques());
    return A_[clique_index].rows();
  }

  /* Returns the number of constraints in this problem. */
  int num_constraints() const { return constraints_.size(); }

  /* Returns the total number of constraint equations. That is, nk = ∑ni where
   ni is the number of constraint equations for the i-th constraint, see
   SapConstraint::num_constraint_equations(). */
  int num_constraint_equations() const {
    return graph_.num_constraint_equations();
  }

  /* Accesses constraint with index `constraint_index` as assigned by the call
   to AddConstraint(). */
  const SapConstraint<T>& get_constraint(int constraint_index) const {
    DRAKE_THROW_UNLESS(0 <= constraint_index &&
                       constraint_index < num_constraints());
    return *constraints_[constraint_index];
  }

  const T& time_step() const { return time_step_; }

  /* Returns the block diagonal dynamics matrix A. */
  const std::vector<MatrixX<T>>& dynamics_matrix() const { return A_; }

  /* Returns the full vector of free-motion velocities v*, of size
   num_velocities(). */
  const VectorX<T>& v_star() const { return v_star_; }

  const ContactProblemGraph& graph() const { return graph_; }

 private:
  int nv_{0};                  // Total number of generalized velocities.
  T time_step_{0.0};           // Discrete time step.
  std::vector<MatrixX<T>> A_;  // Linear dynamics matrix.
  VectorX<T> v_star_;          // Free-motion velocities.
  ContactProblemGraph graph_;  // Contact graph for this problem.
  // Constraints owned by this problem.
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
