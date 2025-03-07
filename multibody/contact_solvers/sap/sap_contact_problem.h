#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"
#include "drake/multibody/contact_solvers/sap/sap_solver_results.h"
#include "drake/multibody/math/spatial_algebra.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* Struct returned by SapContactProblem::MakeReduced() which stores the mapping
   between the original and reduced problems.*/
struct ReducedMapping {
  PartialPermutation velocity_permutation;
  PartialPermutation clique_permutation;
  PartialPermutation constraint_equation_permutation;
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

  /* Returns a deep-copy of `this` instance. */
  std::unique_ptr<SapContactProblem<T>> Clone() const;

  /* When T = double, this method returns the result of Clone().
     When T = AutoDiffXd this method returns a deep copy where gradients were
     discarded. */
  std::unique_ptr<SapContactProblem<double>> ToDouble() const;

  /* Makes a "reduced" contact problem given the DOFs specified in
    `known_free_motion_dofs` are known to equal the free-motion velocities.
    That is, for an i-th DOF in `known_free_motion_dofs`, we know that vᵢ = vᵢ*.
    `per_clique_known_free_motion_dofs` contains the same information stored in
    `known_free_motion_dofs`, but expressed per-clique and in clique local
    indices. e.g. If clique 0 corresponds to global velocity indices {5, 6, 7},
    then its local velocity indices are {0, 1, 2}. If `known_free_motion_dofs` =
    {6}, i.e. we know v₆ = v₆*, then `per_clique_known_free_motion_dofs[0] =
    {1}` containing the known clique local index 1, corresponding to the known
    global velocity index 6. `per_clique_known_free_motion_dofs` need not
    specify indices for every clique in the problem, it may only specify known
    DoFs for the first n cliques where n < num_cliques(). All other cliques are
    assumed to not have any known DoFs.

    @param[in] known_free_motion_dofs Specifies known DOFs to be eliminated.
    i ∈ known_free_motion_dofs specifies the i-th DOF.
    @param[in] per_clique_known_free_motion_dofs Specifies known DOFs to be
    eliminated, per clique. i ∈ per_clique_known_free_motion_dofs[c] specifies
    the i-th DOF of the c-th clique. This parameter should contain the same
    information in `known_free_motion_dofs`, but transformed to clique local
    indices.
    @param[out] mapping On output it will store information to map DOFs and
    constraint equations between the original and reduced problems in a
    ReducedMapping.

    @pre known_free_motion_dofs is a strict ordered subset of
         [0, ..., num_velocities()-1].
    @pre per_clique_known_free_motion_dofs.size() <= num_cliques().
    @pre per_clique_known_free_motion_dofs[c] is a strict ordered subset of
         [0, ..., num_velocities(c)-1].
    @pre mapping != nullptr.
  */
  std::unique_ptr<SapContactProblem<T>> MakeReduced(
      const std::vector<int>& known_free_motion_dofs,
      const std::vector<std::vector<int>>& per_clique_known_free_motion_dofs,
      ReducedMapping* mapping) const;

  /* Maps solver results for a reduced version of this problem obtained with
    MakeReduced() into solver results for this original problem. Known
    velocities eliminated from the reduced problem are set to v* in `results`,
    consistent with the documentation in MakeReduced(). Constraints eliminated
    in the reduced problem do not participate and therefore their corresponding
    impulses are set to zero.

     @param[in] reduced_mapping Stores the mapping between this problem and a
       reduced problem obtained with MakeReduced().
     @param[in] reduced_results Solver results for a reduced version of this
       problem consistent with `reduced_mapping`.
     @param[out] results On output stores the solver results contained in
       `reduced_results` mapped back to this original problem. Known velocities
        are set to v* and impulses for constraints eliminated from the reduced
        problem are set to zero.
     @pre reduced_mapping.velocity_permutation.domain_size() ==
          num_velocities().
     @pre reduced_mapping.clique_permutation.domain_size() == num_cliques().
     @pre reduced_mapping.constraint_equation_permutation.domain_size() ==
       num_constraint_equations().
     @pre reduced_results.v.size() ==
          reduced_mapping.velocity_permutation.permuted_domain_size()
     @pre reduced_results.j.size() ==
          reduced_mapping.velocity_permutation.permuted_domain_size()
     @pre reduced_results.gamma.size() ==
          reduced_mapping.constraint_equation_permutation.permuted_domain_size()
     @pre reduced_results.vc.size() ==
          reduced_mapping.constraint_equation_permutation.permuted_domain_size()
     @pre results != nullptr.
     @see SapContactProblem::MakeReduced() for more information.
  */
  void ExpandContactSolverResults(const ReducedMapping& reduced_mapping,
                                  const SapSolverResults<T>& reduced_results,
                                  SapSolverResults<T>* results) const;

  /* TODO(amcastro-tri): consider constructor API taking std::vector<VectorX<T>>
   for v_star. It could be useful for deformables. */

  /* Adds `constraint` to this problem.
   @throws exception if the clique indices referenced by `constraint` are not in
   the range [0, num_cliques()).
   @throws exception if the number of columns of the Jacobian matrices in
   `constraint` is not consistent with the number of velocities for the cliques
   in this problem referenced by `constraint` or if they are both zero.
   @returns the index to the newly added constraint. */
  int AddConstraint(std::unique_ptr<SapConstraint<T>> constraint);

  /* Sets the number of physical objects associated with this problem.
   This call must be performed before any constraints are added to the problem,
   or an exception is thrown. Constraints (added with AddConstraint()) that
   register objects will be required to register object indices strictly lower
   than num_objects(). See @ref sap_physical_forces.
   @throws if num_constraints() > 0. */
  void set_num_objects(int num_objects);

  /* The number of physical objects associated with this problem. See @ref
   sap_physical_forces. */
  int num_objects() const { return num_objects_; }

  /* Returns the number of cliques. */
  int num_cliques() const { return A_.size(); }

  /* Returns the total number of generalized velocities for this problem. */
  int num_velocities() const { return nv_; }

  /* Returns the index to the first velocity for a given clique, within the
   full vector of generalized velocities for the entire problem.
   That is, with v the full vector of generalized velocities for this problem,
   v.segment(velocities_start(c), num_velocities(c)) corresponds to the vector
   of generalized velocities for the c-th clique. */
  int velocities_start(int clique_index) const {
    DRAKE_THROW_UNLESS(0 <= clique_index && clique_index < num_cliques());
    return velocities_start_[clique_index];
  }

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

  /* Returns the index to the first constraint velocity for a given constraint,
   within the full vector of constraint velocities for the entire problem. That
   is, with vc the full vector of constraint velocities for this problem,
   vc.segment(constraint_equations_start(c), num_constraint_equations(c))
   corresponds to the vector of constraint velocities for the c-th constraint.*/
  int constraint_equations_start(int constraint_index) const {
    DRAKE_THROW_UNLESS(0 <= constraint_index &&
                       constraint_index < num_constraints());
    return constraint_equations_start_[constraint_index];
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

  /* Returns a graph where cliques are nodes and clusters of constraints
  (ContactProblemGraph::ConstraintCluster) are the edges. Refer to the class
  documentation of ContactProblemGraph for a definition of constraint clusters.
  */
  const ContactProblemGraph& graph() const { return graph_; }

  /* Compute generalized forces per DoF and spatial forces per object given
   a known vector of impulses `gamma`.
   @param[in] gamma Constraint impulses for this full problem. Of size
   num_constraint_equations().
   @param[out] generalized_forces On output, the set of generalized forces
   result of the combined action of all constraints in `this` problem given the
   known impulses `gamma`.
   @param[out] spatial_forces On output, the set of spatial forces
   result of the combined action of all constraints in `this` problem given the
   known impulses `gamma`.

   @throws if gamma.size() != num_constraint_equations().
   @throws if either generalized_forces or spatial_forces is nullptr.
   @throws if generalized_forces.size() != num_velocities().
   @throws spatial_forces.size() != num_objects(). */
  void CalcConstraintMultibodyForces(
      const VectorX<T>& gamma, VectorX<T>* generalized_forces,
      std::vector<SpatialForce<T>>* spatial_forces) const;

  /* Computes the generalized forces given a known vector of impulses `gamma`,
   for constraints with index i in the inclusive range constraint_start <= i &&
   i <= constraint_end.

   @param[in] gamma Constraint impulses for this full problem. Of size
   num_constraint_equations().
   @param[out] generalized_forces On output, the set of generalized forces
   result of the combined action of all constraints in `this` problem given the
   known impulses `gamma`.

   @throws if gamma.size() != num_constraint_equations().
   @throws if constraint_start is not in [0, num_constraints()).
   @throws if constraint_end is not in [0, num_constraints()).
   @throws if constraint_end < constraint_start.
   @throws if generalized_forces is nullptr.
   @throws if generalized_forces.size() != num_velocities(). */
  void CalcConstraintGeneralizedForces(const VectorX<T>& gamma,
                                       int constraint_start, int constraint_end,
                                       VectorX<T>* generalized_forces) const;

 private:
  int nv_{0};           // Total number of generalized velocities.
  T time_step_{0.0};    // Discrete time step.
  int num_objects_{0};  // Number of physical objects.
  std::vector<int> velocities_start_;
  // Gives the index of the first constraint equation for each constraint.
  // Has size = num_constraints() + 1 and at any time:
  // constraint_equations_start_.back() == num_constraint_equations().
  std::vector<int> constraint_equations_start_{0};
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
