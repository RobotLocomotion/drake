#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"
#include "drake/multibody/contact_solvers/sap/sap_constraint.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

/* The SAP formulation linearizes the discrete time dynamics of the mechanical
system of interest to obtain the SAP momentum equations:
  A⋅(v−v*) = Jᵀ⋅γ                                                           (1)
where A is an SPD approximation of the linearized discrete dynamics, v* are the
free-motion generalized velocities (i.e. the velocities of the system when
constraint impulses are zero), γ are the contraint's impulses and J the
constraint's Jacobian. SAP's formulation solves a strictly convex optimization
problem such that at optimality:
  1. the linearized momentum equation in Eq. (1) is satisfied and,
  2. impulses γ are constrained to live in a convex set.
Please refer to [Castro et al., 2021] for details.

Therefore a SapContactProblem is described by:
  1. The linear dynamics matrix A,
  2. the free motion velocities v* and,
  3. the set of constraints for the problem (which will define the Jacobian J).

Sparsity Structure of the Dynamics and Cliques
==============================================

The SAP formulation allows to couple an arbitrary number of mechanical systems
by constraints. That is, we can have several Equations (1) coupled by
constraints, each momentum equation corresponding to different mechanical
systems. Consider, say, two robots that only interact with each other by
contact. Unless they are in contact, their dynamics can be solved separately.

Another way to say this is that the dynamics matrix A in Eq. (1) is block
diagonal. Each block of this diagonal matrix groups DOFs together in what herein
we call "cliques". In multibody code, we associate a clique with a tree (a full
MultibodyPlant model is a "forest" of many trees. In particular, a single 6 DOF
floating body is its own tree). With deformable FEM models, our cliques will be
associated with the group of participating DOFs, each deformable body having its
own clique. In both cases we will store A as dense blocks.
NOTE: For FEM the contact problem we actually build only entails those DOFs that
"participate" through contact, typically boundary DOFs. Even though the full
volumetric FEM problem is sparse, the dynamics matrix A that only couples
boundary DOFs is always dense.
NOTE: For rigid body systems we could take advantage of branch induced sparsity.
Future versions of this code will consume additional information (the array of
parents, see [Featherstone, 2008] and [Carpentier et al., 2021]) to this end.


[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107
[Carpentier et al., 2021] Carpentier J., Budhiraja R. and Mansard N., 2021.
Proximal and sparse resolution of constrained dynamic equations. In Robotics:
Science and Systems 2021.
[Featherstone, 2008] Featherstone, R., 2008. Rigid body dynamics algorithms.
Springer.*/
template <typename T>
class SapContactProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapContactProblem);

  // Constructs a SAP contact problem for a system of equations discretized with
  // a given `time_step` provided its linear dynamics matrix A and free motion
  // velocities v*. See this class's documentation for details.
  //
  // @param[in] time_step
  //   The time step used to discretize the dynamics in time, it must be
  //   strictly positive. E.g. the time step used to write an implicit Euler
  //   scheme of the dynamics.
  // @param[in] A
  //   SPD approximation of the linearized dynamics, [Castro et al., 2021]. This
  //   matrix is block diagonal with each block corresponding to a "clique". The
  //   number of cliques is A.size() and the number of DOFs in the c-th clique
  //   is A[c].rows() (or A[c].cols() since each block is square). The total
  //   number of generalized velocities of the system is nv = ∑A[c].rows().
  // @param[in] v_star
  //   Free-motion velocities, of size nv.
  //
  // @throws if the blocks in A are not square or have zero size.
  // @throws if the size of v_star is not nv = ∑A[c].rows().
  SapContactProblem(const T& time_step, std::vector<MatrixX<T>>&& A,
                    VectorX<T>&& v_star);

  // TODO(amcastro-tri): consider constructor API taking std::vector<VectorX<T>>
  // for v_star. It could be useful for deformables.

  void AddConstraint(std::unique_ptr<SapConstraint<T>> c);

  // Returns the number of cliques.
  int num_cliques() const;

  // Returns the number of constraints.
  int num_constraints() const;

  // Returns the total number of constrained DOFs nk. That is, nk = ∑ni where ni
  // is the number of constraints DOFs for the i-th contraints, see
  // SapConstraint::num_constrained_dofs().
  int num_constrained_dofs() const { return num_constrained_dofs_; }

  // The total number of generalized velocities for this problem.
  int num_velocities() const;

  // The number of generalized velocities for clique with index `clique_index`.
  // clique_index must be in the interval [0, num_cliques()).
  int num_velocities(int clique_index) const;

  // Access the k-th constraint.
  const SapConstraint<T>& get_constraint(int k) const;

  const T& time_step() const { return time_step_; }

  // Returns the block diagonal dynamics matrix A.
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  // Returns the full vector of free-motion velocities v*, of size
  // num_velocities().
  const VectorX<T>& v_star() const { return v_star_; }

  ContactProblemGraph MakeGraph() const;

 private:
  int nv_{0};                    // total number of generalized velocities.
  T time_step_{0.0};             // Discrete time step.
  std::vector<MatrixX<T>> A_;    // Linear dynamics matrix.
  VectorX<T> v_star_;            // Free-motion velocities.
  int num_constrained_dofs_{0};  // Total number of constrained DOFs.
  // Vector of constraints owned by this problem.
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
