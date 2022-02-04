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

// SAP Problem defined by:
//   - A⋅(v−v*) = Jᵀ⋅γ
//   - Constraints.
// See design document for more details.
template <typename T>
class SapContactProblem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SapContactProblem);

  // Constructs a SAP contact problem where the linearized dynamics read:
  //  A⋅(v−v*) = Jᵀ⋅γ
  // where A is the linearized discrete momentum matrix and v* are the
  // free-motion generalized velocities. The contact Jacobian J is specified as
  // constraints are added with AddConstraint().
  // 
  // @param time_step The time step used by the discrete SAP formulation.
  // @param A Linearized dynamics momentum matrix. It is block diagonal. Each
  // block corresponds to a group of DOFs or "clique". The number of cliques is
  // A.size() and the number of DOFs in the c-th clique is A[c].rows() (or
  // A[c].cols() since each block is square). The total number of generalized
  // velocities is nv = ∑A[c].rows().
  // @param v_star Free-motion velocities, of size nv. DOFs order must
  // correspond to those referenced in A.
  // 
  // TODO: I found useful to keep a reference to the original data in the scope
  // where the problem is built (the manager). Therefore it might be more
  // convenient for the constructor to keep references to data rather than
  // moving copies of the original data.
  SapContactProblem(const T& time_step, std::vector<MatrixX<T>>&& A,
                    VectorX<T>&& v_star);

  // TODO: I believe this is not needed. Remove.
  SapContactProblem(
      std::vector<MatrixX<T>>&& A, VectorX<T>&& v_star,
      std::vector<std::unique_ptr<SapConstraint<T>>>&& constraints);

  void AddConstraint(std::unique_ptr<SapConstraint<T>> c);

  int num_cliques() const;

  int num_constraints() const;

  int num_constrained_dofs() const { return num_constrained_dofs_; }

  int num_velocities() const;

  int num_velocities(int clique) const;

  const SapConstraint<T>& get_constraint(int k) const;

  const T& time_step() const { return time_step_;  }

  // Returns the block diagonal dynamics matrix A.
  const std::vector<MatrixX<T>>& dynamics_matrix() const;

  const VectorX<T>& v_star() const { return v_star_;  }

  ContactProblemGraph MakeGraph() const;

 private:
  int nv_{0};
  T time_step_{0.0};
  std::vector<MatrixX<T>> A_;
  VectorX<T> v_star_;
  int num_constrained_dofs_{0};
  std::vector<std::unique_ptr<SapConstraint<T>>> constraints_;
};

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
