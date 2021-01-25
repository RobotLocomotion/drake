#pragma once

#include <map>
#include <optional>
#include <utility>

#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %DirichletBoundaryCondition provides functionalities related to Dirichlet
 boundary conditions (BC) to the FEM solver. In particular, it provides the
 following functionalities:
 * 1. storing the information necessary to apply the BC;
 * 2. modifying a given state to comply with the stored BC;
 * 3. modifying a given tangent matrix and residual pair that arises from the
 FEM system without BC and transform it into an equivalent tangent
 matrix/residual pair for the same system under the stored BC.
 @tparam State    The type of FemState subject to the boundary condition.
 `State` must be an instantiation of FemState. */
template <class State>
class DirichletBoundaryCondition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirichletBoundaryCondition);
  using T = typename State::T;

  DirichletBoundaryCondition() = default;

  /** Sets the dof with index `dof_index` to be subject to the prescribed
   `boundary_state`. */
  void AddBoundaryCondition(DofIndex dof_index,
                            Vector<T, State::ode_order() + 1> boundary_state) {
    bcs_[dof_index] = std::move(boundary_state);
  }

  /** Modifies the given `state` so that it complies with the stored boundary
   conditions.
   @pre state != nullptr.
   @throw std::exception if the any of the index of the dof under boundary
   condition as specified by `this` %DirichletBoundaryCondition does not exist
   in the input `state`. */
  void ApplyBoundaryConditions(State* state) const {
    DRAKE_DEMAND(state != nullptr);
    if (bcs_.size() == 0) {
      return;
    }
    /* Check validity of the dof indices stored. */
    const auto last_bc = bcs_.crbegin();
    if (last_bc->first > state->num_generalized_positions()) {
      throw std::runtime_error(
          "An index of the dirichlet boundary condition is out of the range "
          "of the state.");
    }
    using Eigen::VectorBlock;
    /* Grab mutable states to write to. */
    VectorBlock<VectorX<T>> q = state->mutable_q();
    std::optional<VectorBlock<VectorX<T>>> qdot;
    std::optional<VectorBlock<VectorX<T>>> qddot;
    if constexpr (State::ode_order() >= 1) {
      qdot = state->mutable_qdot();
    }
    if constexpr (State::ode_order() == 2) {
      qddot = state->mutable_qddot();
    }
    /* Write the BC to the mutable state. */
    for (const auto& [dof_index, boundary_state] : bcs_) {
      q(dof_index) = boundary_state(0);
      if constexpr (State::ode_order() >= 1) {
        qdot.value()(dof_index) = boundary_state(1);
      }
      if constexpr (State::ode_order() == 2) {
        qddot.value()(dof_index) = boundary_state(2);
      }
    }
  }

  /** Modifies the given tangent matrix/residual pair that arises from an FEM
   system without BC (denoted as Â and b̂) to an equivalent tangent
   matrix/residual pair for the same system subject to `this` BC (denoted as A
   and b). More specifically, A and b satisfy the following properties:
   * 1. A is symmetric and/or positive definite if the input Â is symmetric
   and/or positive definite.
   * 2. If `x` is the solution of the linear system Ax = b, then xᵢ = 0
   if the i-th dof is subject to `this` BC will be 0.
   * 3. Let `x` be the solution of the modified linear system Ax = b, and let yᵢ
   be equal to the prescribed boundary state if the i-th dof is subject to
   `this` BC and equal to xᵢ otherwise. Then Âᵢⱼyⱼ = b̂ᵢ for all i such that the
   i-th dof is not subject to `this` BC.
   @pre tangent_matrix != nullptr.
   @pre residual != nullptr.
   @pre tangent_matrix->rows() == tangent_matrix->cols() &&  residual->size() ==
   tangent_matrix->cols().
   @throw std::exception if the any of the index of the dof under boundary
   condition as specified by `this` %DirichletBoundaryCondition does not exist
   is greater than or equal to the `residual->size()`. */
  void ApplyBoundaryConditions(Eigen::SparseMatrix<T>* tangent_matrix,
                               EigenPtr<VectorX<T>> residual) const {
    DRAKE_DEMAND(tangent_matrix != nullptr);
    DRAKE_DEMAND(residual != nullptr);
    DRAKE_DEMAND(tangent_matrix->rows() == tangent_matrix->cols() &&
                 residual->size() == tangent_matrix->cols());
    if (bcs_.size() == 0) {
      return;
    }
    /* Check validity of the dof indices stored. */
    int size = residual->size();
    const auto& last_bc = bcs_.crbegin();
    if (last_bc->first > size) {
      throw std::runtime_error(
          "The index of the dirichlet boundary condition is out of the range "
          "of the state.");
    }

    /* Set x to the prescribed boundary values where there exists a BC, and set
     x to 0 elsewhere in the first pass. */
    VectorX<T> x = VectorX<T>::Zero(size);
    for (const auto& [dof_index, boundary_state] : bcs_) {
      /* The key variable is the last variable in a state vector. */
      x(dof_index) = boundary_state(State::ode_order());
    }
    /* Account for the contribution of BC from the left-hand side to the
     residual. */
    *residual -= (*tangent_matrix) * x;

    /* In the second pass, zero out all rows and columns of the tangent matrix
     and the residual corresponding to the dof under BC (except the diagonal
     entry). */
    for (const auto& it : bcs_) {
      DofIndex dof_index = it.first;
      (*residual)(dof_index) = 0;
      tangent_matrix->row(dof_index) *= T(0);
      tangent_matrix->col(dof_index) *= T(0);
      tangent_matrix->coeffRef(dof_index, dof_index) = T(1);
    }
  }

 private:
  /* We sort the boundary conditions according to dof indices for better
   cache consistency when applying the BC. The value of the map stores the
   value of q, qdot and qddot (in that order and when applicable) of the dof
   with index of the key. */
  std::map<DofIndex, Vector<T, State::ode_order() + 1>> bcs_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
