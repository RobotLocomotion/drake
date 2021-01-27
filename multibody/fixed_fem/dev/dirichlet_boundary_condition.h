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
// TODO(xuchenhan-tri): Have FemModel own DirichletBoundaryCondition. See issue
// #14668.
/** %DirichletBoundaryCondition provides functionalities related to Dirichlet
 boundary conditions (BC) to the FEM solver. In particular, it provides the
 following functionalities:
 1. storing the information necessary to apply the BC;
 2. modifying a given state to comply with the stored BC;
 3. modifying a given tangent matrix/residual that arises from the FEM system
 without BC and transform it into the tangent matrix/residual for the same
 system under the stored BC.

 The workflow for solving for an equilibrium in a system subject to BC looks
 like:
 ```
 // First, apply the boundary condition to the state.
 bc.ApplyBoundaryCondition(&state);
 // Then find the residual for the system without BC using FemModel.
 model.CalcResidual(state, &residual);
 // Modify the residual to account for the BC.
 bc.ApplyBcToResidual(&residual);
 // Enter Newton-Raphson iteration:
 while (residual.norm() > kTol){
    // Find the tangent matrix and apply the BC.
    model.CalcTangentMatrix(state, &tangent_matrix);
    bc.ApplyBcToTangentMatrix(&tangent_matrix);
    // Solve for the change in the state variable, dz, with a linear solver.
    ...
    // Advance the state with StateUpdater.
    state_updater.UpdateState(dz, &state);
    // Find the residual after the Newton-Raphson iteration.
    model.CalcResidual(state, &residual);
    bc.ApplyBcToResidual(&residual);
 }
 ```
 @tparam State    The type of FemState subject to the boundary condition.
 `State` must be an instantiation of FemState. */
template <class State>
class DirichletBoundaryCondition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirichletBoundaryCondition);
  using T = typename State::T;

  DirichletBoundaryCondition() = default;

  /** Sets the dof with index `dof_index` to be subject to the prescribed
   `boundary_state`.
   @param[in] dof_index Index into the generalized positions (and their
   derivatives) of the state.
   @param[in] boundary_state The tuple of prescibed state at `dof_index`. For a
   2nd-order ODE, it takes the form [q(dof_index), qdot(dof_index),
   qddot(dof_index)]. The `qdot` and `qddot` terms do not show up for lower
   order ODEs. */
  void AddBoundaryCondition(
      DofIndex dof_index,
      const Eigen::Ref<const Vector<T, State::ode_order() + 1>>&
          boundary_state) {
    bcs_[dof_index] = boundary_state;
  }

  /** Modifies the given `state` so that it complies with the stored boundary
   conditions.
   @pre state != nullptr.
   @throw std::exception if the any of the indexes of the dofs under the
   boundary condition specified by `this` %DirichletBoundaryCondition does
   not exist in the input `state`. */
  void ApplyBoundaryConditions(State* state) const {
    DRAKE_DEMAND(state != nullptr);
    if (bcs_.size() == 0) {
      return;
    }
    /* Check validity of the dof indices stored. */
    VerifyBcIndexes(state->num_generalized_positions());
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

  /** Modifies the given tangent matrix that arises from an FEM system without
   BC into the tangent matrix for the same system subject to `this` BC. More
   specifically, the rows and columns corresponding to dofs under the BC will be
   zeroed out with the exception of the diagonal entries for those dofs which
   will be set to 1.
   @pre tangent_matrix != nullptr.
   @pre tangent_matrix->rows() == tangent_matrix->cols().
   @throw std::exception if the any of the indexes of the dofs under the
   boundary condition specified by `this` %DirichletBoundaryCondition is
   greater than or equal to the `tangent_matrix->cols()`. */
  void ApplyBcToTangentMatrix(Eigen::SparseMatrix<T>* tangent_matrix) const {
    DRAKE_DEMAND(tangent_matrix != nullptr);
    DRAKE_DEMAND(tangent_matrix->rows() == tangent_matrix->cols());
    if (bcs_.size() == 0) {
      return;
    }
    /* Check validity of the dof indices stored. */
    VerifyBcIndexes(tangent_matrix->cols());

    /* Zero out all rows and columns of the tangent matrix corresponding to dofs
     under the BC (except the diagonal entry which is set to 1). */
    for (const auto& it : bcs_) {
      const DofIndex dof_index = it.first;
      tangent_matrix->row(dof_index) *= T(0);
      tangent_matrix->col(dof_index) *= T(0);
      tangent_matrix->coeffRef(dof_index, dof_index) = T(1);
    }
  }

  /** Modifies the given residual that arises from an FEM system without BC into
   the residual for the same system subject to `this` BC. More specifically, the
   entries corresponding to dofs under the BC will be zeroed out.
   @pre residual != nullptr.
   @throw std::exception if any of the indexes of the dofs under the boundary
   condition specified by `this` %DirichletBoundaryCondition  is greater than
   or equal to the `residual->size()`. */
  void ApplyBcToResidual(EigenPtr<VectorX<T>> residual) const {
    DRAKE_DEMAND(residual != nullptr);
    if (bcs_.size() == 0) {
      return;
    }
    /* Check validity of the dof indices stored. */
    VerifyBcIndexes(residual->size());

    /* Zero out all entries of the residual corresponding to dofs under the BC.
     */
    for (const auto& it : bcs_) {
      const DofIndex dof_index = it.first;
      (*residual)(dof_index) = 0;
    }
  }

 private:
  /* Verifies that the largest index for the dofs under BC is smaller than the
   given `size`. Otherwise, throw an exception. */
  void VerifyBcIndexes(int size) const {
    const auto& last_bc = bcs_.crbegin();
    if (last_bc->first >= size) {
      throw std::runtime_error(
          "An index of the dirichlet boundary condition is out of the range.");
    }
  }

  /* We sort the boundary conditions according to dof indices for better
   cache consistency when applying the BC. The value of the map stores the
   value of q, qdot and qddot (in that order and when applicable) of the dof
   with index of the key. */
  std::map<DofIndex, Vector<T, State::ode_order() + 1>> bcs_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
