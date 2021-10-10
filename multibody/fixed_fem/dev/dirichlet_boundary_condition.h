#pragma once

#include <map>
#include <optional>
#include <utility>

#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_indexes.h"

namespace drake {
namespace multibody {
namespace fem {
/** %DirichletBoundaryCondition provides functionalities related to Dirichlet
 boundary conditions (BC) to the FEM solver. In particular, it provides the
 following functionalities:
 1. storing the information necessary to apply the BC;
 2. modifying a given state to comply with the stored BC;
 3. modifying a given tangent matrix/residual that arises from the FEM system
 without BC and transform it into the tangent matrix/residual for the same
 system under the stored BC.
 @tparam_nonsymbolic_scalar T. */
template <class T>
class DirichletBoundaryCondition {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DirichletBoundaryCondition);

  /* Constructs a new %DirichletBoundaryCondition that applies to an FEM model
   that has the given `ode_order`. */
  explicit DirichletBoundaryCondition(int ode_order) : ode_order_(ode_order) {}

  /* Returns the ODE order of the FEM model that is compatible with `this`
   %DirichletBoundaryCondition. */
  int ode_order() const { return ode_order_; }

  /** Sets the dof with index `dof_index` to be subject to the prescribed
   `boundary_state`.
   @param[in] dof_index Index into the generalized positions (and their
   derivatives) of the state.
   @param[in] boundary_state The tuple of prescibed state at `dof_index`. For a
   2nd-order ODE, it takes the form [q(dof_index), qdot(dof_index),
   qddot(dof_index)]. The `qdot` and `qddot` terms do not show up for lower
   order ODEs.
   @throw std::exception if boundary_state.size() != ode_order. */
  void AddBoundaryCondition(
      DofIndex dof_index, const Eigen::Ref<const VectorX<T>>& boundary_state) {
    if (boundary_state.size() != ode_order_ + 1) {
      throw std::runtime_error(
          std::to_string(ode_order_ + 1) +
          " boundary states need to be specified. However, " +
          std::to_string(boundary_state.size()) +
          " boundary states were specified.");
    }
    bcs_[dof_index] = boundary_state;
  }

  /** Returns all boundary conditions stored in `this`
   %DirichletBoundaryCondition as a `std::map` with the index of the dof as
   key and the prescribed boundary values as value. */
  const std::map<DofIndex, VectorX<T>>& get_bcs() const { return bcs_; }

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
      (*residual)(int{dof_index}) = 0;
    }
  }

  /** Verifies that the largest index for the dofs under BC is smaller than the
   given `size`. Otherwise, throw an exception. */
  void VerifyBcIndexes(int size) const {
    const auto& last_bc = bcs_.crbegin();
    if (last_bc->first >= size) {
      throw std::runtime_error(
          "An index of the dirichlet boundary condition is out of the range.");
    }
  }

 private:
  /* We sort the boundary conditions according to dof indices for better
   cache consistency when applying the BC. The value of the map stores the
   value of q, qdot and qddot (in that order and when applicable) of the dof
   with index of the key. */
  std::map<DofIndex, VectorX<T>> bcs_{};

  /* The ODE order of the FemModel that `this` DirichletBoundaryCondition
   applies to. */
  const int ode_order_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
