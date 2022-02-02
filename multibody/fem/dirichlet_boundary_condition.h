#pragma once

#include <map>
#include <vector>

#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/petsc_symmetric_block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* DirichletBoundaryCondition provides functionalities related to Dirichlet
 boundary conditions (BC) on FEM models. In particular, it provides the
 following functionalities:
 1. storing the information necessary to apply the BC;
 2. modifying a given state to comply with the stored BC;
 3. modifying a given tangent matrix/residual that arises from the FEM model
 without BC and transform it into the tangent matrix/residual for the same
 model under the stored BC.
 @tparam_nonsymbolic_scalar */
template <class T>
class DirichletBoundaryCondition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirichletBoundaryCondition);

  /* Constructs an empty boundary condition. */
  DirichletBoundaryCondition() {}

  /* Sets the DoF with index `dof_index` to be subject to the prescribed
   `boundary_state`.
   @param[in] dof_index      The index of the degree of freedom (DoF) to which
                             the boundary condition is applied.
   @param[in] boundary_state The prescibed position, velocity, and acceleration
                             at `dof_index`. */
  void AddBoundaryCondition(int dof_index,
                            const Eigen::Ref<const Vector3<T>>& boundary_state);

  /* Returns the boundary conditions as an `std::map` with the index of the DoF
   as the key and the prescribed boundary states as the value. */
  const std::map<int, Vector3<T>>& index_to_boundary_state() const {
    return index_to_boundary_state_;
  }

  /* Modifies the given `state` to comply with the Dirichlet boundary
   conditions.
   @pre state != nullptr. */
  void ApplyBoundaryConditionToState(FemState<T>* state) const;

  /* Modifies the given tangent matrix that arises from an FEM model without
   BC into the tangent matrix for the same model subject to `this` BC. More
   specifically, the rows and columns corresponding to DoFs under the BC will
   be zeroed out with the exception of the diagonal entries for those DoFs
   which will be set to 1.
   @pre tangent_matrix != nullptr.
   @pre tangent_matrix->rows() == tangent_matrix->cols().
   @throw std::exception if the any of the indexes of the DoFs under the
   boundary condition specified by `this` DirichletBoundaryCondition is
   greater than or equal to the `tangent_matrix->cols()`. */
  void ApplyBoundaryConditionToTangentMatrix(
      Eigen::SparseMatrix<T>* tangent_matrix) const;

  /* Alternative sigature for ApplyBoundaryConditionToTangentMatrix() that
   modifies a PETSc matrix.
   @pre tangent_matrix != nullptr.
   @pre tangent_matrix->rows() == tangent_matrix->cols().
   @throw std::exception if the any of the indexes of the DoFs under the
   boundary condition specified by `this` DirichletBoundaryCondition is
   greater than or equal to the `tangent_matrix->cols()`. */
  void ApplyBoundaryConditionToTangentMatrix(
      internal::PetscSymmetricBlockSparseMatrix* tangent_matrix) const;

  /* Modifies the given residual that arises from an FEM model without BC into
   the residual for the same model subject to `this` BC. More specifically,
   the entries corresponding to DoFs under the BC will be zeroed out.
   @pre residual != nullptr.
   @throw std::exception if any of the indexes of the DoFs under the boundary
   condition specified by `this` %DirichletBoundaryCondition  is greater than
   or equal to the `residual->size()`. */
  void ApplyBoundaryConditionToResidual(EigenPtr<VectorX<T>> residual) const;

 private:
  /* Verifies that the largest index for the DoFs under BC is smaller than the
   given `size`. Otherwise, throw an exception. */
  void VerifyIndexes(int size) const;

  /* We sort the boundary conditions according to DoF indices for better
   cache consistency when applying the BC. The value of the map stores the
   value of q, v, and a (in that order and when applicable) of the DoF
   with index of the key. */
  std::map<int, Vector3<T>> index_to_boundary_state_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DirichletBoundaryCondition);
