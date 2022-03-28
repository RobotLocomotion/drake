#pragma once

#include <map>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/fem_state.h"
#include "drake/multibody/fem/petsc_symmetric_block_sparse_matrix.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* DirichletBoundaryCondition provides functionalities related to Dirichlet
 boundary conditions (BC) on FEM models [Zienkiewicz et al. Sec 1.4]. In
 particular, it provides the following functionalities:
 1. storing the information necessary to apply the BC;
 2. modifying a given state to comply with the stored BC;
 3. modifying a given tangent matrix/residual that arises from the FEM model
 without BC and transform it into the tangent matrix/residual for the same
 model under the stored BC.

 Zienkiewicz, Olek C., Robert L. Taylor, and Jian Z. Zhu. The finite element
 method: its basis and fundamentals. Elsevier, 2005.

 @tparam_nonsymbolic_scalar */
template <typename T>
class DirichletBoundaryCondition {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DirichletBoundaryCondition);

  /* Constructs an empty boundary condition. */
  DirichletBoundaryCondition() {}

  /* Sets the degree of freemdom (dof) with index `dof_index` to be subject to
   the prescribed `boundary_state`.
   @param[in] dof_index       The index of the degree of freedom to which the
                              boundary condition is applied.
   @param[in] boundary_state  The position, velocity, and acceleration at
                              `dof_index` prescribed by this boundary condition.
  */
  void AddBoundaryCondition(int dof_index,
                            const Eigen::Ref<const Vector3<T>>& boundary_state);

  /* Returns the boundary conditions as an `std::map` with dof indexes as keys
   and the prescribed boundary states as values. */
  const std::map<int, Vector3<T>>& index_to_boundary_state() const {
    return index_to_boundary_state_;
  }

  /* Modifies the given `fem_state` to comply with this boundary condition.
   @pre fem_state != nullptr.
   @pre fem_state is an "owned" FEM state (see FemState).
   @throw std::exception if any dof index specified by `this` boundary
   condition is greater than or equal to the `fem_state->num_dofs()`. */
  void ApplyBoundaryConditionToState(FemState<T>* fem_state) const;

  /* Modifies the given tangent matrix that arises from an FEM model into the
   tangent matrix for the same model subject to this BC. More specifically,
   the rows and columns corresponding to dofs under this BC will be zeroed out
   with the exception of the diagonal entries for those dofs which will be set
   to 1.
   @pre tangent_matrix != nullptr.
   @pre tangent_matrix->rows() == tangent_matrix->cols().
   @throw std::exception if any dof index specified by `this` boundary
   condition is greater than or equal to the `tangent_matrix->cols()`. */
  void ApplyBoundaryConditionToTangentMatrix(
      PetscSymmetricBlockSparseMatrix* tangent_matrix) const;

  /* Modifies the given vector `v` (e.g, the residual of the system or the
   velocities/positions) that arises from an FEM model without BC into the a
   vector for the same model subject to `this` BC. More specifically, the
   entries corresponding to dofs under the BC will be zeroed out.
   @pre v != nullptr.
   @throw std::exception if any dof index specified by `this` boundary
   condition is greater than or equal to the `v->size()`. */
  void ApplyHomogeneousBoundaryCondition(EigenPtr<VectorX<T>> v) const;

 private:
  /* Throws an exception if the largest dof index specified by this BC is larger
   than or equal to the given `size`. */
  void VerifyIndexes(int size) const;

  /* We sort the boundary conditions according to dof indexes for better
   cache consistency when applying the BC. The value stored is q, v, and a (in
   that order) of the dof under BC. */
  std::map<int, Vector3<T>> index_to_boundary_state_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DirichletBoundaryCondition);
