#include "drake/multibody/fem/dirichlet_boundary_condition.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void DirichletBoundaryCondition<T>::AddBoundaryCondition(
    int dof_index, const Eigen::Ref<const Vector3<T>>& boundary_state) {
  index_to_boundary_state_[dof_index] = boundary_state;
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToState(
    FemState<T>* fem_state) const {
  DRAKE_DEMAND(fem_state != nullptr);
  if (index_to_boundary_state_.empty()) return;
  VerifyIndexes(fem_state->num_dofs());

  // TODO(xuchenhan-tri): Consider avoiding this copy.
  VectorX<T> q = fem_state->GetPositions();
  VectorX<T> v = fem_state->GetVelocities();
  VectorX<T> a = fem_state->GetAccelerations();
  for (const auto& [dof_index, boundary_state] : index_to_boundary_state_) {
    q(dof_index) = boundary_state(0);
    v(dof_index) = boundary_state(1);
    a(dof_index) = boundary_state(2);
  }
  fem_state->SetPositions(q);
  fem_state->SetVelocities(v);
  fem_state->SetAccelerations(a);
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToTangentMatrix(
    internal::PetscSymmetricBlockSparseMatrix* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == tangent_matrix->cols());
  if (index_to_boundary_state_.empty()) return;
  VerifyIndexes(tangent_matrix->cols());

  /* Zero out all rows and columns of the tangent matrix corresponding to
   dofs under the BC (except the diagonal entry which is set to 1). */
  std::vector<int> indexes(index_to_boundary_state_.size());
  int i = 0;
  for (const auto& it : index_to_boundary_state_) {
    indexes[i++] = it.first;
  }
  tangent_matrix->ZeroRowsAndColumns(indexes, /* diagonal entry */ 1.0);
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyHomogeneousBoundaryCondition(
    EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(v != nullptr);
  if (index_to_boundary_state_.empty()) return;
  VerifyIndexes(v->size());

  /* Zero out all entries of `v` corresponding to dofs under the BC. */
  for (const auto& it : index_to_boundary_state_) {
    const int dof_index = it.first;
    (*v)(dof_index) = 0.0;
  }
}

template <typename T>
void DirichletBoundaryCondition<T>::VerifyIndexes(int size) const {
  const auto& last_bc = index_to_boundary_state_.crbegin();
  if (last_bc->first >= size) {
    throw std::out_of_range(
        "An index of the Dirichlet boundary condition is out of "
        "range.");
  }
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DirichletBoundaryCondition);
