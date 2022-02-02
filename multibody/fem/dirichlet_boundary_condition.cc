#include "drake/multibody/fem/dirichlet_boundary_condition.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <class T>
void DirichletBoundaryCondition<T>::AddBoundaryCondition(
    int dof_index, const Eigen::Ref<const Vector3<T>>& boundary_state) {
  index_to_boundary_state_[dof_index] = boundary_state;
}

template <class T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToState(
    FemState<T>* state) const {
  if (index_to_boundary_state_.empty()) return;
  DRAKE_DEMAND(state != nullptr);
  VerifyIndexes(state->num_dofs());
  VectorX<T> q = state->GetPositions();
  VectorX<T> v = state->GetVelocities();
  VectorX<T> a = state->GetAccelerations();
  for (const auto& [dof_index, boundary_state] : index_to_boundary_state_) {
    q(dof_index) = boundary_state(0);
    v(dof_index) = boundary_state(1);
    a(dof_index) = boundary_state(2);
  }
  state->SetPositions(q);
  state->SetVelocities(v);
  state->SetAccelerations(a);
}

template <class T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToTangentMatrix(
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == tangent_matrix->cols());
  if (index_to_boundary_state_.empty()) {
    return;
  }
  /* Check validity of the DoF indexes stored. */
  VerifyIndexes(tangent_matrix->cols());

  /* Zero out all rows and columns of the tangent matrix corresponding to
   DoFs under the BC (except the diagonal entry which is set to 1). */
  for (const auto& it : index_to_boundary_state_) {
    const int dof_index = it.first;
    tangent_matrix->row(dof_index) *= T(0);
    tangent_matrix->col(dof_index) *= T(0);
    tangent_matrix->coeffRef(dof_index, dof_index) = T(1);
  }
}

template <class T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToTangentMatrix(
    internal::PetscSymmetricBlockSparseMatrix* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == tangent_matrix->cols());
  if (index_to_boundary_state_.empty()) {
    return;
  }
  /* Check validity of the DoF indexes stored. */
  VerifyIndexes(tangent_matrix->cols());

  /* Zero out all rows and columns of the tangent matrix corresponding to
   DoFs under the BC (except the diagonal entry which is set to 1). */
  std::vector<int> indexes(index_to_boundary_state_.size());
  int i = 0;
  for (const auto& it : index_to_boundary_state_) {
    indexes[i++] = it.first;
  }
  tangent_matrix->ZeroRowsAndColumns(indexes, /* diagonal entry */ 1.0);
}

template <class T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToResidual(
    EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  if (index_to_boundary_state_.empty()) {
    return;
  }
  /* Check validity of the DoF indices stored. */
  VerifyIndexes(residual->size());

  /* Zero out all entries of the residual corresponding to DoFs under the
   BC. */
  for (const auto& it : index_to_boundary_state_) {
    const int dof_index = it.first;
    (*residual)(dof_index) = 0;
  }
}

template <class T>
void DirichletBoundaryCondition<T>::VerifyIndexes(int size) const {
  const auto& last_bc = index_to_boundary_state_.crbegin();
  if (last_bc->first >= size) {
    throw std::runtime_error(
        "An index of the Dirichlet boundary condition is out of the "
        "range.");
  }
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DirichletBoundaryCondition);
