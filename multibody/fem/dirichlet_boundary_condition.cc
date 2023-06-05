#include "drake/multibody/fem/dirichlet_boundary_condition.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void DirichletBoundaryCondition<T>::AddBoundaryCondition(
    FemNodeIndex index, const NodeState<T>& boundary_state) {
  index_to_boundary_state_[index] = boundary_state;
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToState(
    FemState<T>* fem_state) const {
  DRAKE_DEMAND(fem_state != nullptr);
  if (index_to_boundary_state_.empty()) return;
  VerifyIndices(fem_state->num_nodes());

  // TODO(xuchenhan-tri): Consider avoiding this copy.
  VectorX<T> q = fem_state->GetPositions();
  VectorX<T> v = fem_state->GetVelocities();
  VectorX<T> a = fem_state->GetAccelerations();
  for (const auto& [node_index, boundary_state] : index_to_boundary_state_) {
    q.template segment<3>(3 * node_index) = boundary_state.q;
    v.template segment<3>(3 * node_index) = boundary_state.v;
    a.template segment<3>(3 * node_index) = boundary_state.a;
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
  VerifyIndices(tangent_matrix->cols() / 3);

  /* Zero out all rows and columns of the tangent matrix corresponding to
   DoFs under the BC (except the diagonal entry which is set to 1). */
  std::vector<int> dof_indices(3 * index_to_boundary_state_.size());
  int i = 0;
  for (const auto& it : index_to_boundary_state_) {
    dof_indices[i++] = 3 * it.first;
    dof_indices[i++] = 3 * it.first + 1;
    dof_indices[i++] = 3 * it.first + 2;
  }
  tangent_matrix->ZeroRowsAndColumns(dof_indices, /* diagonal entry */ 1.0);
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToTangentMatrix(
    contact_solvers::internal::Block3x3SparseSymmetricMatrix* tangent_matrix)
    const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  if (index_to_boundary_state_.empty()) return;
  VerifyIndices(tangent_matrix->cols() / 3);

  /* Zero out all rows and columns of the tangent matrix corresponding to
   DoFs under the BC (except the diagonal blocks which is set to be diagonal).
  */
  std::vector<int> node_indices(index_to_boundary_state_.size());
  int i = 0;
  for (const auto& it : index_to_boundary_state_) {
    node_indices[i++] = it.first;
  }
  tangent_matrix->ZeroRowsAndColumns(node_indices);
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyHomogeneousBoundaryCondition(
    EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(v != nullptr);
  if (index_to_boundary_state_.empty()) return;
  VerifyIndices(v->size() / 3);

  /* Zero out all entries of `v` corresponding to dofs under the BC. */
  for (const auto& it : index_to_boundary_state_) {
    const int node_index = it.first;
    v->template segment<3>(3 * node_index).setZero();
  }
}

template <typename T>
void DirichletBoundaryCondition<T>::VerifyIndices(int size) const {
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
