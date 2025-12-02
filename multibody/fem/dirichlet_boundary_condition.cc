#include "drake/multibody/fem/dirichlet_boundary_condition.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void DirichletBoundaryCondition<T>::AddBoundaryCondition(
    FemNodeIndex index, const NodeState<T>& boundary_state) {
  node_to_boundary_state_[index] = boundary_state;
  node_indices_.emplace(index);
}

template <typename T>
void DirichletBoundaryCondition<T>::Merge(
    const DirichletBoundaryCondition<T>& other) {
  if (this == &other) return;
  for (const auto& [index, state] : other.node_to_boundary_state_) {
    AddBoundaryCondition(index, state);
  }
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyBoundaryConditionToState(
    FemState<T>* fem_state) const {
  DRAKE_DEMAND(fem_state != nullptr);
  if (node_to_boundary_state_.empty()) return;
  VerifyIndices(fem_state->num_nodes());

  // TODO(xuchenhan-tri): Consider avoiding this copy.
  VectorX<T> q = fem_state->GetPositions();
  VectorX<T> v = fem_state->GetVelocities();
  VectorX<T> a = fem_state->GetAccelerations();
  for (const auto& [node_index, boundary_state] : node_to_boundary_state_) {
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
    contact_solvers::internal::BlockSparseSymmetricMatrix3d* tangent_matrix)
    const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  if (node_to_boundary_state_.empty()) return;
  VerifyIndices(tangent_matrix->cols() / 3);

  /* Zero out all rows and columns of the tangent matrix corresponding to
   DoFs under the BC (except the diagonal blocks which is set to be diagonal).
  */
  tangent_matrix->ZeroRowsAndColumns(node_indices_);
}

template <typename T>
void DirichletBoundaryCondition<T>::ApplyHomogeneousBoundaryCondition(
    EigenPtr<VectorX<T>> v) const {
  DRAKE_DEMAND(v != nullptr);
  if (node_to_boundary_state_.empty()) return;
  VerifyIndices(v->size() / 3);

  /* Zero out all entries of `v` corresponding to dofs under the BC. */
  for (const auto& it : node_to_boundary_state_) {
    const int node_index = it.first;
    v->template segment<3>(3 * node_index).setZero();
  }
}

template <typename T>
void DirichletBoundaryCondition<T>::VerifyIndices(int size) const {
  const auto& last_bc = node_to_boundary_state_.crbegin();
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

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::DirichletBoundaryCondition);
