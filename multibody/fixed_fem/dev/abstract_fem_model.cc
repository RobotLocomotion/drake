#include "drake/multibody/fixed_fem/dev/abstract_fem_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
template <typename T>
void AbstractFemModel<T>::CalcResidual(const AbstractFemState<T>& state,
                                       EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  DoCalcResidual(state, residual);
}

template <typename T>
void AbstractFemModel<T>::CalcTangentMatrix(
    const AbstractFemState<T>& state,
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == num_dofs());
  DRAKE_DEMAND(tangent_matrix->cols() == num_dofs());
  DoCalcTangentMatrix(state, tangent_matrix);
}

template <typename T>
void AbstractFemModel<T>::SetTangentMatrixSparsityPattern(
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DoSetTangentMatrixSparsityPattern(tangent_matrix);
}

template <typename T>
void AbstractFemModel<T>::UpdateState(const VectorX<T>& dz,
                                      AbstractFemState<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(dz.size() == state->num_generalized_positions());
  DoUpdateState(dz, state);
}

template <typename T>
void AbstractFemModel<T>::AdvanceOneTimeStep(
    const AbstractFemState<T>& prev_state,
    AbstractFemState<T>* next_state) const {
  DRAKE_DEMAND(next_state != nullptr);
  DRAKE_DEMAND(prev_state.num_generalized_positions() ==
               next_state->num_generalized_positions());
  DoAdvanceOneTimeStep(prev_state, next_state);
}

template <typename T>
void AbstractFemModel<T>::ApplyBoundaryConditions(
    AbstractFemState<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  DoApplyBoundaryConditions(state);
}
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fixed_fem::AbstractFemModel);
