#include "drake/multibody/fixed_fem/dev/fem_model_base.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T>
std::unique_ptr<FemStateBase<T>> FemModelBase<T>::MakeFemStateBase() const {
  return DoMakeFemStateBase();
}

template <typename T>
void FemModelBase<T>::CalcResidual(const FemStateBase<T>& state,
                                   EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  ThrowIfModelStateIncompatible(__func__, state);
  DoCalcResidual(state, residual);
  if (dirichlet_bc_ != nullptr) {
    dirichlet_bc_->ApplyBcToResidual(residual);
  }
}

template <typename T>
void FemModelBase<T>::CalcTangentMatrix(
    const FemStateBase<T>& state,
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == num_dofs());
  DRAKE_DEMAND(tangent_matrix->cols() == num_dofs());
  ThrowIfModelStateIncompatible(__func__, state);
  DoCalcTangentMatrix(state, tangent_matrix);
  if (dirichlet_bc_ != nullptr) {
    dirichlet_bc_->ApplyBcToTangentMatrix(tangent_matrix);
  }
}

template <typename T>
void FemModelBase<T>::SetTangentMatrixSparsityPattern(
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DoSetTangentMatrixSparsityPattern(tangent_matrix);
}

template <typename T>
const VectorX<T>& FemModelBase<T>::GetUnknowns(
    const FemStateBase<T>& state) const {
  ThrowIfModelStateIncompatible(__func__, state);
  return state_updater_->GetUnknowns(state);
}

template <typename T>
void FemModelBase<T>::UpdateStateFromChangeInUnknowns(
    const VectorX<T>& dz, FemStateBase<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(dz.size() == state->num_generalized_positions());
  ThrowIfModelStateIncompatible(__func__, *state);
  state_updater_->UpdateStateFromChangeInUnknowns(dz, state);
}

template <typename T>
void FemModelBase<T>::AdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                                         const VectorX<T>& unknown_variable,
                                         FemStateBase<T>* next_state) const {
  DRAKE_DEMAND(next_state != nullptr);
  DRAKE_DEMAND(unknown_variable.size() ==
               next_state->num_generalized_positions());
  DRAKE_THROW_UNLESS(ode_order() > 0);
  ThrowIfModelStateIncompatible(__func__, prev_state);
  ThrowIfModelStateIncompatible(__func__, *next_state);
  state_updater_->AdvanceOneTimeStep(prev_state, unknown_variable, next_state);
}

template <typename T>
void FemModelBase<T>::ApplyBoundaryCondition(FemStateBase<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  ThrowIfModelStateIncompatible(__func__, *state);
  if (dirichlet_bc_ != nullptr) {
    state->ApplyBoundaryCondition(*dirichlet_bc_);
  }
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemModelBase);
