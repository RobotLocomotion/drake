#include "drake/multibody/fem/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
std::unique_ptr<FemState<T>> FemModel<T>::MakeFemState() const {
  DRAKE_DEMAND(num_dofs() == fem_state_system_->num_dofs());
  return std::make_unique<FemState<T>>(fem_state_system_.get());
}

template <typename T>
void FemModel<T>::CalcResidual(const FemState<T>& fem_state,
                               EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  ThrowIfModelDataIncompatible(__func__, fem_state);
  DoCalcResidual(fem_state, residual);
}

template <typename T>
void FemModel<T>::CalcTangentMatrix(
    const FemState<T>& fem_state, const Vector3<T>& weights,
    internal::PetscSymmetricBlockSparseMatrix* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == num_dofs());
  DRAKE_DEMAND(tangent_matrix->cols() == num_dofs());
  ThrowIfModelDataIncompatible(__func__, fem_state);
  DoCalcTangentMatrix(fem_state, weights, tangent_matrix);
}

template <typename T>
std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix>
FemModel<T>::MakePetscSymmetricBlockSparseTangentMatrix() const {
  return DoMakePetscSymmetricBlockSparseTangentMatrix();
}

template <typename T>
void FemModel<T>::ThrowIfModelDataIncompatible(
    const char* func, const FemState<T>& fem_state) const {
  if (!fem_state.is_created_from_system(*fem_state_system_)) {
    throw std::logic_error(std::string(func) +
                           "(): The FEM model and state are not compatible.");
  }
}

template <typename T>
void FemModel<T>::UpdateFemStateSystem() {
  VectorX<T> model_positions = MakeReferencePositions();
  VectorX<T> model_velocities = VectorX<T>::Zero(num_dofs());
  VectorX<T> model_accelerations = VectorX<T>::Zero(num_dofs());
  fem_state_system_ = std::make_unique<internal::FemStateSystem<T>>(
      model_positions, model_velocities, model_accelerations);
  DeclareCacheEntries(fem_state_system_.get());
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemModel);
