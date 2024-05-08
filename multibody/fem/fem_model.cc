#include "drake/multibody/fem/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
void FemModel<T>::Builder::Build() {
  ThrowIfBuilt();
  DoBuild();
  model_->UpdateFemStateSystem();
  built_ = true;
}

template <typename T>
void FemModel<T>::Builder::ThrowIfBuilt() const {
  if (built_) {
    throw std::logic_error(
        "Build() has been called on this Builder. Create a new Builder if you "
        "need to add more elements to the FEM model.");
  }
}

template <typename T>
FemModel<T>::~FemModel() = default;

template <typename T>
std::unique_ptr<FemModel<T>> FemModel<T>::Clone() const {
  std::unique_ptr<FemModel<T>> result = this->DoClone();
  result->UpdateFemStateSystem();
  result->dirichlet_bc_ = this->dirichlet_bc_;
  return result;
}

template <typename T>
std::unique_ptr<FemState<T>> FemModel<T>::MakeFemState() const {
  return std::make_unique<FemState<T>>(fem_state_system_.get());
}

template <typename T>
void FemModel<T>::CalcResidual(const FemState<T>& fem_state,
                               const FemPlantData<T>& plant_data,
                               EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  DRAKE_DEMAND(residual->size() == num_dofs());
  ThrowIfModelStateIncompatible(__func__, fem_state);
  DoCalcResidual(fem_state, plant_data, residual);
  dirichlet_bc_.ApplyHomogeneousBoundaryCondition(residual);
}

template <typename T>
void FemModel<T>::CalcTangentMatrix(
    const FemState<T>& fem_state, const Vector3<T>& weights,
    contact_solvers::internal::Block3x3SparseSymmetricMatrix* tangent_matrix)
    const {
  if constexpr (std::is_same_v<T, double>) {
    DRAKE_DEMAND(tangent_matrix != nullptr);
    DRAKE_DEMAND(tangent_matrix->rows() == num_dofs());
    DRAKE_DEMAND(tangent_matrix->cols() == num_dofs());
    DRAKE_THROW_UNLESS(weights.minCoeff() >= 0.0);
    ThrowIfModelStateIncompatible(__func__, fem_state);
    DoCalcTangentMatrix(fem_state, weights, tangent_matrix);
    dirichlet_bc_.ApplyBoundaryConditionToTangentMatrix(tangent_matrix);
  } else {
    throw std::logic_error(
        "FemModel::CalcTangentMatrix() only supports double at the moment.");
  }
}

template <typename T>
std::unique_ptr<contact_solvers::internal::Block3x3SparseSymmetricMatrix>
FemModel<T>::MakeTangentMatrix() const {
  if constexpr (std::is_same_v<T, double>) {
    return DoMakeTangentMatrix();
  } else {
    throw std::logic_error(
        "FemModel::MakeTangentMatrix() only supports double at the moment.");
  }
}

template <typename T>
void FemModel<T>::ApplyBoundaryCondition(FemState<T>* fem_state) const {
  DRAKE_DEMAND(fem_state != nullptr);
  ThrowIfModelStateIncompatible(__func__, *fem_state);
  dirichlet_bc_.ApplyBoundaryConditionToState(fem_state);
}

template <typename T>
FemModel<T>::FemModel()
    : fem_state_system_(std::make_unique<internal::FemStateSystem<T>>(
          VectorX<T>(0), VectorX<T>(0), VectorX<T>(0))) {}

template <typename T>
void FemModel<T>::ThrowIfModelStateIncompatible(
    const char* func, const FemState<T>& fem_state) const {
  if (!is_compatible_with(fem_state)) {
    throw std::logic_error(std::string(func) +
                           "(): The FEM model and state are not compatible.");
  }
}

template <typename T>
void FemModel<T>::UpdateFemStateSystem() {
  VectorX<T> model_positions = MakeReferencePositions();
  VectorX<T> model_velocities = VectorX<T>::Zero(model_positions.size());
  VectorX<T> model_accelerations = VectorX<T>::Zero(model_positions.size());
  fem_state_system_ = std::make_unique<internal::FemStateSystem<T>>(
      model_positions, model_velocities, model_accelerations);
  DeclareCacheEntries(fem_state_system_.get());
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::FemModel);
