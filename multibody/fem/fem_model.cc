#include "drake/multibody/fem/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
std::unique_ptr<FemState<T>> FemModel<T>::MakeFemState() const {
  return DoMakeFemState();
}

template <typename T>
void FemModel<T>::CalcResidual(const FemState<T>& state,
                                   EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  ThrowIfModelStateIncompatible(__func__, state);
  DoCalcResidual(state, residual);
  dirichlet_bc_.ApplyBoundaryConditionToResidual(residual);
}

template <typename T>
void FemModel<T>::CalcTangentMatrix(
    const FemState<T>& state, const Vector3<T>& weights,
    Eigen::SparseMatrix<T>* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == num_dofs());
  DRAKE_DEMAND(tangent_matrix->cols() == num_dofs());
  ThrowIfModelStateIncompatible(__func__, state);
  DoCalcTangentMatrix(state, weights, tangent_matrix);
  dirichlet_bc_.ApplyBoundaryConditionToTangentMatrix(tangent_matrix);
}

template <typename T>
void FemModel<T>::CalcTangentMatrix(
    const FemState<T>& state, const Vector3<T>& weights,
    internal::PetscSymmetricBlockSparseMatrix* tangent_matrix) const {
  DRAKE_DEMAND(tangent_matrix != nullptr);
  DRAKE_DEMAND(tangent_matrix->rows() == num_dofs());
  DRAKE_DEMAND(tangent_matrix->cols() == num_dofs());
  ThrowIfModelStateIncompatible(__func__, state);
  DoCalcTangentMatrix(state, weights, tangent_matrix);
  dirichlet_bc_.ApplyBoundaryConditionToTangentMatrix(tangent_matrix);
}

template <typename T>
Eigen::SparseMatrix<T> FemModel<T>::MakeEigenSparseTangentMatrix() const {
  return DoMakeEigenSparseTangentMatrix();
}

template <typename T>
std::unique_ptr<internal::PetscSymmetricBlockSparseMatrix>
FemModel<T>::MakePetscSymmetricBlockSparseTangentMatrix() const {
  return DoMakePetscSymmetricBlockSparseTangentMatrix();
}

template <typename T>
void FemModel<T>::ApplyBoundaryCondition(FemState<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  ThrowIfModelStateIncompatible(__func__, *state);
  dirichlet_bc_.ApplyBoundaryConditionToState(state);
}

template <typename T>
void FemModel<T>::SetGravityVector(const Vector3<T>& gravity) {
  /* Store gravity so that all elements added after the call to this method
   get the "new" gravity constant. */
  gravity_ = gravity;
  /* Update the gravity vector in elements added before the call to
   this method. */
  DoSetGravityVector(gravity);
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemModel);
