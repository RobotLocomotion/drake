#include "drake/multibody/fem/dev/fem_elasticity.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T, int NaturalDim>
FemElasticity<T, NaturalDim>::FemElasticity(
    ElementIndex element_index,
    const IsoparametricElement<T, NaturalDim>& shape,
    const Quadrature<T, NaturalDim>& quadrature,
    const std::vector<NodeIndex>& node_indices,
    const Eigen::Ref<const Matrix3X<T>>& reference_positions,
    const ConstitutiveModel<T>& constitutive_model)
    : FemElement<T, NaturalDim>(element_index, shape, quadrature, node_indices),
      constitutive_model_(constitutive_model),
      dxidX_(num_quads()),
      volume_(num_quads()) {
  DRAKE_DEMAND(num_nodes() == reference_positions.cols());
  // Record the quadrature point volumes for the new element.
  std::vector<MatrixX<T>> dXdxi = shape.CalcJacobian(reference_positions);
  for (int q = 0; q < num_quads(); ++q) {
    // Degenerate tetrahedron in the initial configuration is not allowed.
    T det = dXdxi[q].determinant();
    DRAKE_DEMAND(det > 0);
    volume_[q] = det * quadrature.get_weight(q);
  }

  // Record the inverse Jacobian at the reference configuration which is used in
  // the calculation of deformation gradient.
  auto dxidX = shape.CalcJacobianInverse(dXdxi);
  for (int q = 0; q < num_quads(); ++q) {
    dxidX_[q] = Eigen::Ref<MatrixD3>(dxidX[q]);
  }
}

template <typename T, int NaturalDim>
T FemElasticity<T, NaturalDim>::CalcElasticEnergy(const FemState<T>& s) const {
  T elastic_energy = 0;
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  std::vector<T> Psi(num_quads());
  CalcPsi(s, &Psi);
  for (int q = 0; q < num_quads(); ++q) {
    elastic_energy += volume_[q] * Psi[q];
  }
  return elastic_energy;
}

template <typename T, int NaturalDim>
void FemElasticity<T, NaturalDim>::DoCalcResidual(
    const FemState<T>& state, EigenPtr<VectorX<T>> residual) const {
  CalcElasticForce(state, residual);
}

template <typename T, int NaturalDim>
void FemElasticity<T, NaturalDim>::CalcElasticForce(
    const FemState<T>& state, EigenPtr<VectorX<T>> force) const {
  force->setZero();
  auto force_matrix = Eigen::Map<Matrix3X<T>>(force->data(), 3, num_nodes());
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  std::vector<Matrix3<T>> P(num_quads());
  CalcP(state, &P);
  const auto dSdxi = shape_.CalcGradientInParentCoordinates();
  for (int q = 0; q < num_quads(); ++q) {
    force_matrix -=
        volume_[q] * P[q] * dxidX_[q].transpose() * dSdxi[q].transpose();
  }
}

template <typename T, int NaturalDim>
void FemElasticity<T, NaturalDim>::CalcF(const FemState<T>& state,
                                         std::vector<Matrix3<T>>* F) const {
  F->resize(num_quads());
  Matrix3X<T> element_x(3, num_nodes());
  const auto& x_tmp = state.x();
  const auto& x =
      Eigen::Map<const Matrix3X<T>>(x_tmp.data(), 3, x_tmp.size() / 3);
  for (int i = 0; i < num_nodes(); ++i) {
    element_x.col(i) = x.col(node_indices_[i]);
  }
  auto dxdxi = shape_.CalcJacobian(element_x);
  for (int q = 0; q < num_quads(); ++q) {
    (*F)[q] = dxdxi[q] * dxidX_[q];
  }
}

template <typename T, int NaturalDim>
const std::unique_ptr<DeformationGradientCache<T>>&
FemElasticity<T, NaturalDim>::CalcDeformationGradientCache(
    const FemState<T>& state) const {
  ElasticityElementCache<T>& mutable_cache =
      static_cast<ElasticityElementCache<T>&>(
          state.mutable_cache_at(element_index_));
  auto& deformation_gradient_cache =
      mutable_cache.mutable_deformation_gradient_cache();
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  std::vector<Matrix3<T>> F(num_quads());
  CalcF(state, &F);
  deformation_gradient_cache->UpdateCache(F);
  return deformation_gradient_cache;
}

template <typename T, int NaturalDim>
void FemElasticity<T, NaturalDim>::CalcPsi(const FemState<T>& state,
                                           std::vector<T>* Psi) const {
  Psi->resize(num_quads());
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  const auto& deformation_gradient_cache = CalcDeformationGradientCache(state);
  constitutive_model_.CalcPsi(*deformation_gradient_cache, Psi);
}

template <typename T, int NaturalDim>
void FemElasticity<T, NaturalDim>::CalcP(const FemState<T>& state,
                                         std::vector<Matrix3<T>>* P) const {
  P->resize(num_quads());
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  const auto& deformation_gradient_cache = CalcDeformationGradientCache(state);
  constitutive_model_.CalcP(*deformation_gradient_cache, P);
}
template class FemElasticity<double, 2>;
template class FemElasticity<double, 3>;
template class FemElasticity<AutoDiffXd, 2>;
template class FemElasticity<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
