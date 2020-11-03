#include "drake/multibody/fem/dev/fem_elasticity.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T, int NaturalDim>
ElasticityElement<T, NaturalDim>::ElasticityElement(
    ElementIndex element_index,
    std::unique_ptr<IsoparametricElement<T, NaturalDim>> shape,
    std::unique_ptr<Quadrature<T, NaturalDim>> quadrature,
    const std::vector<NodeIndex>& node_indices,
    const ElasticityElementParameters<T>& param,
    std::unique_ptr<ConstitutiveModel<T>> constitutive_model)
    : FemElement<T, NaturalDim>(element_index, std::move(shape),
                                std::move(quadrature), node_indices),
      param_(param),
      constitutive_model_(std::move(constitutive_model)),
      dxidX_(num_quads()),
      reference_volume_(num_quads()) {
  DRAKE_DEMAND(num_nodes() == param_.reference_positions.cols());
  // Record the quadrature point volumes for the new element.
  const std::vector<MatrixX<T>> dXdxi =
      shape_->CalcJacobian(param_.reference_positions);
  for (int q = 0; q < num_quads(); ++q) {
    // Degenerate tetrahedron in the initial configuration is not allowed.
    T det = dXdxi[q].determinant();
    DRAKE_DEMAND(det > 0);
    reference_volume_[q] = det * this->quadrature_->get_weight(q);
  }

  // Record the inverse Jacobian at the reference configuration which is used in
  // the calculation of deformation gradient.
  const std::vector<MatrixX<T>> dxidX = shape_->CalcJacobianInverse(dXdxi);
  for (int q = 0; q < num_quads(); ++q) {
    dxidX_[q] = Eigen::Ref<const MatrixD3>(dxidX[q]);
  }
}

template <typename T, int NaturalDim>
T ElasticityElement<T, NaturalDim>::CalcElasticEnergy(
    const FemState<T>& s) const {
  T elastic_energy = 0;
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  std::vector<T> Psi(num_quads());
  CalcElasticEnergyDensity(s, &Psi);
  for (int q = 0; q < num_quads(); ++q) {
    elastic_energy += reference_volume_[q] * Psi[q];
  }
  return elastic_energy;
}

template <typename T, int NaturalDim>
void ElasticityElement<T, NaturalDim>::DoCalcResidual(
    const FemState<T>& state, EigenPtr<VectorX<T>> residual) const {
  // TODO(xuchenhan-tri): Add gravity and inertia terms when mass matrix is in
  // place.
  // TODO(xuchenhan-tri): Add damping force.
  CalcNegativeElasticForce(state, residual);
}

template <typename T, int NaturalDim>
void ElasticityElement<T, NaturalDim>::CalcNegativeElasticForce(
    const FemState<T>& state, EigenPtr<VectorX<T>> neg_force) const {
  neg_force->setZero();
  auto neg_force_matrix =
      Eigen::Map<Matrix3X<T>>(neg_force->data(), 3, num_nodes());
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  std::vector<Matrix3<T>> P(num_quads());
  CalcFirstPiolaStress(state, &P);
  const std::vector<MatrixX<T>>& dSdxi =
      shape_->CalcGradientInParentCoordinates();
  for (int q = 0; q < num_quads(); ++q) {
    // Negative force is the gradient of energy.
    // -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
    neg_force_matrix += reference_volume_[q] * P[q] * dxidX_[q].transpose() *
                        dSdxi[q].transpose();
  }
}

template <typename T, int NaturalDim>
void ElasticityElement<T, NaturalDim>::CalcDeformationGradient(
    const FemState<T>& state, std::vector<Matrix3<T>>* F) const {
  F->resize(num_quads());
  // TODO(xuchenhan-tri): Consider abstracting this potential common operation
  // into FemElement.
  Matrix3X<T> element_x(3, num_nodes());
  const VectorX<T>& x_tmp = state.q();
  const auto& x =
      Eigen::Map<const Matrix3X<T>>(x_tmp.data(), 3, x_tmp.size() / 3);
  for (int i = 0; i < num_nodes(); ++i) {
    element_x.col(i) = x.col(node_indices_[i]);
  }
  const std::vector<MatrixX<T>> dxdxi = shape_->CalcJacobian(element_x);
  for (int q = 0; q < num_quads(); ++q) {
    (*F)[q] = dxdxi[q] * dxidX_[q];
  }
}

template <typename T, int NaturalDim>
const DeformationGradientCache<T>&
ElasticityElement<T, NaturalDim>::CalcDeformationGradientCache(
    const FemState<T>& state) const {
  ElasticityElementCache<T>& mutable_cache =
      static_cast<ElasticityElementCache<T>&>(
          state.mutable_element_cache(element_index_));
  DeformationGradientCache<T>& deformation_gradient_cache =
      mutable_cache.mutable_deformation_gradient_cache();
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  std::vector<Matrix3<T>> F(num_quads());
  CalcDeformationGradient(state, &F);
  deformation_gradient_cache.UpdateCache(F);
  return deformation_gradient_cache;
}

template <typename T, int NaturalDim>
void ElasticityElement<T, NaturalDim>::CalcElasticEnergyDensity(
    const FemState<T>& state, std::vector<T>* Psi) const {
  Psi->resize(num_quads());
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  const DeformationGradientCache<T>& deformation_gradient_cache =
      CalcDeformationGradientCache(state);
  constitutive_model_->CalcElasticEnergyDensity(deformation_gradient_cache,
                                                Psi);
}

template <typename T, int NaturalDim>
void ElasticityElement<T, NaturalDim>::CalcFirstPiolaStress(
    const FemState<T>& state, std::vector<Matrix3<T>>* P) const {
  P->resize(num_quads());
  // TODO(xuchenhan-tri): Use the corresponding Eval method with cache is in
  // place.
  const DeformationGradientCache<T>& deformation_gradient_cache =
      CalcDeformationGradientCache(state);
  constitutive_model_->CalcFirstPiolaStress(deformation_gradient_cache, P);
}
template class ElasticityElement<double, 2>;
template class ElasticityElement<double, 3>;
template class ElasticityElement<AutoDiffXd, 2>;
template class ElasticityElement<AutoDiffXd, 3>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
