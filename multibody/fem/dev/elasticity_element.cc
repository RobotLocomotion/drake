#include "drake/multibody/fem/dev/elasticity_element.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/dev/elasticity_element_cache.h"
#include "drake/multibody/fem/dev/linear_simplex_element.h"

namespace drake {
namespace multibody {
namespace fem {
template <typename T, class IsoparametricElementType, class QuadratureType>
ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    ElasticityElement(ElementIndex element_index,
                      const std::vector<NodeIndex>& node_indices,
                      const T& density,
                      std::unique_ptr<ConstitutiveModel<T>> constitutive_model,
                      const Matrix3X<T>& reference_positions)
    : ElasticityElementBase<T>(element_index, node_indices),
      density_(density),
      constitutive_model_(std::move(constitutive_model)),
      dxidX_(num_quadrature_points()),
      reference_positions_(reference_positions),
      reference_volume_(num_quadrature_points()) {
  /* TODO(xuchenhan-tri): Consider removing the template NaturalDim from
   IsoparametricElement and Quadrature (e.g. with CRTP). */
  static_assert(is_isoparametric_element<IsoparametricElementType>::value,
      "IsoparametricElementType must be a derived class of "
      "IsoparametricElement<T, NaturalDim>, where NaturalDim can "
      "be 1, 2 or 3.");
  static_assert(is_quadrature<QuadratureType>::value,
                "QuadratureType must be a derived class of "
                "Quadrature<T, NaturalDim>, where NaturalDim can "
                "be 1, 2 or 3.");
  static_assert(
      IsoparametricElementType::kNaturalDim == QuadratureType::kNaturalDim,
      "The dimension of the parent domain for IsoparametricElement and "
      "Quadrature must be the same.");
  DRAKE_ASSERT(num_nodes() == reference_positions.cols());
  DRAKE_ASSERT(num_nodes() == static_cast<int>(node_indices.size()));
  // Record the quadrature point volumes for the new element.
  const std::vector<MatrixX<T>> dXdxi =
      shape_.CalcJacobian(reference_positions);
  for (int q = 0; q < num_quadrature_points(); ++q) {
    // The scale to transform quadrature weight in parent coordinates to
    // reference coordinates.
    T volume_scale;
    if (kNaturalDim == 3) {
      volume_scale = dXdxi[q].determinant();
      // Degenerate tetrahedron in the initial configuration is not allowed.
      DRAKE_ASSERT(volume_scale > 0);
    } else if (kNaturalDim == 2) {
      Eigen::ColPivHouseholderQR<MatrixX<T>> qr(dXdxi[q]);
      volume_scale = abs(qr.matrixR()
                             .topLeftCorner(kNaturalDim, kNaturalDim)
                             .template triangularView<Eigen::Upper>()
                             .determinant());
    } else if (kNaturalDim == 1) {
      volume_scale = dXdxi[q].norm();
    } else {
      DRAKE_UNREACHABLE();
    }
    reference_volume_[q] = volume_scale * quadrature_.get_weight(q);
  }

  // Record the inverse Jacobian at the reference configuration which is used in
  // the calculation of deformation gradient.
  const std::vector<MatrixX<T>> dxidX = shape_.CalcJacobianInverse(dXdxi);
  for (int q = 0; q < num_quadrature_points(); ++q) {
    dxidX_[q] = Eigen::Ref<const MatrixD3>(dxidX[q]);
  }
}

template <typename T, class IsoparametricElementType, class QuadratureType>
std::unique_ptr<ElementCache<T>> ElasticityElement<
    T, IsoparametricElementType, QuadratureType>::MakeElementCache() const {
  std::unique_ptr<DeformationGradientCache<T>> deformation_gradient_cache =
      constitutive_model_->MakeDeformationGradientCache(
          this->element_index(), this->num_quadrature_points());
  return std::make_unique<ElasticityElementCache<T>>(
      this->element_index(), this->num_quadrature_points(),
      std::move(deformation_gradient_cache));
}

template <typename T, class IsoparametricElementType, class QuadratureType>
T ElasticityElement<T, IsoparametricElementType,
                    QuadratureType>::CalcElasticEnergy(const FemState<T>& s)
    const {
  T elastic_energy = 0;
  // TODO(xuchenhan-tri): Use the corresponding Eval method when cache is in
  // place.
  // TODO(xuchenhan-tri): Use fixed size array here.
  std::vector<T> Psi(num_quadrature_points());
  CalcElasticEnergyDensity(s, &Psi);
  for (int q = 0; q < num_quadrature_points(); ++q) {
    elastic_energy += reference_volume_[q] * Psi[q];
  }
  return elastic_energy;
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType,
                       QuadratureType>::DoCalcResidual(const FemState<T>& state,
                                                       EigenPtr<VectorX<T>>
                                                           residual) const {
  // TODO(xuchenhan-tri): Add gravity and inertia terms when mass matrix is in
  // place.
  // TODO(xuchenhan-tri): Add damping force.
  CalcNegativeElasticForce(state, residual);
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcNegativeElasticForce(const FemState<T>& state,
                             EigenPtr<VectorX<T>> neg_force) const {
  neg_force->setZero();
  auto neg_force_matrix =
      Eigen::Map<Matrix3X<T>>(neg_force->data(), 3, num_nodes());
  // TODO(xuchenhan-tri): Use the corresponding Eval method when cache is in
  // place.
  std::vector<Matrix3<T>> P(num_quadrature_points());
  CalcFirstPiolaStress(state, &P);
  const std::vector<MatrixX<T>>& dSdxi =
      shape_.CalcGradientInParentCoordinates();
  for (int q = 0; q < num_quadrature_points(); ++q) {
    /* Negative force is the gradient of energy.
     -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
     Notice that Fᵢⱼ = xₐᵢdSₐ/dXⱼ, so dFᵢⱼ/dxᵦₖ = δₐᵦδᵢₖdSₐ/dXⱼ,
     and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
     PᵢⱼδₐᵦδᵢₖdSₐ/dXⱼ = PₖⱼdSᵦ/dXⱼ = P * dSdX.transpose() */
    const auto& dSdX = dSdxi[q] * dxidX_[q];
    neg_force_matrix += reference_volume_[q] * P[q] * dSdX.transpose();
  }
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcDeformationGradient(const FemState<T>& state,
                            std::vector<Matrix3<T>>* F) const {
  F->resize(num_quadrature_points());
  // TODO(xuchenhan-tri): Consider abstracting this potential common operation
  // into FemElement.
  Matrix3X<T> element_x(3, num_nodes());
  const VectorX<T>& x_tmp = state.q();
  const auto& x =
      Eigen::Map<const Matrix3X<T>>(x_tmp.data(), 3, x_tmp.size() / 3);
  for (int i = 0; i < num_nodes(); ++i) {
    element_x.col(i) = x.col(this->node_indices()[i]);
  }
  const std::vector<MatrixX<T>> dxdxi = shape_.CalcJacobian(element_x);
  for (int q = 0; q < num_quadrature_points(); ++q) {
    (*F)[q] = dxdxi[q] * dxidX_[q];
  }
}

template <typename T, class IsoparametricElementType, class QuadratureType>
const DeformationGradientCache<T>&
ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    EvalDeformationGradientCache(const FemState<T>& state) const {
  ElasticityElementCache<T>& mutable_cache =
      static_cast<ElasticityElementCache<T>&>(
          state.mutable_element_cache(this->element_index()));
  DeformationGradientCache<T>& deformation_gradient_cache =
      mutable_cache.mutable_deformation_gradient_cache();
  // TODO(xuchenhan-tri): Enable caching when caching is in place.
  std::vector<Matrix3<T>> F(num_quadrature_points());
  CalcDeformationGradient(state, &F);
  deformation_gradient_cache.UpdateCache(F);
  return deformation_gradient_cache;
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcElasticEnergyDensity(const FemState<T>& state,
                             std::vector<T>* Psi) const {
  Psi->resize(num_quadrature_points());
  const DeformationGradientCache<T>& deformation_gradient_cache =
      EvalDeformationGradientCache(state);
  constitutive_model_->CalcElasticEnergyDensity(deformation_gradient_cache,
                                                Psi);
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcFirstPiolaStress(const FemState<T>& state,
                         std::vector<Matrix3<T>>* P) const {
  P->resize(num_quadrature_points());
  const DeformationGradientCache<T>& deformation_gradient_cache =
      EvalDeformationGradientCache(state);
  constitutive_model_->CalcFirstPiolaStress(deformation_gradient_cache, P);
}
template class ElasticityElement<double, LinearSimplexElement<double, 3>,
                                 SimplexGaussianQuadrature<double, 1, 3>>;
template class ElasticityElement<AutoDiffXd,
                                 LinearSimplexElement<AutoDiffXd, 3>,
                                 SimplexGaussianQuadrature<AutoDiffXd, 1, 3>>;
template class ElasticityElement<double, LinearSimplexElement<double, 3>,
                                 SimplexGaussianQuadrature<double, 2, 3>>;
template class ElasticityElement<AutoDiffXd,
                                 LinearSimplexElement<AutoDiffXd, 3>,
                                 SimplexGaussianQuadrature<AutoDiffXd, 2, 3>>;
template class ElasticityElement<double, LinearSimplexElement<double, 3>,
                                 SimplexGaussianQuadrature<double, 3, 3>>;
template class ElasticityElement<AutoDiffXd,
                                 LinearSimplexElement<AutoDiffXd, 3>,
                                 SimplexGaussianQuadrature<AutoDiffXd, 3, 3>>;
}  // namespace fem
}  // namespace multibody
}  // namespace drake
