#include "drake/multibody/fem/dev/elasticity_element.h"

#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/dev/elasticity_element_cache_entry.h"
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
      dSdX_transpose_(num_quadrature_points()),
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
  const std::vector<MatrixX<T>>& dSdxi =
      shape_.CalcGradientInParentCoordinates();
  for (int q = 0; q < num_quadrature_points(); ++q) {
    dxidX_[q] = Eigen::Ref<const MatrixD3>(dxidX[q]);
    dSdX_transpose_[q] = (dSdxi[q] * dxidX[q]).transpose();
  }
}

template <typename T, class IsoparametricElementType, class QuadratureType>
std::unique_ptr<ElementCacheEntry<T>>
ElasticityElement<T, IsoparametricElementType,
                  QuadratureType>::MakeElementCacheEntry() const {
  std::unique_ptr<DeformationGradientCacheEntry<T>>
      deformation_gradient_cache_entry =
          constitutive_model_->MakeDeformationGradientCacheEntry(
              this->element_index(), this->num_quadrature_points());
  return std::make_unique<ElasticityElementCacheEntry<T>>(
      this->element_index(), this->num_quadrature_points(),
      std::move(deformation_gradient_cache_entry));
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

/* The stiffness matrix calculated here is the same as the stiffness matrix
 calculated in [Bonet, 2016] equation (9.50b) without the external force
 component.
 Without the external force component, (9,50b) reads Kₐᵦ = Kₐᵦ,c + Kₐᵦ,σ.
 Kₐᵦ,c is given by ∫dSᵃ/dxₖ cᵢₖⱼₗ dSᵇ/dxₗ dx (9.35), and
 Kₐᵦ,σ is given by ∫dSᵃ/dxₖ σₖₗ dSᵇ/dxₗ dx (9.44c). Notice that we used S to
 denote shape functions whereas [Bonet, 2016] uses N.
 The stiffness we calculate here is given by ∫ dF/dxᵇ : dP/dF : dF/dxᵃ dX.
 The calculation uses a different conjugate pair pair, but is analytically equal
 to Kₐᵦ,c + Kₐᵦ,σ. See multibody/fem/dev/doc/stiffness_matrix.pdf for the
 derivation that shows the equivalence.
 // TODO(xuchenhan-tri): Update the directory above when this file moves out of
 dev/.

 Reference: [Bonet, 2016] Bonet, Javier, Antonio J.Gil, and
 Richard D. Wood. Nonlinear solid mechanics for finite element analysis:
 statics. Cambridge University Press, 2016. */

/* TODO(xuchenhan-tri): Consider performing the calculation in current
coordinates. A few trade-offs:
 1. The shape function derivatives needs to be recalculated every time.
 2. There will be two terms instead of one.
 3. The c matrix has symmetries that can be exploited and can be represented by
 a symmetric 6x6 matrix, whereas dP/dF is an unsymmetric 9x9 matrix.
 The two stress-strain pairs need to be carefully profiled against each other as
 this operation might be (one of) the bottleneck(s). */
template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcStiffnessMatrix(const FemState<T>& state,
                        EigenPtr<MatrixX<T>> K) const {
  DRAKE_ASSERT(K->rows() == 3 * num_nodes());
  DRAKE_ASSERT(K->cols() == 3 * num_nodes());
  K->setZero();
  // clang-format off
  /* Let e be the elastic energy, then the ab-th block of the stiffness
   matrix K is given by:
   Kᵃᵇᵢⱼ = d²e/dxᵃᵢdxᵇⱼ = ∫dF/dxᵇⱼ:d²ψ/dF²:dF/dxᵃᵢ + dψ/dF:d²F/dxᵃᵢdxᵇⱼ dX.
   The second term vanishes because Fₖₗ = xᵃₖdSᵃ/dXₗ is linear in x.
   We calculate the first term:
   dF/dxᵇⱼ : d²ψ/dF² : dF/dxᵃᵢ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ.  */
  // clang-format on
  // TODO(xuchenhan-tri): Use the corresponding Eval method when caching is
  // ready.
  std::vector<Eigen::Matrix<T, 9, 9>> dPdF;
  CalcFirstPiolaStressDerivative(state, &dPdF);
  // The ab-th 3-by-3 block of K.
  Matrix3<T> K_ab;
  for (int q = 0; q < num_quadrature_points(); ++q) {
    /* Notice that Fₖₗ = xᵃₖdSᵃ/dXₗ, so dFₖₗ/dxᵇⱼ = δᵃᵇδₖⱼdSᵃ/dXₗ, and thus
     Kᵃᵇᵢⱼ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ =  dSᵃ/dXₙ dPᵢₙ/dFⱼₗ dSᵇ/dXₗ. */
    for (int a = 0; a < num_nodes(); ++a) {
      for (int b = 0; b < num_nodes(); ++b) {
        DoDoubleTensorContraction(
            dPdF[q], dSdX_transpose_[q].col(a),
            dSdX_transpose_[q].col(b) * reference_volume_[q], &K_ab);
        AccumulateMatrixBlock(K_ab, a, b, K);
      }
    }
  }
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
    DoCalcTangentMatrix(const FemState<T>& state,
                        EigenPtr<MatrixX<T>> tangent_matrix) const {
  // TODO(xuchenhan-tri): Add inertia terms when mass matrix is in
  // place.
  // TODO(xuchenhan-tri): Add damping force derivatives.
  CalcStiffnessMatrix(state, tangent_matrix);
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
  for (int q = 0; q < num_quadrature_points(); ++q) {
    /* Negative force is the gradient of energy.
     -f = ∫dΨ/dx dX = ∫dΨ/dF : dF/dx dX.
     Notice that Fᵢⱼ = xᵃᵢdSᵃ/dXⱼ, so dFᵢⱼ/dxᵇₖ = δᵃᵇδᵢₖdSᵃ/dXⱼ,
     and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
     PᵢⱼδᵃᵇδᵢₖdSᵃ/dXⱼ = PₖⱼdSᵇ/dXⱼ = P * dSdX.transpose() */
    neg_force_matrix += reference_volume_[q] * P[q] * dSdX_transpose_[q];
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
const DeformationGradientCacheEntry<T>&
ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    EvalDeformationGradientCacheEntry(const FemState<T>& state) const {
  ElasticityElementCacheEntry<T>& mutable_cache_entry =
      static_cast<ElasticityElementCacheEntry<T>&>(
          state.mutable_element_cache_entry(this->element_index()));
  DeformationGradientCacheEntry<T>& deformation_gradient_cache_entry =
      mutable_cache_entry.mutable_deformation_gradient_cache_entry();
  // TODO(xuchenhan-tri): Enable caching when caching is in place.
  std::vector<Matrix3<T>> F(num_quadrature_points());
  CalcDeformationGradient(state, &F);
  deformation_gradient_cache_entry.UpdateCacheEntry(F);
  return deformation_gradient_cache_entry;
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcElasticEnergyDensity(const FemState<T>& state,
                             std::vector<T>* Psi) const {
  Psi->resize(num_quadrature_points());
  const DeformationGradientCacheEntry<T>& deformation_gradient_cache_entry =
      EvalDeformationGradientCacheEntry(state);
  constitutive_model_->CalcElasticEnergyDensity(
      deformation_gradient_cache_entry, Psi);
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcFirstPiolaStress(const FemState<T>& state,
                         std::vector<Matrix3<T>>* P) const {
  P->resize(num_quadrature_points());
  const DeformationGradientCacheEntry<T>& deformation_gradient_cache_entry =
      EvalDeformationGradientCacheEntry(state);
  constitutive_model_->CalcFirstPiolaStress(deformation_gradient_cache_entry,
                                            P);
}

template <typename T, class IsoparametricElementType, class QuadratureType>
void ElasticityElement<T, IsoparametricElementType, QuadratureType>::
    CalcFirstPiolaStressDerivative(
        const FemState<T>& state,
        std::vector<Eigen::Matrix<T, 9, 9>>* dPdF) const {
  dPdF->resize(num_quadrature_points());
  const DeformationGradientCacheEntry<T>& deformation_gradient_cache_entry =
      EvalDeformationGradientCacheEntry(state);
  constitutive_model_->CalcFirstPiolaStressDerivative(
      deformation_gradient_cache_entry, dPdF);
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
