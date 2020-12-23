#pragma once

#include <array>
#include <type_traits>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element_cache_entry.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/isoparametric_element.h"
#include "drake/multibody/fixed_fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The traits class for ElasticityElement. This traits class is meant to be
 inherited from by StaticElasticityElementTraits and
 DynamicElasticityElementTraits and is not meant to be used by itself. */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel>
struct ElasticityElementTraits {
  // Check that template parameters are of the correct types.
  static_assert(is_isoparametric_element<IsoparametricElement>::value,
                "IsoparametricElementType must be a derived class of "
                "IsoparametricElement");
  static_assert(is_quadrature<Quadrature>::value,
                "Quadrature must be a derived class of "
                "Quadrature<T, NaturalDim, NumLocations>, where NaturalDim can "
                "be 1, 2 or 3.");
  static_assert(is_constitutive_model<ConstitutiveModel>::value,
                "ConstitutiveModel must be a derived class of "
                "ConstitutiveModel");
  /* Check that the scalar types are compatible. */
  static_assert(std::is_same_v<typename IsoparametricElement::T,
                               typename ConstitutiveModel::T>);
  /* Check that the number of quadrature points are compatible. */
  static_assert(Quadrature::num_quadrature_points() ==
                IsoparametricElement::num_sample_locations());
  static_assert(Quadrature::num_quadrature_points() ==
                ConstitutiveModel::num_locations());
  /* Check that the natural dimensions are compatible. */
  static_assert(IsoparametricElement::natural_dimension() ==
                Quadrature::natural_dimension());
  /* Only 3D elasticity are supported. */
  static_assert(IsoparametricElement::spatial_dimension() == 3);

  using T = typename ConstitutiveModel::T;
  using DeformationGradientCacheEntryType =
      typename ConstitutiveModel::DeformationGradientCacheEntryType;
  using ElementCacheEntryType = ElasticityElementCacheEntry<ConstitutiveModel>;
  static constexpr int kNumNodes = IsoparametricElement::num_nodes();
  static constexpr int kNumQuadraturePoints =
      Quadrature::num_quadrature_points();
  static constexpr int kNaturalDimension = Quadrature::natural_dimension();
  static constexpr int kSpatialDimension =
      IsoparametricElement::spatial_dimension();
  static constexpr int kSolutionDimension = 3;
  static constexpr int kNumDofs = kSolutionDimension * kNumNodes;
};

/** The FEM element class for static and dynamic 3D elasticity problems.
 %ElasticityElement serves as an intermediate base class for
 DynamicElasticityElement and StaticElasticityElement. It provides methods such
 as `CalcElasticEnergy()` that are common to both static and dynamic elasticity
 problems. It does not (and should not), however, implement the interface
 provided in the CRTP base class FemElement. Therefore, the interface has to be
 implemented in the most derived classes, namely, DynamicElasticityElement and
 StaticElasticityElement.
 @tparam IsoparametricElement The type of isoparametric element used in this
 %ElasticityElement. %IsoparametricElement must be a derived class from
 IsoparametricElement.
 @tparam Quadrature The type of quadrature rule used in this %ElasticityElement.
 %Quadrature must be a derived class from Quadrature.
 @tparam ConstitutiveModel The type of constitutive model used in this
 %ElasticityElement. %ConstitutiveModel must be a derived class from
 ConstitutiveModel.
 @tparam DerivedElement The concrete FEM element that inherits from
 %ElasticityElement through CRTP.
 @tparam DerivedTraits The traits class associated with the DerivedElement. */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel,
          class DerivedElement, class DerivedTraits>
class ElasticityElement : public FemElement<DerivedElement, DerivedTraits> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityElement);
  /* Promoted types and methods from the base class for readability. */
  using Base = FemElement<DerivedElement, DerivedTraits>;
  using T = typename Base::T;
  using Base::natural_dimension;
  using Base::num_dofs;
  using Base::num_nodes;
  using Base::num_quadrature_points;
  using Base::solution_dimension;
  using Base::spatial_dimension;

  /** Given the current state, calculates the elastic potential energy stored in
   this element in unit J. */
  T CalcElasticEnergy(const FemState<DerivedElement>& state) const {
    T elastic_energy = 0;
    // TODO(xuchenhan-tri): Use the corresponding Eval method when cache
    // is in place.
    std::array<T, num_quadrature_points()> Psi;
    CalcElasticEnergyDensity(state, &Psi);
    for (int q = 0; q < num_quadrature_points(); ++q) {
      elastic_energy += reference_volume_[q] * Psi[q];
    }
    return elastic_energy;
  }

 protected:
  /** Constructs a new FEM elasticity element.
   @param[in] element_index The global index of the new element.
   @param[in] node_indices The global node indices of the nodes of this
   element.
   @param[in] density The mass density of the element in the reference
   configuration, with unit kg/m³.
   @param[in] constitutive_model The ConstitutiveModel to be used for this
   element.
   @param[in] reference_positions The positions of the nodes of this element in
   the reference configuration.
   @pre element_index must be valid. */
  ElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, num_nodes()>& node_indices,
      const ConstitutiveModel& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, solution_dimension(),
                                           num_nodes()>>& reference_positions)
      : Base(element_index, node_indices),
        constitutive_model_(constitutive_model),
        reference_positions_(reference_positions) {
    // Record the quadrature point volumes for the new element.
    const std::array<
        Eigen::Matrix<T, solution_dimension(), natural_dimension()>,
        num_quadrature_points()>
        dXdxi = shape_.CalcJacobian(reference_positions);
    for (int q = 0; q < num_quadrature_points(); ++q) {
      /* The scale to transform quadrature weight in parent coordinates to
       reference coordinates. */
      T volume_scale;
      if constexpr (natural_dimension() == 3) {
        volume_scale = dXdxi[q].determinant();
        /* Degenerate tetrahedron in the initial configuration is not allowed.
         */
        DRAKE_DEMAND(volume_scale > 0);
        // NOLINTNEXTLINE(readability/braces)
      } else if constexpr (natural_dimension() == 2) {
        /* Given the QR decomposition of the Jacobian matrix J = QR, where Q is
         unitary and R is upper triangular, the 2x2 top left corner of R gives
         the in plane deformation of the reference triangle. Its determinant
         provides the ratio of the area of triangle in the reference
         configuration over the area of the triangle in parent domain. */
        Eigen::ColPivHouseholderQR<
            Eigen::Matrix<T, solution_dimension(), natural_dimension()>>
            qr(dXdxi[q]);
        volume_scale =
            abs(qr.matrixR()
                    .topLeftCorner(natural_dimension(), natural_dimension())
                    .template triangularView<Eigen::Upper>()
                    .determinant());
      } else if constexpr (natural_dimension() == 1) {
        volume_scale = dXdxi[q].norm();
      } else {
        DRAKE_UNREACHABLE();
      }
      reference_volume_[q] = volume_scale * quadrature_.get_weight(q);
    }

    /* Record the inverse Jacobian at the reference configuration which is used
     in the calculation of deformation gradient.*/
    dxidX_ = shape_.CalcJacobianPseudoinverse(dXdxi);

    const std::array<Eigen::Matrix<T, num_nodes(), natural_dimension()>,
                     num_quadrature_points()>& dSdxi =
        shape_.GetGradientInParentCoordinates();
    // TODO(xuchenhan-tri) Replace this with CalcGradientInSpatialCoordinates()
    //  when this is available in IsoparametricElement.
    for (int q = 0; q < num_quadrature_points(); ++q) {
      dSdX_transpose_[q] = (dSdxi[q] * dxidX_[q]).transpose();
    }
  }

  /** Calculates the negative elastic force on the nodes in this element.
   @param[in] state The FEM state at which to evaluate the negative elastic
   force.
   @param[out] neg_force The negative force vector.
   @pre neg_force != nullptr.
   @warning The data in `neg_force` will NOT be set to zero before writing in
   the new data. The caller is responsible for clearing any stale data. */
  void CalcNegativeElasticForce(
      const FemState<DerivedElement>& state,
      EigenPtr<Vector<T, num_dofs()>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    auto neg_force_matrix =
        Eigen::Map<Eigen::Matrix<T, solution_dimension(), num_nodes()>>(
            neg_force->data(), solution_dimension(), num_nodes());
    // TODO(xuchenhan-tri): Use the corresponding Eval method when cache is in
    //  place.
    std::array<Matrix3<T>, num_quadrature_points()> P;
    CalcFirstPiolaStress(state, &P);
    for (int q = 0; q < num_quadrature_points(); ++q) {
      /* Negative force is the gradient of energy.
       -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
       Notice that Fᵢⱼ = xₐᵢdSₐ/dXⱼ, so dFᵢⱼ/dxᵦₖ = δₐᵦδᵢₖdSₐ/dXⱼ,
       and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
       PᵢⱼδₐᵦδᵢₖdSₐ/dXⱼ = PₖⱼdSᵦ/dXⱼ = P * dSdX.transpose() */
      neg_force_matrix += reference_volume_[q] * P[q] * dSdX_transpose_[q];
    }
  }

  /** Calculates the derivative of the negative elastic force on the nodes with
   respect to the positions of the nodes in this element.
   @param[in] state The FEM state at which to evaluate the negative elastic
   force derivatives.
   @param[out] K The negative force derivative matrix.
   @pre K != nullptr.
   @warning The data in `K` will NOT be set to zero before writing in the new
   data. The caller is responsible for clearing any stale data. */
  /* The matrix calculated here is the same as the stiffness matrix
   calculated in [Bonet, 2016] equation (9.50b) without the external force
   component.
   Without the external force component, (9,50b) reads Kₐᵦ = Kₐᵦ,c + Kₐᵦ,σ.
   Kₐᵦ,c is given by ∫dSᵃ/dxₖ cᵢₖⱼₗ dSᵇ/dxₗ dx (9.35), and
   Kₐᵦ,σ is given by ∫dSᵃ/dxₖ σₖₗ dSᵇ/dxₗ dx (9.44c). Notice that we used S to
   denote shape functions whereas [Bonet, 2016] uses N.
   The negative force derivative we calculate here is given by ∫ dF/dxᵇ : dP/dF
   : dF/dxᵃ dX. The calculation uses a different conjugate pair pair, but is
   analytically equal to Kₐᵦ,c + Kₐᵦ,σ. See
   multibody/fem/dev/doc/stiffness_matrix.pdf for the derivation that shows the
   equivalence.
   // TODO(xuchenhan-tri): Update the directory above when this file moves out
   //  of dev/.

   Reference: [Bonet, 2016] Bonet, Javier, Antonio J.Gil, and
   Richard D. Wood. Nonlinear solid mechanics for finite element analysis:
   statics. Cambridge University Press, 2016. */

  /* TODO(xuchenhan-tri): Consider performing the calculation in current
  coordinates. A few trade-offs:
   1. The shape function derivatives needs to be recalculated every time.
   2. There will be two terms instead of one.
   3. The c matrix has symmetries that can be exploited and can be represented
  by a symmetric 6x6 matrix, whereas dP/dF is an unsymmetric 9x9 matrix. The two
  stress-strain pairs need to be carefully profiled against each other as this
  operation might be (one of) the bottleneck(s). */
  void CalcNegativeElasticForceDerivative(
      const FemState<DerivedElement>& state,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> K) const {
    DRAKE_ASSERT(K != nullptr);
    // clang-format off
    /* Let e be the elastic energy, then the ab-th block of the stiffness
     matrix K is given by:
     Kᵃᵇᵢⱼ = d²e/dxᵃᵢdxᵇⱼ = ∫dF/dxᵇⱼ:d²ψ/dF²:dF/dxᵃᵢ + dψ/dF:d²F/dxᵃᵢdxᵇⱼ dX.
     The second term vanishes because Fₖₗ = xᵃₖdSᵃ/dXₗ is linear in x.
     We calculate the first term:
     dF/dxᵇⱼ : d²ψ/dF² : dF/dxᵃᵢ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ.  */
    // clang-format on
    // TODO(xuchenhan-tri): Use the corresponding Eval method when caching is
    //  ready.
    std::array<Eigen::Matrix<T, spatial_dimension() * solution_dimension(),
                             spatial_dimension() * solution_dimension()>,
               num_quadrature_points()>
        dPdF;
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

  const IsoparametricElement& isoparametric_element() const { return shape_; }

  const std::array<T, num_quadrature_points()>& reference_volume() const {
    return reference_volume_;
  }

 private:
  friend class ElasticityElementTest;
  /* Do not friend the base class (FemElement) to avoid implementation of the
   interface. */

  /* Convenient typedef for readability. */
  using DeformationGradientCacheEntryType =
      typename ConstitutiveModel::DeformationGradientCacheEntryType;

  /* Calculates the deformation gradient at all quadrature points in this
   element. */
  void CalcDeformationGradient(
      const FemState<DerivedElement>& state,
      std::array<Matrix3<T>, num_quadrature_points()>* F) const {
    // TODO(xuchenhan-tri): Consider abstracting this potential common operation
    //  into FemElement.
    DRAKE_ASSERT(F != nullptr);
    Eigen::Matrix<T, solution_dimension(), num_nodes()> element_x;
    const VectorX<T>& x_tmp = state.q();
    const auto& x =
        Eigen::Map<const Matrix3X<T>>(x_tmp.data(), solution_dimension(),
                                      x_tmp.size() / solution_dimension());
    for (int i = 0; i < num_nodes(); ++i) {
      element_x.col(i) = x.col(this->node_indices()[i]);
    }
    const std::array<typename IsoparametricElement::JacobianMatrix,
                     num_quadrature_points()>
        dxdxi = shape_.CalcJacobian(element_x);
    for (int q = 0; q < num_quadrature_points(); ++q) {
      (*F)[q] = dxdxi[q] * dxidX_[q];
    }
  }

  /* Evaluates the DeformationGradientCacheEntry for this element. */
  // TODO(xuchenhan-tri): This method unconditionally recomputes the
  //  DeformationGradientCacheEntry. Enable caching when caching is in place.
  const DeformationGradientCacheEntryType& EvalDeformationGradientCacheEntry(
      const FemState<DerivedElement>& state) const {
    DeformationGradientCacheEntryType& deformation_gradient_cache_entry =
        state.mutable_element_cache_entry(this->element_index())
            .mutable_deformation_gradient_cache_entry();
    // TODO(xuchenhan-tri): Enable caching when caching is in place.
    std::array<Matrix3<T>, num_quadrature_points()> deformation_gradients;
    CalcDeformationGradient(state, &deformation_gradients);
    deformation_gradient_cache_entry.UpdateCacheEntry(deformation_gradients);
    return deformation_gradient_cache_entry;
  }

  /* Calculates the elastic energy density per unit reference volume, in unit
   J/m³, at each quadrature point in this element. */
  void CalcElasticEnergyDensity(
      const FemState<DerivedElement>& state,
      std::array<T, num_quadrature_points()>* Psi) const {
    DRAKE_ASSERT(Psi != nullptr);
    const DeformationGradientCacheEntryType& deformation_gradient_cache_entry =
        EvalDeformationGradientCacheEntry(state);
    constitutive_model_.CalcElasticEnergyDensity(
        deformation_gradient_cache_entry, Psi);
  }

  /* Calculates the first Piola stress, in unit Pa, at each quadrature point in
   * this element. */
  void CalcFirstPiolaStress(
      const FemState<DerivedElement>& state,
      std::array<Matrix3<T>, num_quadrature_points()>* P) const {
    DRAKE_ASSERT(P != nullptr);
    const DeformationGradientCacheEntryType& deformation_gradient_cache_entry =
        EvalDeformationGradientCacheEntry(state);
    constitutive_model_.CalcFirstPiolaStress(deformation_gradient_cache_entry,
                                             P);
  }

  /* Calculates the derivative of first Piola stress at each quadrature point in
     this element. */
  void CalcFirstPiolaStressDerivative(
      const FemState<DerivedElement>& state,
      std::array<Eigen::Matrix<T, spatial_dimension() * solution_dimension(),
                               spatial_dimension() * solution_dimension()>,
                 num_quadrature_points()>* dPdF) const {
    const DeformationGradientCacheEntryType& deformation_gradient_cache_entry =
        EvalDeformationGradientCacheEntry(state);
    constitutive_model_.CalcFirstPiolaStressDerivative(
        deformation_gradient_cache_entry, dPdF);
  }

  /* Helper function that performs a contraction between a 4th order tensor A
   and two vectors u and v and returns a matrix B. In Einstein notation, the
   contraction is: Bᵢₖ = uⱼ Aᵢⱼₖₗ vₗ. The 4th order tensor A of dimension
   3*3*3*3 is flattened to a 9*9 matrix that is organized as following

                    l = 1       l = 2       l = 3
                -------------------------------------
                |           |           |           |
      j = 1     |   Aᵢ₁ₖ₁   |   Aᵢ₁ₖ₂   |   Aᵢ₁ₖ₃   |
                |           |           |           |
                -------------------------------------
                |           |           |           |
      j = 2     |   Aᵢ₂ₖ₁   |   Aᵢ₂ₖ₂   |   Aᵢ₂ₖ₃   |
                |           |           |           |
                -------------------------------------
                |           |           |           |
      j = 3     |   Aᵢ₃ₖ₁   |   Aᵢ₃ₖ₂   |   Aᵢ₃ₖ₃   |
                |           |           |           |
                -------------------------------------
  Namely the ik-th entry in the jl-th block corresponds to the value Aᵢⱼₖₗ. */
  static void DoDoubleTensorContraction(
      const Eigen::Ref<const Eigen::Matrix<T, 9, 9>>& A,
      const Eigen::Ref<const Vector3<T>>& u,
      const Eigen::Ref<const Vector3<T>>& v, EigenPtr<Matrix3<T>> B) {
    B->setZero();
    for (int l = 0; l < 3; ++l) {
      for (int j = 0; j < 3; ++j) {
        *B += A.template block<3, 3>(3 * j, 3 * l) * u(j) * v(l);
      }
    }
  }

  /* Helper function that adds a 3x3 matrix into the 3x3 block in a bigger
   matrix `matrix` with starting row index 3*node_a and starting column index
   3*node_b. Note that this function assumes the pointer `matrix` is not null
   and does not check the index it tries to write in `matrix` is valid and. */
  static void AccumulateMatrixBlock(
      const Eigen::Ref<const Matrix3<T>>& block, int node_a, int node_b,
      EigenPtr<Eigen::Matrix<T, num_dofs(), num_dofs()>> matrix) {
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 3; ++i) {
        (*matrix)(3 * node_a + i, 3 * node_b + j) += block(i, j);
      }
    }
  }

  /* The quadrature rule used for this element. */
  Quadrature quadrature_;
  /* The isoparametric shape function used for this element. */
  IsoparametricElement shape_{quadrature_.get_points()};
  /* The constitutive model that describes the stress-strain relationship
   for this element. */
  ConstitutiveModel constitutive_model_;
  /* The inverse element Jacobian evaluated at reference configuration at
   the quadrature points in this element. */
  std::array<Eigen::Matrix<T, natural_dimension(), solution_dimension()>,
             num_quadrature_points()>
      dxidX_;
  /* The transpose of the derivatives of the shape functions with respect to the
   reference positions evaluated at the quadrature points in this element. */
  std::array<Eigen::Matrix<T, spatial_dimension(), num_nodes()>,
             num_quadrature_points()>
      dSdX_transpose_;
  /* The positions of the nodes of this element in the reference configuration.
   */
  Eigen::Matrix<T, solution_dimension(), num_nodes()> reference_positions_;
  /* The volume evaluated at reference configuration occupied by the
   quadrature points in this element. To integrate a function f over the
   reference domain, sum f(q)*reference_volume_[q] over all the quadrature
   points q in the element. */
  std::array<T, num_quadrature_points()> reference_volume_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
