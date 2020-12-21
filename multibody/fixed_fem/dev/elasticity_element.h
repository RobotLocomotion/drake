#pragma once

#include <array>
#include <type_traits>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/constitutive_model.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"
#include "drake/multibody/fixed_fem/dev/isoparametric_element.h"
#include "drake/multibody/fixed_fem/dev/quadrature.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The traits class for ElasticityElement. This traits class is meant to be
 inherited by StaticElasticityElementTraits and DynamicElasticityElementTraits
 and is not meant to be used directly by itself. */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel>
struct ElasticityElementTraits {
  /* Check that template parameters are of the correct types. */
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
  /* Only 3D elasticity is supported. */
  static_assert(IsoparametricElement::spatial_dimension() == 3);

  using T = typename ConstitutiveModel::T;

  static constexpr int kNumNodes = IsoparametricElement::num_nodes();
  static constexpr int kNumQuadraturePoints =
      Quadrature::num_quadrature_points();
  static constexpr int kNaturalDimension = Quadrature::natural_dimension();
  static constexpr int kSpatialDimension =
      IsoparametricElement::spatial_dimension();
  static constexpr int kSolutionDimension = 3;
  static constexpr int kNumDofs = kSolutionDimension * kNumNodes;

  struct Data {
    typename ConstitutiveModel::Traits::DeformationGradientCacheEntryType
        deformation_gradient_cache_entry;
    /* The elastic energy density evaluated at quadrature points. */
    std::array<T, kNumQuadraturePoints> Psi;
    /* The first Piola stress evaluated at quadrature points. */
    std::array<Eigen::Matrix<T, kSpatialDimension, kSolutionDimension>,
               kNumQuadraturePoints>
        P;
    /* The derivative of first Piola stress with respect to the deformation
     gradient evaluated at quadrature points. */
    std::array<Eigen::Matrix<T, kSpatialDimension * kSolutionDimension,
                             kSpatialDimension * kSolutionDimension>,
               kNumQuadraturePoints>
        dPdF;
  };
};

/** The FEM element class for static and dynamic 3D elasticity problems.
 %ElasticityElement serves as an intermediate base class for
 DynamicElasticityElement and StaticElasticityElement. It provides methods such
 as `CalcElasticEnergy()` that are common to both static and dynamic elasticity
 problems. It does not (and should not), however, implement
 the CRTP base class FemElement because the calculations related to
 static and dynamic elasticities are different.
 @tparam IsoparametricElement    The type of isoparametric element used in this
 %ElasticityElement. %IsoparametricElement must be a derived class from
 IsoparametricElement.
 @tparam Quadrature    The type of quadrature rule used in this
 %ElasticityElement. %Quadrature must be a derived class from Quadrature.
 @tparam ConstitutiveModel    The type of constitutive model used in this
 %ElasticityElement. %ConstitutiveModel must be a derived class from
 ConstitutiveModel.
 @tparam DerivedElement    The concrete FEM element that inherits from
 %ElasticityElement through CRTP.
 @tparam DerivedTraits    The traits class associated with the DerivedElement.
 */
template <class IsoparametricElement, class Quadrature, class ConstitutiveModel,
          class DerivedElement, class DerivedTraits>
class ElasticityElement : public FemElement<DerivedElement, DerivedTraits> {
 public:
  using Traits = DerivedTraits;
  using T = typename Traits::T;

  /** Given the current state, calculates the elastic potential energy stored in
   this element in unit J. */
  T CalcElasticEnergy(const FemState<DerivedElement>& state) const {
    T elastic_energy = 0;
    const typename Traits::Data& data =
        state.element_data(get_derived_element());
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      elastic_energy += reference_volume_[q] * data.Psi[q];
    }
    return elastic_energy;
  }

 protected:
  /** Assignment and copy constructions are prohibited. Move constructor is
   allowed so that %ElasticityElement can be stored in `std::vector`. */
  ElasticityElement(const ElasticityElement&) = delete;
  ElasticityElement(ElasticityElement&&) = default;
  const ElasticityElement& operator=(const ElasticityElement&) = delete;
  ElasticityElement&& operator=(const ElasticityElement&&) = delete;

  /** Constructs a new ElasticityElement. The constructor is made protected
   because ElasticityElement should not be constructed directly. Use the
   constructor of the derived classes instead.
   @param[in] element_index    The index of the new element.
   @param[in] node_indices    The node indices of the nodes of this element.
   @param[in] constitutive_model    The ConstitutiveModel to be used for this
   element.
   @param[in] reference_positions    The positions (in world frame) of the nodes
   of this element in the reference configuration.
   @pre element_index must be valid. */
  ElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModel& constitutive_model,
      const Eigen::Ref<
          const Eigen::Matrix<T, Traits::kSpatialDimension, Traits::kNumNodes>>&
          reference_positions)
      : FemElement<DerivedElement, DerivedTraits>(element_index, node_indices),
        constitutive_model_(constitutive_model) {
    /* Record the quadrature point volumes for the new element. */
    const std::array<
        Eigen::Matrix<T, Traits::kSpatialDimension, Traits::kNaturalDimension>,
        Traits::kNumQuadraturePoints>
        dXdxi = shape_.CalcJacobian(reference_positions);
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      /* The scale to transform quadrature weight in parent coordinates to
       reference coordinates. */
      T volume_scale;
      if constexpr (Traits::kNaturalDimension == 3) {
        volume_scale = dXdxi[q].determinant();
        /* Degenerate tetrahedron in the initial configuration is not allowed.
         */
        DRAKE_DEMAND(volume_scale > 0);
        // NOLINTNEXTLINE(readability/braces)
      } else if constexpr (Traits::kNaturalDimension == 2) {
        /* Given the QR decomposition of the Jacobian matrix J = QR, where Q is
         unitary and R is upper triangular, the 2x2 top left corner of R gives
         the in plane deformation of the reference triangle. Its determinant
         provides the ratio of the area of triangle in the reference
         configuration over the area of the triangle in parent domain. */
        Eigen::ColPivHouseholderQR<Eigen::Matrix<T, Traits::kSpatialDimension,
                                                 Traits::kNaturalDimension>>
            qr(dXdxi[q]);
        volume_scale = abs(qr.matrixR()
                               .topLeftCorner(Traits::kNaturalDimension,
                                              Traits::kNaturalDimension)
                               .template triangularView<Eigen::Upper>()
                               .determinant());
      } else if constexpr (Traits::kNaturalDimension == 1) {
        volume_scale = dXdxi[q].norm();
      } else {
        DRAKE_UNREACHABLE();
      }
      reference_volume_[q] = volume_scale * quadrature_.get_weight(q);
    }

    /* Record the inverse Jacobian at the reference configuration which is used
     in the calculation of deformation gradient. */
    dxidX_ = shape_.CalcJacobianPseudoinverse(dXdxi);

    const std::array<
        Eigen::Matrix<T, Traits::kNumNodes, Traits::kNaturalDimension>,
        Traits::kNumQuadraturePoints>& dSdxi =
        shape_.GetGradientInParentCoordinates();
    // TODO(xuchenhan-tri) Replace this with CalcGradientInSpatialCoordinates()
    //  when it is available in IsoparametricElement.
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      dSdX_transpose_[q] = (dSdxi[q] * dxidX_[q]).transpose();
    }
  }

  /** Calculates the negative elastic force on the nodes in this element.
   @param[in] state    The FEM state at which to evaluate the negative elastic
   force.
   @param[out] neg_force    The negative force vector.
   @pre neg_force != nullptr.
   @warning The data in `neg_force` will NOT be set to zero before writing in
   the new data. The caller is responsible for clearing any stale data. */
  void CalcNegativeElasticForce(
      const FemState<DerivedElement>& state,
      EigenPtr<Vector<T, Traits::kNumDofs>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    auto neg_force_matrix = Eigen::Map<
        Eigen::Matrix<T, Traits::kSolutionDimension, Traits::kNumNodes>>(
        neg_force->data(), Traits::kSolutionDimension, Traits::kNumNodes);
    const typename Traits::Data& data =
        state.element_data(get_derived_element());
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      /* Negative force is the gradient of energy.
       -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
       Notice that Fᵢⱼ = xₐᵢdSₐ/dXⱼ, so dFᵢⱼ/dxᵦₖ = δₐᵦδᵢₖdSₐ/dXⱼ,
       and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
       PᵢⱼδₐᵦδᵢₖdSₐ/dXⱼ = PₖⱼdSᵦ/dXⱼ = P * dSdX.transpose() */
      neg_force_matrix += reference_volume_[q] * data.P[q] * dSdX_transpose_[q];
    }
  }

  /** Calculates the derivative of the negative elastic force on the nodes with
   respect to the positions of the nodes in this element.
   @param[in] state    The FEM state at which to evaluate the negative elastic
   force derivatives.
   @param[out] K    The negative force derivative matrix.
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
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    DRAKE_ASSERT(K != nullptr);
    // clang-format off
    /* Let e be the elastic energy, then the ab-th block of the stiffness
     matrix K is given by:
     Kᵃᵇᵢⱼ = d²e/dxᵃᵢdxᵇⱼ = ∫dF/dxᵇⱼ:d²ψ/dF²:dF/dxᵃᵢ + dψ/dF:d²F/dxᵃᵢdxᵇⱼ dX.
     The second term vanishes because Fₖₗ = xᵃₖdSᵃ/dXₗ is linear in x.
     We calculate the first term:
     dF/dxᵇⱼ : d²ψ/dF² : dF/dxᵃᵢ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ.  */
    // clang-format on
    const typename Traits::Data& data =
        state.element_data(get_derived_element());
    // The ab-th 3-by-3 block of K.
    Matrix3<T> K_ab;
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      /* Notice that Fₖₗ = xᵃₖdSᵃ/dXₗ, so dFₖₗ/dxᵇⱼ = δᵃᵇδₖⱼdSᵃ/dXₗ, and thus
       Kᵃᵇᵢⱼ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ =  dSᵃ/dXₙ dPᵢₙ/dFⱼₗ dSᵇ/dXₗ. */
      for (int a = 0; a < Traits::kNumNodes; ++a) {
        for (int b = 0; b < Traits::kNumNodes; ++b) {
          DoDoubleTensorContraction(
              data.dPdF[q], dSdX_transpose_[q].col(a),
              dSdX_transpose_[q].col(b) * reference_volume_[q], &K_ab);
          AccumulateMatrixBlock(K_ab, a, b, K);
        }
      }
    }
  }

  const IsoparametricElement isoparametric_element() const { return shape_; }

  const std::array<T, Traits::kNumQuadraturePoints>& reference_volume() const {
    return reference_volume_;
  }

 private:
  /* Friend the base class so that FemElement::ComputeData() can reach its
   implementation. */
  friend FemElement<DerivedElement, DerivedTraits>;
  friend class ElasticityElementTest;

  /* Implements FemElement::ComputeData(). */
  typename Traits::Data DoComputeData(
      const FemState<DerivedElement>& state) const {
    typename Traits::Data data;
    data.deformation_gradient_cache_entry.mutable_deformation_gradient() =
        CalcDeformationGradient(state);
    data.deformation_gradient_cache_entry.UpdateCacheEntry(
        data.deformation_gradient_cache_entry.deformation_gradient());
    constitutive_model_.CalcElasticEnergyDensity(
        data.deformation_gradient_cache_entry, &data.Psi);
    constitutive_model_.CalcFirstPiolaStress(
        data.deformation_gradient_cache_entry, &data.P);
    constitutive_model_.CalcFirstPiolaStressDerivative(
        data.deformation_gradient_cache_entry, &data.dPdF);
    return data;
  }

  /* Calculates the deformation gradient at all quadrature points in this
   element. */
  std::array<Matrix3<T>, Traits::kNumQuadraturePoints> CalcDeformationGradient(
      const FemState<DerivedElement>& state) const {
    // TODO(xuchenhan-tri): Consider abstracting this potential common operation
    //  into FemElement.
    std::array<Matrix3<T>, Traits::kNumQuadraturePoints> F;
    Eigen::Matrix<T, Traits::kSolutionDimension, Traits::kNumNodes> element_x;
    const VectorX<T>& x_tmp = state.q();
    const auto& x = Eigen::Map<const Matrix3X<T>>(
        x_tmp.data(), Traits::kSolutionDimension,
        x_tmp.size() / Traits::kSolutionDimension);
    for (int i = 0; i < Traits::kNumNodes; ++i) {
      element_x.col(i) = x.col(this->node_indices()[i]);
    }
    const std::array<typename IsoparametricElement::JacobianMatrix,
                     Traits::kNumQuadraturePoints>
        dxdxi = shape_.CalcJacobian(element_x);
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      F[q] = dxdxi[q] * dxidX_[q];
    }
    return F;
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
   3*node_b. Note that this function assumes the pointer `matrix` is not null.
   It also does not check the index it tries to write in `matrix` is valid and
   does not clear any stale data that might exist in `matrix`. */
  static void AccumulateMatrixBlock(
      const Eigen::Ref<const Matrix3<T>>& block, int node_a, int node_b,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> matrix) {
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 3; ++i) {
        (*matrix)(3 * node_a + i, 3 * node_b + j) += block(i, j);
      }
    }
  }

  /* Return `this` element statically cast either as StaticElasticityElement or
   DynamicElasticityElement depending on its type. */
  const DerivedElement& get_derived_element() const {
    return static_cast<const DerivedElement&>(*this);
  }

  // TODO(xuchenhan-tri): Consider bumping this up into FemElement when new
  //  FemElement types are added.
  /* The quadrature rule used for this element. */
  Quadrature quadrature_;
  /* The isoparametric shape function used for this element. */
  IsoparametricElement shape_{quadrature_.get_points()};
  /* The constitutive model that describes the stress-strain relationship
   for this element. */
  ConstitutiveModel constitutive_model_;
  /* The inverse element Jacobian evaluated at reference configuration at
   the quadrature points in this element. */
  std::array<
      Eigen::Matrix<T, Traits::kNaturalDimension, Traits::kSolutionDimension>,
      Traits::kNumQuadraturePoints>
      dxidX_;
  /* The transpose of the derivatives of the shape functions with respect to the
   reference positions evaluated at the quadrature points in this element. */
  std::array<Eigen::Matrix<T, Traits::kSpatialDimension, Traits::kNumNodes>,
             Traits::kNumQuadraturePoints>
      dSdX_transpose_;
  /* The volume evaluated at reference configuration occupied by the
   quadrature points in this element. To integrate a function f over the
   reference domain, sum f(q)*reference_volume_[q] over all the quadrature
   points q in the element. */
  std::array<T, Traits::kNumQuadraturePoints> reference_volume_;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
