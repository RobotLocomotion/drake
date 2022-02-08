#pragma once

#include <array>
#include <type_traits>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/fem_state_impl.h"
#include "drake/multibody/fem/isoparametric_element.h"
#include "drake/multibody/fem/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

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
template <typename T>
void PerformDoubleTensorContraction(
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

/* The traits class for volumetric elasticity FEM element. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct VolumetricElementTraits {
  /* Check that template parameters are of the correct types. */
  static_assert(
      is_isoparametric_element<IsoparametricElementType>::value,
      "The IsoparametricElementType template parameter must be a derived "
      "class of IsoparametricElement");
  static_assert(
      is_quadrature<QuadratureType>::value,
      "The QuadratureType template parameter must be a derived class of "
      "Quadrature<T, natural_dimension, num_quadrature_points>, where "
      "`natural_dimension` can be 1, 2 or 3.");
  static_assert(
      is_constitutive_model<ConstitutiveModelType>::value,
      "The ConstitutiveModelType template parameter must be a derived "
      "class of ConstitutiveModel");
  /* Check that the scalar types are compatible. */
  static_assert(std::is_same_v<typename IsoparametricElementType::T,
                               typename ConstitutiveModelType::T>,
                "The scalar type of the isoparametric element and the "
                "constitutive model must be the same.");
  /* Check that the number of quadrature points are compatible. */
  static_assert(
      QuadratureType::num_quadrature_points ==
          IsoparametricElementType::num_sample_locations,
      "The number of quadrature points of the quadrature rule must be the same "
      "as the number of evaluation locations in the isoparametric element.");
  static_assert(
      QuadratureType::num_quadrature_points ==
          ConstitutiveModelType::num_locations,
      "The number of quadrature points must be the same as the number of "
      "locations at which the constitutive model is evaluated.");
  /* Check that the natural dimensions are compatible. */
  static_assert(IsoparametricElementType::natural_dimension ==
                    QuadratureType::natural_dimension,
                "The natural dimension of the isoparametric element and the "
                "quadrature rule must be the same.");
  /* Only 3D elasticity is supported. */
  static_assert(IsoparametricElementType::spatial_dimension == 3,
                "The spatial dimension of the isoparametric element must be 3 "
                "for volumetric FEM elements.");

  using T = typename ConstitutiveModelType::T;
  using IsoparametricElement = IsoparametricElementType;
  using Quadrature = QuadratureType;
  using ConstitutiveModel = ConstitutiveModelType;

  static constexpr int num_nodes = IsoparametricElementType::num_nodes;
  static constexpr int num_quadrature_points =
      QuadratureType::num_quadrature_points;
  static constexpr int natural_dimension = QuadratureType::natural_dimension;
  static constexpr int kSpatialDimension =
      IsoparametricElementType::spatial_dimension;
  /* The number of degrees of freedom is equal to the spatial dimension (which
   gives the number of degrees of freedom for a single node) times the number of
   nodes. */
  static constexpr int num_dofs = kSpatialDimension * num_nodes;

  struct Data {
    typename ConstitutiveModelType::Data deformation_gradient_data;
    /* The elastic energy density evaluated at quadrature points. */
    std::array<T, num_quadrature_points> Psi;
    /* The first Piola stress evaluated at quadrature points. */
    std::array<Matrix3<T>, num_quadrature_points> P;
    /* The derivative of first Piola stress with respect to the deformation
     gradient evaluated at quadrature points. */
    std::array<Eigen::Matrix<T, 9, 9>, num_quadrature_points> dPdF;
  };
};

/* This class models a single 3D elasticity FEM element.
 @tparam IsoparametricElementType  The type of isoparametric element used in
                                   this VolumetricElement.
                                   IsoparametricElementType must derived
                                   from IsoparametricElement.
 @tparam QuadratureType  The type of quadrature rule used in this
                         VolumetricElement. QuadratureType must be derived from
                         Quadrature.
 @tparam ConstitutiveModelType  The type of constitutive model used in this
                                VolumetricElement. ConstitutiveModelType must be
                                derived from ConstitutiveModel.
 @tparam ElementType  The concrete FEM element that inherits from
                         VolumetricElement through CRTP.
 @tparam DerivedTraits   The traits class associated with the ElementType. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
class VolumetricElement
    : public FemElement<
          VolumetricElement<IsoparametricElementType, QuadratureType,
                            ConstitutiveModelType>,
          VolumetricElementTraits<IsoparametricElementType, QuadratureType,
                                  ConstitutiveModelType>> {
 public:
  using ElementType = VolumetricElement<IsoparametricElementType,
                                        QuadratureType, ConstitutiveModelType>;
  using Traits = VolumetricElementTraits<IsoparametricElementType,
                                         QuadratureType, ConstitutiveModelType>;
  using Data = typename Traits::Data;
  using T = typename Traits::T;
  static constexpr int natural_dimension = Traits::natural_dimension;
  static constexpr int kSpatialDimension = Traits::kSpatialDimension;
  static constexpr int num_quadrature_points = Traits::num_quadrature_points;
  static constexpr int num_dofs = Traits::num_dofs;
  static constexpr int num_nodes = Traits::num_nodes;

  /* Constructs a new VolumetricElement. In that process, precomputes the mass
   matrix and the gravity force acting on the element.
   @param[in] element_index        The index of the new element.
   @param[in] node_indices         The node indices of the nodes of this
                                   element.
   @param[in] constitutive_model   The ConstitutiveModel to be used for this
                                   element.
   @param[in] reference_positions  The positions (in world frame) of the nodes
                                   of this element in the reference
                                   configuration.
   @param[in] denstiy              The mass density of the element with unit
                                   kg/m³.
   @param[in] damping_model        The DampingModel to be used for this element.
   @pre element_index must be valid.
   @pre density > 0. */
  VolumetricElement(ElementIndex element_index,
                    const std::array<NodeIndex, num_nodes>& node_indices,
                    const ConstitutiveModelType& constitutive_model,
                    const Eigen::Ref<const Eigen::Matrix<T, 3, num_nodes>>&
                        reference_positions,
                    const T& density, const DampingModel<T>& damping_model)
      : FemElement<ElementType, Traits>(element_index, node_indices,
                                        constitutive_model, damping_model),
        density_(density) {
    DRAKE_DEMAND(density_ > 0);
    /* Find the Jacobian of the change of variable function X(ξ). */
    const std::array<Eigen::Matrix<T, 3, natural_dimension>,
                     num_quadrature_points>
        dXdxi = isoparametric_element_.CalcJacobian(reference_positions);
    /* Record the quadrature point volume in reference configuration for each
     quadrature location. */
    for (int q = 0; q < num_quadrature_points; ++q) {
      /* The scale to transform quadrature weight in parent coordinates to
       reference coordinates. */
      T volume_scale;
      if constexpr (natural_dimension == 3) {
        volume_scale = dXdxi[q].determinant();
        /* Degenerate tetrahedron in the initial configuration is not allowed.
         */
        DRAKE_DEMAND(volume_scale > 0);
        // NOLINTNEXTLINE(readability/braces) false positive
      } else if constexpr (natural_dimension == 2) {
        /* Given the QR decomposition of the Jacobian matrix J = QR, where Q is
         unitary and R is upper triangular, the 2x2 top left corner of R gives
         the in plane deformation of the reference triangle. Its determinant
         provides the ratio of the area of triangle in the reference
         configuration over the area of the triangle in parent domain. */
        Eigen::ColPivHouseholderQR<
            Eigen::Matrix<T, kSpatialDimension, natural_dimension>>
            qr(dXdxi[q]);
        volume_scale =
            abs(qr.matrixR()
                    .topLeftCorner(natural_dimension, natural_dimension)
                    .template triangularView<Eigen::Upper>()
                    .determinant());
      } else if constexpr (natural_dimension == 1) {
        volume_scale = dXdxi[q].norm();
      } else {
        DRAKE_UNREACHABLE();
      }
      reference_volume_[q] = volume_scale * quadrature_.get_weight(q);
    }

    /* Record the inverse Jacobian at the reference configuration which is used
     in the calculation of deformation gradient. */
    dxidX_ = isoparametric_element_.CalcJacobianPseudoinverse(dXdxi);
    /* Record the gradient of the shape functions w.r.t. the reference
     positions, which is used in calculating the residual. */
    const auto dSdX = isoparametric_element_.CalcGradientInSpatialCoordinates(
        reference_positions);
    for (int q = 0; q < num_quadrature_points; ++q) {
      dSdX_transpose_[q] = dSdX[q].transpose();
    }
    mass_matrix_ = PrecomputeMassMatrix();
    lumped_mass_ = mass_matrix_.rowwise().sum().eval();
  }

  /* Assignment and copy constructions are prohibited. Move constructor is
   allowed so that elasticity elements can be stored in `std::vector`. */
  VolumetricElement(const VolumetricElement&) = delete;
  VolumetricElement(VolumetricElement&&) = default;
  const VolumetricElement& operator=(const VolumetricElement&) = delete;
  VolumetricElement&& operator=(const VolumetricElement&&) = delete;

  /* Calculates the elastic potential energy (in joules) stored in this element
   at the given `state`. */
  T CalcElasticEnergy(const FemStateImpl<ElementType>& state) const {
    T elastic_energy = 0;
    const Data& data = state.element_data(*this);
    for (int q = 0; q < num_quadrature_points; ++q) {
      elastic_energy += reference_volume_[q] * data.Psi[q];
    }
    return elastic_energy;
  }

 private:
  /* Friend the base class so that FemElement::DoFoo() can reach its
   implementation. */
  friend FemElement<ElementType, Traits>;
  friend class VolumetricElementTest;

  /* Adds the negative elastic force on the nodes of this element into the
   given force vector. The negative elastic force is the derivative of the
   elastic energy (see CalcElasticEnergy()) with respect to the generalized
   positions of the nodes.
   @param[in] state       The FEM state at which to evaluate the negative
                          elastic force.
   @param[out] neg_force  The negative force vector.
   @pre neg_force != nullptr.
   @warning It is the responsibility of the caller to initialize neg_force to
   zero appropriately. */
  void AddNegativeElasticForce(const FemStateImpl<ElementType>& state,
                               EigenPtr<Vector<T, num_dofs>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    auto neg_force_matrix = Eigen::Map<Eigen::Matrix<T, 3, num_nodes>>(
        neg_force->data(), 3, num_nodes);
    const Data& data = state.element_data(*this);
    for (int q = 0; q < num_quadrature_points; ++q) {
      /* Negative force is the gradient of energy.
       -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
       Notice that Fᵢⱼ = xₐᵢdSₐ/dXⱼ, so dFᵢⱼ/dxᵦₖ = δₐᵦδᵢₖdSₐ/dXⱼ,
       and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
       PᵢⱼδₐᵦδᵢₖdSₐ/dXⱼ = PₖⱼdSᵦ/dXⱼ = P * dSdX.transpose() */
      neg_force_matrix += reference_volume_[q] * data.P[q] * dSdX_transpose_[q];
    }
  }

  /* Adds the negative damping force on the nodes of this element into the given
   `negative_damping_force`. The negative damping force is given by the product
   of the damping matrix with the velocity of the nodes.
   @param[in] state       The FEM state at which to evaluate the negative
                          damping force.
   @param[out] neg_force  The negative force vector.
   @pre neg_force != nullptr.
   @warning It is the responsibility of the caller to initialize neg_force to
   zero appropriately. */
  void AddNegativeDampingForce(const FemStateImpl<ElementType>& state,
                               EigenPtr<Vector<T, num_dofs>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    Eigen::Matrix<T, num_dofs, num_dofs> damping_matrix =
        Eigen::Matrix<T, num_dofs, num_dofs>::Zero();
    this->AddScaledDampingMatrix(state, 1, &damping_matrix);
    /* Note that the damping force fᵥ = -D * v, where D is the damping matrix.
     As we are accumulating the negative damping force here, the `+=` sign
     should be used. */
    *neg_force +=
        damping_matrix *
        this->ExtractElementDofs(this->node_indices(), state.GetVelocities());
  }

  /* The matrix calculated here is the same as the stiffness matrix
   calculated in [Bonet, 2016] equation (9.50b) without the external force
   component.
   Without the external force component, (9,50b) reads Kₐᵦ = Kₐᵦ,c + Kₐᵦ,σ.
   Kₐᵦ,c is given by ∫dSᵃ/dxₖ cᵢₖⱼₗ dSᵇ/dxₗ dx (9.35), and
   Kₐᵦ,σ is given by ∫dSᵃ/dxₖ σₖₗ dSᵇ/dxₗ dx (9.44c). Notice that we use S to
   denote shape functions whereas [Bonet, 2016] uses N.
   The negative force derivative we calculate here is given by ∫ dF/dxᵇ :
   dP/dF : dF/dxᵃ dX. The calculation uses a different conjugate pair, but is
   analytically equal to Kₐᵦ,c + Kₐᵦ,σ. See
   multibody/fem/doc/stiffness_matrix.pdf for the derivation that
   shows the equivalence.
   // TODO(xuchenhan-tri): Update the directory above when this file moves out
   //  of .

   Reference: [Bonet, 2016] Bonet, Javier, Antonio J.Gil, and
   Richard D. Wood. Nonlinear solid mechanics for finite element analysis:
   statics. Cambridge University Press, 2016. */

  /* TODO(xuchenhan-tri): Consider performing the calculation in current
   coordinates. A few trade-offs:
    1. The shape function derivatives needs to be recalculated every time.
    2. There will be two terms instead of one.
    3. The c matrix has symmetries that can be exploited and can be represented
   by a symmetric 6x6 matrix, whereas dP/dF is an asymmetric 9x9 matrix. The
   two stress-strain pairs need to be carefully profiled against each other as
   this operation might be (one of) the bottleneck(s). */

  /* Adds a scaled derivative of the elastic force on the nodes of this
   element into the given matrix.
   @param[in] state   The FEM state at which to evaluate the elastic force
                      derivatives.
   @param[out] scale  The scaling factor applied to the derivative.
   @param[out] K      The scaled force derivative matrix.
   @pre K != nullptr. */
  void AddScaledElasticForceDerivative(
      const FemStateImpl<ElementType>& state, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> K) const {
    DRAKE_ASSERT(K != nullptr);
    // clang-format off
    /* Let e be the elastic energy, then the ab-th block of the stiffness
     matrix K is given by:
     Kᵃᵇᵢⱼ = d²e/dxᵃᵢdxᵇⱼ = ∫dF/dxᵇⱼ:d²ψ/dF²:dF/dxᵃᵢ + dψ/dF:d²F/dxᵃᵢdxᵇⱼ dX.
     The second term vanishes because Fₖₗ = xᵃₖdSᵃ/dXₗ is linear in x.
     We calculate the first term:
     dF/dxᵇⱼ : d²ψ/dF² : dF/dxᵃᵢ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ.  */
    // clang-format on
    const Data& data = state.element_data(*this);
    // The ab-th 3-by-3 block of K.
    Matrix3<T> K_ab;
    for (int q = 0; q < num_quadrature_points; ++q) {
      /* Notice that Fₖₗ = xᵃₖdSᵃ/dXₗ, so dFₖₗ/dxᵇⱼ = δᵃᵇδₖⱼdSᵃ/dXₗ, and thus
       Kᵃᵇᵢⱼ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ =  dSᵃ/dXₙ dPᵢₙ/dFⱼₗ dSᵇ/dXₗ. */
      for (int a = 0; a < num_nodes; ++a) {
        for (int b = 0; b < num_nodes; ++b) {
          /* Note that the scale is negated here because the tensor contraction
           gives the second derivative of energy, which is the opposite of the
           force derivative. */
          PerformDoubleTensorContraction<T>(
              data.dPdF[q], dSdX_transpose_[q].col(a),
              dSdX_transpose_[q].col(b) * reference_volume_[q] * -scale, &K_ab);
          AccumulateMatrixBlock(K_ab, a, b, K);
        }
      }
    }
  }

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemStateImpl<ElementType>& state,
                      EigenPtr<Vector<T, num_dofs>> residual) const {
    /* residual = Ma-fₑ(x)-fᵥ(x, v)-fₑₓₜ, where M is the mass matrix, fₑ(x) is
     the elastic force, fᵥ(x, v) is the damping force and fₑₓₜ is the external
     force. */
    *residual +=
        mass_matrix_ * this->ExtractElementDofs(this->node_indices(),
                                                state.GetAccelerations());
    this->AddNegativeElasticForce(state, residual);
    AddNegativeDampingForce(state, residual);
    this->AddScaledExternalForce(state, -1.0, residual);
  }

  /* Implements FemElement::DoAddScaledStiffnessMatrix().
   @warning This method calculates a first-order approximation of the stiffness
   matrix. In other words, the contribution of the term ∂fᵥ(x, v)/∂x is ignored
   as it involves complex second derivatives of the elastic force. */
  void DoAddScaledStiffnessMatrix(
      const FemStateImpl<ElementType>& state, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> K) const {
    /* Negate `scale` since stiffness matrix is the negative force derivative.
     */
    this->AddScaledElasticForceDerivative(state, -scale, K);
  }

  /* Implements FemElement::DoAddScaledDampingMatrix(). */
  void DoAddScaledDampingMatrix(
      const FemStateImpl<ElementType>& state, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> D) const {
    /* D = αM + βK, where α is the mass damping coefficient and β is the
     stiffness damping coefficient. */
    const T& alpha = this->damping_model().mass_coeff();
    const T& beta = this->damping_model().stiffness_coeff();
    this->AddScaledMassMatrix(state, scale * alpha, D);
    this->AddScaledStiffnessMatrix(state, scale * beta, D);
  }

  /* Implements FemElement::DoAddScaledMassMatrix(). */
  void DoAddScaledMassMatrix(
      const FemStateImpl<ElementType>&, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> M) const {
    *M += scale * mass_matrix_;
  }

  /* Implements FemElement::ComputeData(). */
  Data DoComputeData(const FemState<T>& state) const {
    Data data;
    data.deformation_gradient_data.UpdateData(CalcDeformationGradient(state));
    this->constitutive_model().CalcElasticEnergyDensity(
        data.deformation_gradient_data, &data.Psi);
    this->constitutive_model().CalcFirstPiolaStress(
        data.deformation_gradient_data, &data.P);
    this->constitutive_model().CalcFirstPiolaStressDerivative(
        data.deformation_gradient_data, &data.dPdF);
    return data;
  }

  /* Calculates the deformation gradient at all quadrature points in this
   element. */
  std::array<Matrix3<T>, num_quadrature_points> CalcDeformationGradient(
      const FemState<T>& state) const {
    std::array<Matrix3<T>, num_quadrature_points> F;
    const Vector<T, num_dofs> element_x =
        this->ExtractElementDofs(this->node_indices(), state.GetPositions());
    const auto& element_x_reshaped =
        Eigen::Map<const Eigen::Matrix<T, 3, num_nodes>>(element_x.data(), 3,
                                                         num_nodes);
    const std::array<typename IsoparametricElementType::JacobianMatrix,
                     num_quadrature_points>
        dxdxi = isoparametric_element_.CalcJacobian(element_x_reshaped);
    for (int q = 0; q < num_quadrature_points; ++q) {
      F[q] = dxdxi[q] * dxidX_[q];
    }
    return F;
  }

  /* Helper function that adds a 3x3 matrix into the 3x3 block in a bigger
   matrix `matrix` with starting row index 3*node_a and starting column index
   3*node_b. Note that this function assumes the pointer `matrix` is not null.
   It also does not check the index it tries to write in `matrix` is valid and
   does not clear any stale data that might exist in `matrix`. */
  static void AccumulateMatrixBlock(
      const Eigen::Ref<const Matrix3<T>>& block, int node_a, int node_b,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> matrix) {
    matrix->template block<3, 3>(3 * node_a, 3 * node_b) += block;
  }

  Eigen::Matrix<T, num_dofs, num_dofs> PrecomputeMassMatrix() const {
    Eigen::Matrix<T, num_dofs, num_dofs> mass =
        Eigen::Matrix<T, num_dofs, num_dofs>::Zero();
    const std::array<Vector<T, num_nodes>, num_quadrature_points>& S =
        isoparametric_element_.GetShapeFunctions();
    /* S_mat is the matrix representation of S. */
    Eigen::Matrix<T, num_nodes, num_quadrature_points> S_mat;
    for (int q = 0; q < num_quadrature_points; ++q) {
      S_mat.col(q) = S[q];
    }
    /* weighted_S stores the shape function weighted by the reference
     volume of the quadrature point. */
    Eigen::Matrix<T, num_nodes, num_quadrature_points> weighted_S(S_mat);
    for (int q = 0; q < num_quadrature_points; ++q) {
      weighted_S.col(q) *= reference_volume_[q];
    }
    /* weighted_SST = weighted_S * Sᵀ. The ij-th entry approximates the integral
     ∫SᵢSⱼ dX */
    const Eigen::Matrix<T, num_nodes, num_nodes> weighted_SST =
        weighted_S * S_mat.transpose();
    constexpr int kDim = 3;
    for (int i = 0; i < num_nodes; ++i) {
      for (int j = 0; j < num_nodes; ++j) {
        mass.template block<kDim, kDim>(kDim * i, kDim * j) =
            Eigen::Matrix<T, kDim, kDim>::Identity() * weighted_SST(i, j) *
            density_;
      }
    }
    return mass;
  }

  /* Computes the gravity force on each node in the element using the stored
   mass and gravity vector. */
  void AddScaledGravityForce(const FemStateImpl<ElementType>& state,
                             const T& scale,
                             EigenPtr<Vector<T, num_dofs>> force) const {
    // TODO(xuchenhan-tri): The calculation here is only required whenever the
    //  gravity vector changes. Consider cahcing the gravity force.
    unused(state);
    constexpr int kDim = 3;
    for (int i = 0; i < num_nodes; ++i) {
      /* The following computation is equivalent to performing the matrix-vector
       multiplication of the mass matrix and the stacked gravity vector. */
      force->template segment<kDim>(kDim * i) +=
          scale * lumped_mass_.template segment<kDim>(kDim * i).cwiseProduct(
                      this->gravity_vector());
    }
  }

  // TODO(xuchenhan-tri): Consider bumping this up into FemElement when new
  //  FemElement types are added.
  /* The quadrature rule used for this element. */
  QuadratureType quadrature_{};
  /* The isoparametric element used for this element. */
  IsoparametricElementType isoparametric_element_{quadrature_.get_points()};
  /* The inverse element Jacobian evaluated at reference configuration at
   the quadrature points in this element. */
  std::array<Eigen::Matrix<T, natural_dimension, 3>, num_quadrature_points>
      dxidX_;
  /* The transpose of the derivatives of the shape functions with respect to the
   reference positions evaluated at the quadrature points in this element. */
  std::array<Eigen::Matrix<T, 3, num_nodes>, num_quadrature_points>
      dSdX_transpose_;
  /* The volume evaluated at reference configuration occupied by the
   quadrature points in this element. To integrate a function f over the
   reference domain, sum f(q)*reference_volume_[q] over all the quadrature
   points q in the element. */
  std::array<T, num_quadrature_points> reference_volume_;
  /* The uniform mass density of the element in the reference configuration with
   unit kg/m³. */
  T density_;
  /* Precomputed mass matrix. */
  Eigen::Matrix<T, num_dofs, num_dofs> mass_matrix_;
  /* The diagonal of the lumped mass matrix (sum over each row). */
  Vector<T, num_dofs> lumped_mass_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
