#pragma once

#include <array>
#include <type_traits>
#include <utility>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fem/fem_element.h"
#include "drake/multibody/fem/isoparametric_element.h"
#include "drake/multibody/fem/quadrature.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

// TODO(xuchenhan-tri): Encapsulate the the memory layout of 4th order tensors
//  and the contraction operation in a FourthOrderTensor class.
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

/* The data struct that stores per element data for VolumetricElement. See
 FemElement for the requirement. We define it here instead of nesting it in the
 traits class below due to #17109. */
template <typename ConstitutiveModelType, int num_dofs,
          int num_quadrature_points>
struct VolumetricElementData {
  using T = typename ConstitutiveModelType::T;
  /* The states evaluated at nodes of the element. */
  Vector<T, num_dofs> element_q;
  Vector<T, num_dofs> element_q0;
  Vector<T, num_dofs> element_v;
  Vector<T, num_dofs> element_a;
  typename ConstitutiveModelType::Data deformation_gradient_data;
  /* The elastic energy density evaluated at quadrature points. Note that this
   is energy per unit of "reference" volume. */
  std::array<T, num_quadrature_points> Psi;
  /* The first Piola stress evaluated at quadrature points. */
  std::array<Matrix3<T>, num_quadrature_points> P;
  /* The derivative of first Piola stress with respect to the deformation
   gradient evaluated at quadrature points. */
  std::array<Eigen::Matrix<T, 9, 9>, num_quadrature_points> dPdF;
};

/* Forward declaration needed for defining the traits below. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
class VolumetricElement;

/* The traits class for volumetric elasticity FEM element. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct FemElementTraits<VolumetricElement<
    IsoparametricElementType, QuadratureType, ConstitutiveModelType>> {
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
  /* Codimensional objects are not fully supported yet. */
  static_assert(IsoparametricElementType::natural_dimension == 3,
                "The natural dimension of the isoparametric element must be 3 "
                "for volumetric FEM elements. Codimensional objects are not "
                "yet supported.");

  using T = typename ConstitutiveModelType::T;
  using ConstitutiveModel = ConstitutiveModelType;

  static constexpr int num_nodes = IsoparametricElementType::num_nodes;
  static constexpr int num_quadrature_points =
      QuadratureType::num_quadrature_points;
  static constexpr int natural_dimension = QuadratureType::natural_dimension;
  /* The number of degrees of freedom is equal to the spatial dimension (which
   gives the number of degrees of freedom for a single node) times the number of
   nodes. */
  static constexpr int num_dofs = 3 * num_nodes;

  using Data = VolumetricElementData<ConstitutiveModelType, num_dofs,
                                     num_quadrature_points>;
};

/* This class models a single 3D elasticity FEM element in which the
 displacement of the material can be interpolated from that of the element nodes
 using the isoparametric shape function.
 @tparam IsoparametricElementType  The type of isoparametric element used in
                                   this VolumetricElement.
                                   IsoparametricElementType must derived
                                   from IsoparametricElement.
 @tparam QuadratureType  The type of quadrature rule used in this
                         VolumetricElement. QuadratureType must be derived from
                         Quadrature.
 @tparam ConstitutiveModelType  The type of constitutive model used in this
                                VolumetricElement. ConstitutiveModelType must be
                                derived from ConstitutiveModel. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
class VolumetricElement
    : public FemElement<VolumetricElement<
          IsoparametricElementType, QuadratureType, ConstitutiveModelType>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VolumetricElement);

  using ElementType = VolumetricElement<IsoparametricElementType,
                                        QuadratureType, ConstitutiveModelType>;
  using IsoparametricElement = IsoparametricElementType;
  using Quadrature = QuadratureType;

  using Traits = FemElementTraits<ElementType>;
  using Data = typename Traits::Data;
  using T = typename Traits::T;
  static constexpr int natural_dimension = Traits::natural_dimension;
  static constexpr int kSpatialDimension = 3;
  static constexpr int num_quadrature_points = Traits::num_quadrature_points;
  static constexpr int num_dofs = Traits::num_dofs;
  static constexpr int num_nodes = Traits::num_nodes;

  /* Constructs a new VolumetricElement. In that process, precomputes the mass
   matrix and the gravity force acting on the element.
   @param[in] node_indices         The node indices of the nodes of this
                                   element.
   @param[in] constitutive_model   The ConstitutiveModel to be used for this
                                   element.
   @param[in] reference_positions  The positions (in world frame) of the nodes
                                   of this element in the reference
                                   configuration. The positions must be such
                                   that the element is not degenerate or
                                   inverted.
   @param[in] density              The mass density of the element with unit
                                   kg/m³.
   @param[in] damping_model        The DampingModel to be used for this element.
   @pre element_index and node_indices are valid.
   @pre density > 0. */
  VolumetricElement(const std::array<FemNodeIndex, num_nodes>& node_indices,
                    ConstitutiveModelType constitutive_model,
                    const Eigen::Ref<const Eigen::Matrix<T, 3, num_nodes>>&
                        reference_positions,
                    T density, DampingModel<T> damping_model)
      : FemElement<ElementType>(node_indices, std::move(constitutive_model),
                                std::move(damping_model)),
        density_(std::move(density)) {
    DRAKE_DEMAND(density_ > 0);
    /* Computes the Jacobian of the change of variable function X(ξ). */
    const std::array<Eigen::Matrix<T, 3, natural_dimension>,
                     num_quadrature_points>
        dXdxi = isoparametric_element_.CalcJacobian(reference_positions);
    /* Record the quadrature point volume in reference configuration for each
     quadrature location. */
    for (int q = 0; q < num_quadrature_points; ++q) {
      // TODO(xuchenhan-tri): The volume scale calculation should be in
      // IsoparametricElement.
      /* The scale to transform quadrature weight in parent coordinates to
       reference coordinates. */
      T volume_scale;
      volume_scale = dXdxi[q].determinant();
      /* Degenerate element in the initial configuration is not allowed. */
      DRAKE_DEMAND(volume_scale > 0);
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

  /* Calculates the elastic potential energy (in joules) stored in this element
   using the given `data`. */
  T CalcElasticEnergy(const Data& data) const {
    T elastic_energy = 0;
    for (int q = 0; q < num_quadrature_points; ++q) {
      elastic_energy += reference_volume_[q] * data.Psi[q];
    }
    return elastic_energy;
  }

  /* Computes the scaled gravity force on each node in the element using the
   pre-calculated mass matrix with the given `gravity_vector`. */
  void AddScaledGravityForce(const Data&, const T& scale,
                             const Vector3<T>& gravity_vector,
                             EigenPtr<Vector<T, num_dofs>> force) const {
    constexpr int kDim = 3;
    for (int a = 0; a < num_nodes; ++a) {
      /* The following computation is equivalent to performing the matrix-vector
       multiplication of the mass matrix and the stacked gravity vector. */
      force->template segment<kDim>(kDim * a) +=
          scale * lumped_mass_.template segment<kDim>(kDim * a).cwiseProduct(
                      gravity_vector);
    }
  }

 private:
  /* Friend the base class so that FemElement::DoFoo() can reach its
   implementation. */
  friend FemElement<ElementType>;
  friend class VolumetricElementTest;

  /* Adds the negative elastic force on the nodes of this element into the
   given force vector. The negative elastic force is the derivative of the
   elastic energy (see CalcElasticEnergy()) with respect to the generalized
   positions of the nodes.
   @param[in] data            The FEM data on this element used to evaluate the
                              negative elastic force.
   @param[in, out] neg_force  The negative force vector to be added to.
   @pre neg_force != nullptr. */
  void AddNegativeElasticForce(const Data& data,
                               EigenPtr<Vector<T, num_dofs>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    auto neg_force_matrix = Eigen::Map<Eigen::Matrix<T, 3, num_nodes>>(
        neg_force->data(), 3, num_nodes);
    for (int q = 0; q < num_quadrature_points; ++q) {
      /* Negative force is the gradient of energy.
       -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
       Notice that Fᵢⱼ = xᵃᵢdSᵃ/dXⱼ, so dFᵢⱼ/dxᵇₖ = δᵃᵇδᵢₖdSᵃ/dXⱼ,
       and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
       PᵢⱼδᵃᵇδᵢₖdSᵃ/dXⱼ = PₖⱼdSᵇ/dXⱼ = P * dSdX.transpose() */
      neg_force_matrix += reference_volume_[q] * data.P[q] * dSdX_transpose_[q];
    }
  }

  /* Adds the negative damping force on the nodes of this element into the given
   `negative_damping_force`. The negative damping force is given by the product
   of the damping matrix with the velocity of the nodes.
   @param[in] data            The FEM data on this element used to evaluate the
                              negative damping force.
   @param[in, out] neg_force  The negative force vector to be added to.
   @pre neg_force != nullptr. */
  void AddNegativeDampingForce(const Data& data,
                               EigenPtr<Vector<T, num_dofs>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    Eigen::Matrix<T, num_dofs, num_dofs> damping_matrix =
        Eigen::Matrix<T, num_dofs, num_dofs>::Zero();
    // TODO(xuchenhan-tri): We should cache the dampign matrix to avoid
    // repeatedly calculate the mass and the stiffness matrix.
    this->AddScaledDampingMatrix(data, 1, &damping_matrix);
    /* Note that the damping force fᵥ = -D * v, where D is the damping matrix.
     As we are accumulating the negative damping force here, the `+=` sign
     should be used. */
    *neg_force += damping_matrix * data.element_v;
  }

  /* The matrix calculated here is the same as the stiffness matrix
   calculated in [Bonet, 2016] equation (9.50b) without the external force
   component.
   Without the external force component, (9.50b) reads Kₐᵦ = Kₐᵦ,c + Kₐᵦ,σ.
   Kₐᵦ,c is given by ∫dSᵃ/dxₖ cᵢₖⱼₗ dSᵇ/dxₗ dx (9.35), and
   Kₐᵦ,σ is given by ∫dSᵃ/dxₖ σₖₗ dSᵇ/dxₗ dx (9.44c). Notice that we use S to
   denote shape functions whereas [Bonet, 2016] uses N.
   The negative force derivative we calculate here is given by ∫ dF/dxᵇ :
   dP/dF : dF/dxᵃ dX. The calculation uses a different conjugate pair, but is
   analytically equal to Kₐᵦ,c + Kₐᵦ,σ. See
   multibody/fem/doc/stiffness_matrix.tex for the derivation that
   shows the equivalence.

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
   @param[in] data    The FEM data on this element used to evaluate the elastic
                      force derivatives.
   @param[in] scale   The scaling factor applied to the derivative.
   @param[in, out] K  The scaled force derivative matrix to be added to.
   @pre K != nullptr. */
  void AddScaledElasticForceDerivative(
      const Data& data, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> K) const {
    DRAKE_ASSERT(K != nullptr);
    // clang-format off
    /* Let e be the elastic energy, then the elastic force f is given by

     fᵃᵢ = -de/dxᵃᵢ = -∫dψ/dxᵃᵢ dX = -∫dψ/dF:dF/dxᵃᵢ dX.

     and the ab-th block of the stiffness matrix K is given by:
     Kᵃᵇᵢⱼ = dfᵃᵢ/dxᵇⱼ
           = d²e/dxᵃᵢdxᵇⱼ = ∫dF/dxᵇⱼ:d²ψ/dF²:dF/dxᵃᵢ + dψ/dF:d²F/dxᵃᵢdxᵇⱼ dX.

     The second term vanishes because Fₖₗ = xᵃₖdSᵃ/dXₗ is linear in x.
     We calculate the first term:
     dF/dxᵇⱼ : d²ψ/dF² : dF/dxᵃᵢ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ.  */
    // clang-format on
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

  /* Implements FemElement::CalcInverseDynamics(). */
  void DoCalcInverseDynamics(const Data& data,
                             EigenPtr<Vector<T, num_dofs>> residual) const {
    /* residual = Ma-fₑ(x)-fᵥ(x, v), where M is the mass matrix, fₑ(x) is
     the elastic force, and fᵥ(x, v) is the damping force. */
    *residual += mass_matrix_ * data.element_a;
    this->AddNegativeElasticForce(data, residual);
    AddNegativeDampingForce(data, residual);
  }

  /* Implements FemElement::DoAddScaledStiffnessMatrix().
   @warning This method calculates a first-order approximation of the stiffness
   matrix. In other words, the contribution of the term ∂fᵥ(x, v)/∂x is ignored
   as it involves complex second derivatives of the elastic force. */
  void DoAddScaledStiffnessMatrix(
      const Data& data, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> K) const {
    /* Negate `scale` since stiffness matrix is the negative force derivative.
     */
    this->AddScaledElasticForceDerivative(data, -scale, K);
  }

  /* Implements FemElement::DoAddScaledMassMatrix(). */
  void DoAddScaledMassMatrix(
      const Data&, const T& scale,
      EigenPtr<Eigen::Matrix<T, num_dofs, num_dofs>> M) const {
    *M += scale * mass_matrix_;
  }

  /* Implements FemElement::ComputeData(). */
  Data DoComputeData(const FemState<T>& state) const {
    Data data;
    data.element_q = this->ExtractElementDofs(state.GetPositions());
    data.element_q0 =
        this->ExtractElementDofs(state.GetPreviousStepPositions());
    data.element_v = this->ExtractElementDofs(state.GetVelocities());
    data.element_a = this->ExtractElementDofs(state.GetAccelerations());
    data.deformation_gradient_data.UpdateData(
        CalcDeformationGradient(data.element_q),
        CalcDeformationGradient(data.element_q0));
    this->constitutive_model().CalcElasticEnergyDensity(
        data.deformation_gradient_data, &data.Psi);
    this->constitutive_model().CalcFirstPiolaStress(
        data.deformation_gradient_data, &data.P);
    this->constitutive_model().CalcFirstPiolaStressDerivative(
        data.deformation_gradient_data, &data.dPdF);
    return data;
  }

  /* Calculates the deformation gradient at all quadrature points in this
   element.
   @param[in] element_q  The positions of the nodes of the element in a flat
   vector. */
  std::array<Matrix3<T>, num_quadrature_points> CalcDeformationGradient(
      const Eigen::Ref<const VectorX<T>>& element_q) const {
    std::array<Matrix3<T>, num_quadrature_points> F;
    const auto& element_q_reshaped =
        Eigen::Map<const Eigen::Matrix<T, 3, num_nodes>>(element_q.data(), 3,
                                                         num_nodes);
    const std::array<typename IsoparametricElementType::JacobianMatrix,
                     num_quadrature_points>
        dxdxi = isoparametric_element_.CalcJacobian(element_q_reshaped);
    for (int quad = 0; quad < num_quadrature_points; ++quad) {
      F[quad] = dxdxi[quad] * dxidX_[quad];
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
    Eigen::Matrix<T, num_dofs, num_dofs> mass_matrix =
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
    for (int a = 0; a < num_nodes; ++a) {
      for (int b = 0; b < num_nodes; ++b) {
        mass_matrix.template block<kDim, kDim>(kDim * a, kDim * b) =
            Eigen::Matrix<T, kDim, kDim>::Identity() * weighted_SST(a, b) *
            density_;
      }
    }
    return mass_matrix;
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
  // TODO(xuchenhan-tri): Consider exposing this through an accessor if it turns
  // out to be useful.
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
  /* The diagonal of the lumped mass matrix (sum over each row). Only used for
   calculating gravity forces at the moment. */
  Vector<T, num_dofs> lumped_mass_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
