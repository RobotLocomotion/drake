#pragma once

#include <array>
#include <type_traits>

#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/multibody/fem/constitutive_model.h"
#include "drake/multibody/fem/isoparametric_element.h"
#include "drake/multibody/fem/quadrature.h"
#include "drake/multibody/fixed_fem/dev/fem_element.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
// TODO(xuchenhan-tri): ElasticityElement and its derived classes should be
//  placed in internal namespace.
/** The traits class for ElasticityElement. This traits class is meant to be
 inherited by StaticElasticityElementTraits and DynamicElasticityElementTraits
 and is not meant to be used directly by itself. Furthermore, derived elasticity
 elements can *add* to the traits, but should not overwrite any of the members
 defined here. In particular, the derived elasticity elements *must* specify the
 order of the spatially discretized ODE problem, `kOdeOrder`, for the traits to
 be compatible with `FemState`. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct ElasticityElementTraits {
  /* Check that template parameters are of the correct types. */
  static_assert(
      internal::is_isoparametric_element<IsoparametricElementType>::value,
      "The IsoparametricElementType template parameter must be a derived "
      "class of IsoparametricElement");
  static_assert(
      internal::is_quadrature<QuadratureType>::value,
      "The QuadratureType template parameter must be a derived class of "
      "Quadrature<T, NaturalDim, NumLocations>, where NaturalDim can "
      "be 1, 2 or 3.");
  static_assert(
      internal::is_constitutive_model<ConstitutiveModelType>::value,
      "The ConstitutiveModelType template parameter must be a derived "
      "class of ConstitutiveModel");
  /* Check that the scalar types are compatible. */
  static_assert(std::is_same_v<typename IsoparametricElementType::T,
                               typename ConstitutiveModelType::T>);
  /* Check that the number of quadrature points are compatible. */
  static_assert(QuadratureType::num_quadrature_points ==
                IsoparametricElementType::num_sample_locations);
  static_assert(QuadratureType::num_quadrature_points ==
                ConstitutiveModelType::num_locations);
  /* Check that the natural dimensions are compatible. */
  static_assert(IsoparametricElementType::natural_dimension ==
                QuadratureType::natural_dimension);
  /* Only 3D elasticity is supported. */
  static_assert(IsoparametricElementType::spatial_dimension == 3);

  using T = typename ConstitutiveModelType::T;
  using IsoparametricElement = IsoparametricElementType;
  using Quadrature = QuadratureType;
  using ConstitutiveModel = ConstitutiveModelType;

  static constexpr int kNumNodes = IsoparametricElementType::num_nodes;
  static constexpr int kNumQuadraturePoints =
      QuadratureType::num_quadrature_points;
  static constexpr int natural_dimension = QuadratureType::natural_dimension;
  static constexpr int kSpatialDimension =
      IsoparametricElementType::spatial_dimension;
  static constexpr int kSolutionDimension = 3;
  /* The number of degrees of freedom should be equal to the solution dimension
   (which gives the number of degrees of freedom for a single vertex) times
   the number of nodes. */
  static constexpr int kNumDofs = kSolutionDimension * kNumNodes;

  /* The data shared by any elasticity element. Derived classes may extend this
   Data by inheriting it. The ElasticityElement relies on the fields defined in
   Data. Derived elasticity element classes may have Data types with additional
   fields, but must guarantee that these fields remain intact. */
  // TODO(xuchenhan-tri): Enforce the constraint mentioned above with
  //  static_assert with easy-to-parse error messages.
  struct Data {
    typename ConstitutiveModelType::Data deformation_gradient_data;
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

/** The FEM class which encodes the common aspects of static and dynamic 3D
 elasticity problems. It serves as the non-virtual base class for
 DynamicElasticityElement and StaticElasticityElement. This builds on
 FemElement, adding the common elasticity functionality (e.g.,
 CalcElasticEnergy()), but does not implement the FemElement interface (e.g.
 DoCalcResidual()); the details of those methods differ according to dynamic and
 static elasticities. Derived elasticity element types are responsible for
 implementing those. This is similar to an abstract class in that it cannot be
 instantiated itself; only derived class can be.
 @tparam IsoparametricElementType    The type of isoparametric element used in
 this %ElasticityElement. IsoparametricElementType must be a derived class from
 IsoparametricElement.
 @tparam QuadratureType    The type of quadrature rule used in this
 %ElasticityElement. QuadratureType must be a derived class from Quadrature.
 @tparam ConstitutiveModelType    The type of constitutive model used in this
 %ElasticityElement. ConstitutiveModelType must be a derived class from
 ConstitutiveModel.
 @tparam DerivedElement    The concrete FEM element that inherits from
 %ElasticityElement through CRTP.
 @tparam DerivedTraits    The traits class associated with the DerivedElement.
 DerivedTraits should be derived from ElasticityElementTraits. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType, class DerivedElement,
          class DerivedTraits>
class ElasticityElement : public FemElement<DerivedElement, DerivedTraits> {
 public:
  using Traits = DerivedTraits;
  using T = typename Traits::T;
  static_assert(
      std::is_base_of_v<
          ElasticityElementTraits<IsoparametricElementType, QuadratureType,
                                  ConstitutiveModelType>,
          Traits>,
      "The DerivedTraits template parameter must be derived from "
      "ElasticityElementTraits.");
  static_assert(Traits::kSolutionDimension == 3 &&
                    Traits::kSpatialDimension == 3,
                "Only 3D elasticity is supported and thus kSpatialDimension "
                "and kSolutionDimension must be 3.");
  static_assert(Traits::kNumDofs ==
                    Traits::kNumNodes * Traits::kSolutionDimension,
                "kNumDofs must be equal to kNumNodes * kSolutionDimension");

  /** Given the current state, calculates the elastic potential energy (in
   joules) stored in this element. */
  T CalcElasticEnergy(const FemState<DerivedElement>& state) const {
    T elastic_energy = 0;
    const typename Traits::Data& data = state.element_data(derived_element());
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      elastic_energy += reference_volume_[q] * data.Psi[q];
    }
    return elastic_energy;
  }

  /** Accumulates the total external force exerted on the %ElasticityElement at
   the given `state` scaled by `scale` into the output parameter
   `external_force`.
   @pre external_force != nullptr. */
  void AddScaledExternalForce(
      const FemState<DerivedElement>& state, const T& scale,
      EigenPtr<Vector<T, Traits::kNumDofs>> external_force) const {
    DRAKE_ASSERT(external_force != nullptr);
    unused(state);
    /* So far, the only external force is gravity. */
    *external_force += scale * gravity_force_;
  }

  /** Computes the gravity force on each node in the element using the stored
   mass and gravity vector. */
  void PrecomputeGravityForce(
      const Vector<T, Traits::kSpatialDimension>& gravity) {
    constexpr int kDim = Traits::kSpatialDimension;
    // TODO(xuchenhan-tri): Consider caching the lumped mass locally when it is
    //  used for more than computing the gravity force.
    const auto& lumped_mass = mass_matrix_.rowwise().sum().eval();
    for (int i = 0; i < Traits::kNumNodes; ++i) {
      /* The following computation is equivalent to performing the matrix-vector
       multiplication of the mass matrix and the stacked gravity vector. */
      gravity_force_.template segment<kDim>(kDim * i) =
          lumped_mass.template segment<kDim>(kDim * i).cwiseProduct(gravity);
    }
  }

 protected:
  /** Assignment and copy constructions are prohibited. Move constructor is
   allowed so that derived elasticity elements can likewise implement the move
   constructor so they can be stored in `std::vector`. */
  ElasticityElement(const ElasticityElement&) = delete;
  ElasticityElement(ElasticityElement&&) = default;
  const ElasticityElement& operator=(const ElasticityElement&) = delete;
  ElasticityElement&& operator=(const ElasticityElement&&) = delete;

  /** Constructs a new ElasticityElement. In that process, precomputes the mass
   matrix and the gravity force acting on the element. The constructor is made
   protected because ElasticityElement should not be constructed directly. Use
   the constructor of the derived classes instead.
   @param[in] element_index    The index of the new element.
   @param[in] node_indices    The node indices of the nodes of this element.
   @param[in] constitutive_model    The ConstitutiveModel to be used for this
   element.
   @param[in] reference_positions    The positions (in world frame) of the nodes
   of this element in the reference configuration.
   @param[in] denstiy    The mass density of the element with unit kg/m³.
   @param[in] gravity    The gravitational acceleration (in world frame) for the
   new element with unit m/s².
   @pre element_index must be valid.
   @pre density > 0. */
  ElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModelType& constitutive_model,
      const Eigen::Ref<
          const Eigen::Matrix<T, Traits::kSpatialDimension, Traits::kNumNodes>>&
          reference_positions,
      const T& density, const Vector<T, Traits::kSpatialDimension>& gravity)
      : FemElement<DerivedElement, DerivedTraits>(element_index, node_indices),
        constitutive_model_(constitutive_model),
        density_(density) {
    DRAKE_DEMAND(density_ > 0);
    /* Find the Jacobian of the change of variable function X(ξ). */
    const std::array<
        Eigen::Matrix<T, Traits::kSpatialDimension, Traits::natural_dimension>,
        Traits::kNumQuadraturePoints>
        dXdxi = isoparametric_element_.CalcJacobian(reference_positions);
    /* Record the quadrature point volume in reference configuration for each
     quadrature location. */
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      /* The scale to transform quadrature weight in parent coordinates to
       reference coordinates. */
      T volume_scale;
      if constexpr (Traits::natural_dimension == 3) {
        volume_scale = dXdxi[q].determinant();
        /* Degenerate tetrahedron in the initial configuration is not allowed.
         */
        DRAKE_DEMAND(volume_scale > 0);
        // NOLINTNEXTLINE(readability/braces) false positive
      } else if constexpr (Traits::natural_dimension == 2) {
        /* Given the QR decomposition of the Jacobian matrix J = QR, where Q is
         unitary and R is upper triangular, the 2x2 top left corner of R gives
         the in plane deformation of the reference triangle. Its determinant
         provides the ratio of the area of triangle in the reference
         configuration over the area of the triangle in parent domain. */
        Eigen::ColPivHouseholderQR<Eigen::Matrix<T, Traits::kSpatialDimension,
                                                 Traits::natural_dimension>>
            qr(dXdxi[q]);
        volume_scale = abs(qr.matrixR()
                               .topLeftCorner(Traits::natural_dimension,
                                              Traits::natural_dimension)
                               .template triangularView<Eigen::Upper>()
                               .determinant());
      } else if constexpr (Traits::natural_dimension == 1) {
        volume_scale = dXdxi[q].norm();
      } else {
        DRAKE_UNREACHABLE();
      }
      reference_volume_[q] = volume_scale * quadrature_.get_weight(q);
    }

    /* Record the inverse Jacobian at the reference configuration which is used
     in the calculation of deformation gradient. */
    dxidX_ = isoparametric_element_.CalcJacobianPseudoinverse(dXdxi);

    const auto dSdX = isoparametric_element_.CalcGradientInSpatialCoordinates(
        reference_positions);
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      dSdX_transpose_[q] = dSdX[q].transpose();
    }
    /* Gravity force depends on mass, which depends on volume. Therefore we must
     compute volume, mass, gravity in that order. */
    mass_matrix_ = PrecomputeMassMatrix();

    PrecomputeGravityForce(gravity);
  }

  /** Adds the negative elastic force on the nodes of this element into the
   given force vector. The negative elastic force is the derivative of the
   elastic energy (see CalcElasticEnergy()) with respect to the generalized
   positions of the nodes.
   @param[in] state    The FEM state at which to evaluate the negative elastic
   force.
   @param[out] neg_force    The negative force vector.
   @pre neg_force != nullptr.
   @warning It is the responsibility of the caller to initialize neg_force to
   zero appropriately. */
  void AddNegativeElasticForce(
      const FemState<DerivedElement>& state,
      EigenPtr<Vector<T, Traits::kNumDofs>> neg_force) const {
    DRAKE_ASSERT(neg_force != nullptr);
    auto neg_force_matrix = Eigen::Map<
        Eigen::Matrix<T, Traits::kSolutionDimension, Traits::kNumNodes>>(
        neg_force->data(), Traits::kSolutionDimension, Traits::kNumNodes);
    const typename Traits::Data& data = state.element_data(derived_element());
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      /* Negative force is the gradient of energy.
       -f = ∫dΨ/dx = ∫dΨ/dF : dF/dx dX.
       Notice that Fᵢⱼ = xₐᵢdSₐ/dXⱼ, so dFᵢⱼ/dxᵦₖ = δₐᵦδᵢₖdSₐ/dXⱼ,
       and dΨ/dFᵢⱼ = Pᵢⱼ, so the integrand becomes
       PᵢⱼδₐᵦδᵢₖdSₐ/dXⱼ = PₖⱼdSᵦ/dXⱼ = P * dSdX.transpose() */
      neg_force_matrix += reference_volume_[q] * data.P[q] * dSdX_transpose_[q];
    }
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
   multibody/fixed_fem/dev/doc/stiffness_matrix.pdf for the derivation that
   shows the equivalence.
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
   by a symmetric 6x6 matrix, whereas dP/dF is an asymmetric 9x9 matrix. The
   two stress-strain pairs need to be carefully profiled against each other as
   this operation might be (one of) the bottleneck(s). */

  /** Adds the derivative of the negative elastic force on the nodes of this
   element into the given matrix.
   @param[in] state    The FEM state at which to evaluate the negative elastic
   force derivatives.
   @param[out] K    The negative force derivative matrix.
   @pre K != nullptr.
   @warning It is the responsibility of the caller to initialize K to
   zero appropriately. */
  void AddNegativeElasticForceDerivative(
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
    const typename Traits::Data& data = state.element_data(derived_element());
    // The ab-th 3-by-3 block of K.
    Matrix3<T> K_ab;
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      /* Notice that Fₖₗ = xᵃₖdSᵃ/dXₗ, so dFₖₗ/dxᵇⱼ = δᵃᵇδₖⱼdSᵃ/dXₗ, and thus
       Kᵃᵇᵢⱼ = dFₘₙ/dxᵃᵢ dPₘₙ/dFₖₗ dFₖₗ/dxᵇⱼ =  dSᵃ/dXₙ dPᵢₙ/dFⱼₗ dSᵇ/dXₗ. */
      for (int a = 0; a < Traits::kNumNodes; ++a) {
        for (int b = 0; b < Traits::kNumNodes; ++b) {
          PerformDoubleTensorContraction(
              data.dPdF[q], dSdX_transpose_[q].col(a),
              dSdX_transpose_[q].col(b) * reference_volume_[q], &K_ab);
          AccumulateMatrixBlock(K_ab, a, b, K);
        }
      }
    }
  }

  const IsoparametricElementType& isoparametric_element() const {
    return isoparametric_element_;
  }

  const std::array<T, Traits::kNumQuadraturePoints>& reference_volume() const {
    return reference_volume_;
  }

  const Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>& mass_matrix()
      const {
    return mass_matrix_;
  }

  const Vector<T, Traits::kNumDofs>& gravity_force() const {
    return gravity_force_;
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
    data.deformation_gradient_data.UpdateData(CalcDeformationGradient(state));
    constitutive_model_.CalcElasticEnergyDensity(data.deformation_gradient_data,
                                                 &data.Psi);
    constitutive_model_.CalcFirstPiolaStress(data.deformation_gradient_data,
                                             &data.P);
    constitutive_model_.CalcFirstPiolaStressDerivative(
        data.deformation_gradient_data, &data.dPdF);
    return data;
  }

  /* Calculates the deformation gradient at all quadrature points in this
   element. */
  std::array<Matrix3<T>, Traits::kNumQuadraturePoints> CalcDeformationGradient(
      const FemState<DerivedElement>& state) const {
    std::array<Matrix3<T>, Traits::kNumQuadraturePoints> F;
    constexpr int kNumDofs = Traits::kSolutionDimension * Traits::kNumNodes;
    const Vector<T, kNumDofs> element_x =
        this->ExtractElementDofs(this->node_indices(), state.q());
    const auto& element_x_reshaped = Eigen::Map<
        const Eigen::Matrix<T, Traits::kSolutionDimension, Traits::kNumNodes>>(
        element_x.data(), Traits::kSolutionDimension, Traits::kNumNodes);
    const std::array<typename IsoparametricElementType::JacobianMatrix,
                     Traits::kNumQuadraturePoints>
        dxdxi = isoparametric_element_.CalcJacobian(element_x_reshaped);
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
  static void PerformDoubleTensorContraction(
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
    matrix->template block<3, 3>(3 * node_a, 3 * node_b) += block;
  }

  /* Return `this` element statically cast either as StaticElasticityElement or
   DynamicElasticityElement depending on its type. */
  const DerivedElement& derived_element() const {
    return static_cast<const DerivedElement&>(*this);
  }

  Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> PrecomputeMassMatrix()
      const {
    Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> mass =
        Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>::Zero();
    const std::array<Vector<T, Traits::kNumNodes>,
                     Traits::kNumQuadraturePoints>& S =
        isoparametric_element().GetShapeFunctions();
    /* S_mat is the matrix representation of S. */
    Eigen::Matrix<T, Traits::kNumNodes, Traits::kNumQuadraturePoints> S_mat;
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      S_mat.col(q) = S[q];
    }
    /* weighted_S stores the shape function weighted by the reference
     volume of the quadrature point. */
    Eigen::Matrix<T, Traits::kNumNodes, Traits::kNumQuadraturePoints>
        weighted_S(S_mat);
    for (int q = 0; q < Traits::kNumQuadraturePoints; ++q) {
      weighted_S.col(q) *= reference_volume_[q];
    }
    /* weighted_SST = weighted_S * Sᵀ. The ij-th entry approximates the integral
     ∫SᵢSⱼ dX */
    Eigen::Matrix<T, Traits::kNumNodes, Traits::kNumNodes> weighted_SST =
        weighted_S * S_mat.transpose();
    constexpr int kDim = Traits::kSolutionDimension;
    for (int i = 0; i < Traits::kNumNodes; ++i) {
      for (int j = 0; j < Traits::kNumNodes; ++j) {
        mass.template block<kDim, kDim>(kDim * i, kDim * j) =
            Eigen::Matrix<T, kDim, kDim>::Identity() * weighted_SST(i, j) *
            density_;
      }
    }
    return mass;
  }

  // TODO(xuchenhan-tri): Consider bumping this up into FemElement when new
  //  FemElement types are added.
  /* The quadrature rule used for this element. */
  QuadratureType quadrature_;
  /* The isoparametric element used for this element. */
  IsoparametricElementType isoparametric_element_{quadrature_.get_points()};
  /* The constitutive model that describes the stress-strain relationship
   for this element. */
  ConstitutiveModelType constitutive_model_;
  /* The inverse element Jacobian evaluated at reference configuration at
   the quadrature points in this element. */
  std::array<
      Eigen::Matrix<T, Traits::natural_dimension, Traits::kSolutionDimension>,
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
  /* The mass density of the element in the reference configuration with
   unit kg/m³. */
  T density_;
  /* Precomputed mass matrix. */
  Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs> mass_matrix_;
  /* Gravity force on the element. */
  Vector<T, Traits::kNumDofs> gravity_force_;
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
