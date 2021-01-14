#pragma once

#include <array>

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/elasticity_element.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Traits class for FEM static elasticity. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
struct StaticElasticityElementTraits
    : ElasticityElementTraits<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType> {
  /* The static elasticity problem does not involve time derivatives. */
  static constexpr int kOdeOrder = 0;
};

/** The FEM element class for static 3D elasticity problems. Implements the
 the CRTP base class, FemElement.
 @tparam IsoparametricElementType The type of isoparametric element used in this
 %StaticElasticityElement. IsoparametricElementType must be a derived class from
 IsoparametricElement.
 @tparam QuadratureType The type of quadrature rule used in this
 %StaticElasticityElement. QuadratureType must be a derived class from
 Quadrature.
 @tparam ConstitutiveModelType The type of constitutive model used in this
 %StaticElasticityElement. ConstitutiveModelType must be a derived class from
 ConstitutiveModel. */
template <class IsoparametricElementType, class QuadratureType,
          class ConstitutiveModelType>
class StaticElasticityElement final
    : public ElasticityElement<
          IsoparametricElementType, QuadratureType, ConstitutiveModelType,
          StaticElasticityElement<IsoparametricElementType, QuadratureType,
                                  ConstitutiveModelType>,
          StaticElasticityElementTraits<IsoparametricElementType,
                                        QuadratureType,
                                        ConstitutiveModelType>> {
 public:
  /** Assignment and copy constructions are prohibited. Move constructor is
   allowed so that %StaticElasticityElement can be stored in `std::vector`. */
  StaticElasticityElement(const StaticElasticityElement&) = delete;
  StaticElasticityElement(StaticElasticityElement&&) = default;
  const StaticElasticityElement& operator=(const StaticElasticityElement&) =
      delete;
  StaticElasticityElement&& operator=(const StaticElasticityElement&&) = delete;

  using Traits =
      StaticElasticityElementTraits<IsoparametricElementType, QuadratureType,
                                    ConstitutiveModelType>;
  using T = typename Traits::T;

  /** Constructs a new FEM static elasticity element. */
  StaticElasticityElement(
      ElementIndex element_index,
      const std::array<NodeIndex, Traits::kNumNodes>& node_indices,
      const ConstitutiveModelType& constitutive_model,
      const Eigen::Ref<const Eigen::Matrix<T, Traits::kSolutionDimension,
                                           Traits::kNumNodes>>&
          reference_positions)
      : ElasticityElement<
            IsoparametricElementType, QuadratureType, ConstitutiveModelType,
            StaticElasticityElement<IsoparametricElementType, QuadratureType,
                                    ConstitutiveModelType>,
            Traits>(element_index, node_indices, constitutive_model,
                    reference_positions) {}

 private:
  /** Type alias for convenience and readability. */
  using ElementType =
      StaticElasticityElement<IsoparametricElementType, QuadratureType,
                              ConstitutiveModelType>;
  using ElasticityElementType =
      ElasticityElement<IsoparametricElementType, QuadratureType,
                        ConstitutiveModelType, ElementType, Traits>;
  using FemElementType = FemElement<ElementType, Traits>;

  /* Friend the base class so that the public methods in the CRTP base class can
   access the private implementations of this class. */
  friend FemElementType;
  friend class StaticElasticityElementTest;

  /* Implements FemElement::CalcResidual(). */
  void DoCalcResidual(const FemState<ElementType>& state,
                      EigenPtr<Vector<T, Traits::kNumDofs>> residual) const {
    /* residual = -fₑ(x) + fₑₓₜ, where fₑ(x) is the elastic force and fₑₓₜ is
     the external force. */
    ElasticityElementType::AddNegativeElasticForce(state, residual);
    // TODO(xuchenhan-tri): Add external force component. This may involve
    //  moving mass density from DynamicElasticityElement into
    //  ElasticityElement.
  }

  /* Implements FemElement::CalcStiffnessMatrix(). */
  void DoCalcStiffnessMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> K) const {
    ElasticityElementType::AddNegativeElasticForceDerivative(state, K);
  }

  /* Implements FemElement::CalcDampingMatrix(). */
  void DoCalcDampingMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> D) const {
    throw std::logic_error(
        "Static elasticity forms a zero-th order ODE and does not provide a "
        "damping matrix.");
  }

  /* Implements FemElement::CalcMassMatrix(). */
  void DoCalcMassMatrix(
      const FemState<ElementType>& state,
      EigenPtr<Eigen::Matrix<T, Traits::kNumDofs, Traits::kNumDofs>> M) const {
    throw std::logic_error(
        "Static elasticity forms a zero-th order ODE and does not provide a "
        "mass matrix.");
  }
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
