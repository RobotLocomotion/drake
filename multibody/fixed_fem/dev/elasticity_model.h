#pragma once

#include <array>

#include "drake/multibody/fixed_fem/dev/elasticity_element.h"
#include "drake/multibody/fixed_fem/dev/fem_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The FEM model for static and dynamic 3D elasticity problems. Provides
 methods common to both StaticElasticityModel and DynamicElasticityModel.
 @tparam Element    The type of ElasticityElement used in this %ElasticityModel.
 Element must be derived from ElasticityElement. */
template <class Element>
class ElasticityModel : public FemModel<Element> {
 public:
  static_assert(
      std::is_base_of_v<
          ElasticityElement<typename Element::Traits::IsoparametricElement,
                            typename Element::Traits::Quadrature,
                            typename Element::Traits::ConstitutiveModel,
                            Element, typename Element::Traits>,
          Element>,
      "The template parameter Element must be derived from ElasticityModel.");
  static_assert(Element::Traits::kSpatialDimension == 3,
                "Only 3D elasticity is supported.");
  using T = typename Element::Traits::T;

  /** Calculates the total elastic potential energy (in joules) in this
   %ElasticityModel. */
  T CalcElasticEnergy(const FemState<Element>& state) const {
    T energy(0);
    for (ElementIndex i(0); i < this->num_elements(); ++i) {
      const Element& e = this->element(i);
      energy += e.CalcElasticEnergy(state);
    }
    return energy;
  }

  /** Sets gravity and precomputes gravity force for all elements in the model.
   The default value for gravity is [0, 0, -9.81]. */
  void SetGravity(
      const Vector<T, Element::Traits::kSpatialDimension>& gravity) {
    /* Store gravity so that all elements added after the call to this method
     get the "new" gravity constant. */
    gravity_ = gravity;
    /* Update the precomputed gravity force in elements added before the call to
     this method. */
    for (ElementIndex e(0); e < this->num_elements(); ++e) {
      this->mutable_element(e).PrecomputeGravityForce(gravity);
    }
  }

  const Vector<T, Element::Traits::kSpatialDimension> gravity() const {
    return gravity_;
  }

  /** Calculates the total external force exerted on the %ElasticityModel at
   the given `state`.
   @param[in] state The FemState at which to evaluate the external force.
   @param[out] external_force The external force evaluated at `state`.
   @pre external_force != nullptr.
   @pre The size of the vector behind `external_force` must be `num_dofs()`. */
  void CalcExternalForce(const FemState<Element>& state,
                         EigenPtr<VectorX<T>> external_force) {
    DRAKE_DEMAND(external_force != nullptr);
    DRAKE_DEMAND(external_force->size() == this->num_dofs());
    DRAKE_DEMAND(state.element_cache_size() == this->num_elements());
    /* The values are accumulated in the external_force, so it is important to
     clear the old data. */
    external_force->setZero();
    /* Aliases to improve readability. */
    constexpr int kNumDofs = Element::Traits::kNumDofs;
    constexpr int kNumNodes = Element::Traits::kNumNodes;
    constexpr int kDim = Element::Traits::kSpatialDimension;
    /* Scratch space to store the contribution to the residual from each
     element. */
    Vector<T, kNumDofs> element_force;
    for (ElementIndex e(0); e < this->num_elements(); ++e) {
      element_force.setZero();
      this->element(e).AddExternalForce(state, &element_force);
      const std::array<NodeIndex, kNumNodes>& element_node_indices =
          this->element(e).node_indices();
      for (int i = 0; i < kNumNodes; ++i) {
        const int ei = element_node_indices[i];
        external_force->template segment<kDim>(ei * kDim) +=
            element_force.template segment<kDim>(i * kDim);
      }
    }
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityModel);
  ElasticityModel() = default;
  virtual ~ElasticityModel() = default;

 private:
  /* The gravitational acceleration in the world frame. */
  Vector<T, Element::Traits::kSpatialDimension> gravity_{0, 0, -9.81};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
