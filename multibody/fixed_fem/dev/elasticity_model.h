#pragma once

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

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityModel);
  ElasticityModel() = default;
  virtual ~ElasticityModel() = default;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
