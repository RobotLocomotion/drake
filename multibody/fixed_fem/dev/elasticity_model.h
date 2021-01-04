#pragma once

#include "drake/common/eigen_types.h"
#include "drake/multibody/fixed_fem/dev/fem_model.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** The FEM model for static and dynamic 3D elasticity problems. Provides method
 common to both StaticElasticityElement and DynamicElasticityElement.
 @tparam Element    The type of ElasticityElement used in this %ElasticityModel.
 Element must be derived from ElasticityElement. */
template <class Element>
class ElasticityModel : public FemModel<Element> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElasticityModel);

  using T = typename Element::Traits::T;

  /** Calculates the total elastic potential energy (in joules) in this
   %ElasticityModel. */
  T CalcElasticEnergy(const FemState<Element>& state) const;

 protected:
  ElasticityModel() = default;
  ~ElasticityModel() = default;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
