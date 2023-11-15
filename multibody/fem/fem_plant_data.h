#pragma once
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/force_density_field.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace fem {

/* FemPlantData stores pointers to the owning MultibodyPlant's resources for an
 FemModel. FemPlantData should not be persisted. It is advisable to acquire it
 for evaluation of FEM computation that depends on the owning MultibodyPlant's
 resources in a limited scope and then discard it. Constructing an
 FemPlantData is cheap. */
template <typename T>
struct FemPlantData {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemPlantData)

  FemPlantData() = default;

  const systems::Context<T>* plant_context{nullptr};
  std::vector<const ForceDensityField<T>*> force_density_fields;
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemPlantData)
