#include "drake/manipulation/util/zero_force_driver_functions.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
namespace manipulation {

using multibody::ModelInstanceIndex;
using multibody::MultibodyPlant;
using multibody::parsing::ModelInstanceInfo;
using systems::ConstantVectorSource;
using systems::DiagramBuilder;
using systems::lcm::LcmBuses;

void ApplyDriverConfig(const ZeroForceDriver&,
                       const std::string& model_instance_name,
                       const MultibodyPlant<double>& sim_plant,
                       const std::map<std::string, ModelInstanceInfo>&,
                       const LcmBuses&, DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  const ModelInstanceIndex& model_instance =
      sim_plant.GetModelInstanceByName(model_instance_name);
  const int num_dofs = sim_plant.num_actuated_dofs(model_instance);
  // Fail-fast if we try to actuate an inert model.
  DRAKE_THROW_UNLESS(num_dofs > 0);
  auto actuation = builder->AddSystem<ConstantVectorSource<double>>(
      VectorX<double>::Zero(num_dofs));
  actuation->set_name("zero_force_source_for_" + model_instance_name);
  builder->Connect(actuation->get_output_port(),
                   sim_plant.get_actuation_input_port(model_instance));
}

}  // namespace manipulation
}  // namespace drake
