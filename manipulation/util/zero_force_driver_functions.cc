#include "sim/common/zero_force_driver_functions.h"

#include "drake/common/eigen_types.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace anzu {
namespace sim {

using drake::multibody::MultibodyPlant;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::parsing::ModelInstanceInfo;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmBuses;

void ApplyDriverConfig(
    const ZeroForceDriver& driver_config,
    const std::string& model_instance_name,
    const MultibodyPlant<double>& sim_plant,
    const std::map<std::string, ModelInstanceInfo>& models_from_directives,
    const LcmBuses& /* lcms */,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  if (models_from_directives.count(model_instance_name) == 0) {
    throw std::runtime_error(fmt::format(
        "ZeroForceDriver could not find model directive '{}' to actuate",
        model_instance_name));
  }
  const ModelInstanceInfo& model_info =
      models_from_directives.at(model_instance_name);
  const ModelInstanceIndex& model_instance = model_info.model_instance;
  const int num_dofs = sim_plant.num_actuated_dofs(model_instance);
  // Fail-fast if we try to actuate an inert model.
  DRAKE_THROW_UNLESS(num_dofs > 0);
  auto actuation =
      builder->AddSystem<drake::systems::ConstantVectorSource<double>>(
          drake::VectorX<double>::Zero(num_dofs));
  actuation->set_name("zero_force_source_for_" + model_instance_name);
  builder->Connect(actuation->get_output_port(),
                   sim_plant.get_actuation_input_port(model_instance));
}

}  // namespace sim
}  // namespace anzu
