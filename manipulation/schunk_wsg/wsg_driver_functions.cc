#include "sim/common/wsg_driver_functions.h"

#include "sim/common/build_wsg_control.h"

namespace anzu {
namespace sim {

using drake::lcm::DrakeLcmInterface;
using drake::multibody::MultibodyPlant;
using drake::multibody::parsing::ModelInstanceInfo;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmBuses;

void ApplyDriverConfig(
    const WsgDriver& driver_config,
    const std::string& model_instance_name,
    const MultibodyPlant<double>& sim_plant,
    const std::map<std::string, ModelInstanceInfo>& models_from_directives,
    const LcmBuses& lcms,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  const std::string& hand_name = model_instance_name;
  if (models_from_directives.count(hand_name) == 0) {
    throw std::runtime_error(fmt::format(
        "WsgDriver could not find hand model directive '{}' to actuate",
        hand_name));
  }
  drake::lcm::DrakeLcmInterface* lcm =
      lcms.Find("Driver for " + hand_name, driver_config.lcm_bus);
  const ModelInstanceInfo& hand_model = models_from_directives.at(hand_name);
  BuildWsgControl(
      sim_plant, hand_model.model_instance, lcm, builder,
      driver_config.pid_gains);
}

}  // namespace sim
}  // namespace anzu
