#include "drake/manipulation/schunk_wsg/schunk_wsg_driver_functions.h"

#include "drake/manipulation/schunk_wsg/build_schunk_wsg_control.h"

namespace drake {
namespace manipulation {
namespace schunk_wsg {

using lcm::DrakeLcmInterface;
using multibody::MultibodyPlant;
using multibody::parsing::ModelInstanceInfo;
using systems::DiagramBuilder;
using systems::lcm::LcmBuses;

void ApplyDriverConfig(
    const SchunkWsgDriver& driver_config,
    const std::string& model_instance_name,
    const MultibodyPlant<double>& sim_plant,
    const std::map<std::string, ModelInstanceInfo>& models_from_directives,
    const LcmBuses& lcms,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  const std::string& hand_name = model_instance_name;
  if (models_from_directives.count(hand_name) == 0) {
    throw std::runtime_error(fmt::format(
        "SchunkWsgDriver could not find hand model directive '{}' to actuate",
        hand_name));
  }
  DrakeLcmInterface* lcm =
      lcms.Find("Driver for " + hand_name, driver_config.lcm_bus);
  const ModelInstanceInfo& hand_model = models_from_directives.at(hand_name);
  BuildSchunkWsgControl(
      sim_plant, hand_model.model_instance, lcm, builder,
      driver_config.pid_gains);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
