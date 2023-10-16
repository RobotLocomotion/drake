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

void ApplyDriverConfig(const SchunkWsgDriver& driver_config,
                       const std::string& model_instance_name,
                       const MultibodyPlant<double>& sim_plant,
                       const std::map<std::string, ModelInstanceInfo>&,
                       const LcmBuses& lcms, DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  DrakeLcmInterface* lcm =
      lcms.Find("Driver for " + model_instance_name, driver_config.lcm_bus);
  BuildSchunkWsgControl(sim_plant,
                        sim_plant.GetModelInstanceByName(model_instance_name),
                        lcm, builder, driver_config.pid_gains);
}

}  // namespace schunk_wsg
}  // namespace manipulation
}  // namespace drake
