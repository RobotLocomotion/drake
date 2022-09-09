#include "sim/common/iiwa_driver_functions.h"

#include "drake/systems/primitives/shared_pointer_system.h"
#include "sim/common/build_iiwa_control.h"
#include "sim/common/make_arm_controller_model.h"

namespace anzu {
namespace sim {

using drake::lcm::DrakeLcmInterface;
using drake::multibody::MultibodyPlant;
using drake::multibody::parsing::ModelInstanceInfo;
using drake::systems::DiagramBuilder;
using drake::systems::SharedPointerSystem;
using drake::systems::lcm::LcmBuses;

void ApplyDriverConfig(
    const IiwaDriver& driver_config,
    const std::string& model_instance_name,
    const MultibodyPlant<double>& sim_plant,
    const std::map<std::string, ModelInstanceInfo>& models_from_directives,
    const LcmBuses& lcms,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  const std::string& arm_name = model_instance_name;
  const std::string& hand_name = driver_config.hand_model_name;
  if (models_from_directives.count(arm_name) == 0) {
    throw std::runtime_error(fmt::format(
        "IiwaDriver could not find arm model directive '{}' to actuate",
        arm_name));
  }
  if (models_from_directives.count(hand_name) == 0) {
    throw std::runtime_error(fmt::format(
        "IiwaDriver could not find hand model directive '{}' to actuate",
        hand_name));
  }
  drake::lcm::DrakeLcmInterface* lcm =
      lcms.Find("Driver for " + arm_name, driver_config.lcm_bus);
  const ModelInstanceInfo& arm_model = models_from_directives.at(arm_name);
  const ModelInstanceInfo& hand_model = models_from_directives.at(hand_name);
  MultibodyPlant<double>* controller_plant =
      SharedPointerSystem<double>::AddToBuilder(
          builder, internal::MakeArmControllerModel(
              sim_plant, arm_model, hand_model));
  // TODO(jeremy.nimmer) Make desired_iiwa_kp_gains configurable.
  BuildIiwaControl(
      sim_plant, arm_model.model_instance, *controller_plant, lcm, builder,
      driver_config.ext_joint_filter_tau);
}

}  // namespace sim
}  // namespace anzu
