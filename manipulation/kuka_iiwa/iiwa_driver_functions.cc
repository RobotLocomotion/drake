#include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"

#include <optional>

#include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/util/make_arm_controller_model.h"
#include "drake/systems/primitives/shared_pointer_system.h"

namespace drake {
namespace manipulation {
namespace kuka_iiwa {

using lcm::DrakeLcmInterface;
using multibody::MultibodyPlant;
using multibody::parsing::ModelInstanceInfo;
using systems::DiagramBuilder;
using systems::SharedPointerSystem;
using systems::lcm::LcmBuses;

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
  const ModelInstanceInfo& arm_model = models_from_directives.at(arm_name);
  std::optional<ModelInstanceInfo> hand_model;
  if (!hand_name.empty()) {
    if (models_from_directives.count(hand_name) == 0) {
      throw std::runtime_error(fmt::format(
          "IiwaDriver could not find hand model directive '{}' to actuate",
          hand_name));
    }
    hand_model = models_from_directives.at(hand_name);
  }
  DrakeLcmInterface* lcm =
      lcms.Find("Driver for " + arm_name, driver_config.lcm_bus);
  MultibodyPlant<double>* controller_plant =
      SharedPointerSystem<double>::AddToBuilder(
          builder, internal::MakeArmControllerModel(
              sim_plant, arm_model, hand_model));
  // TODO(jwnimmer-tri) Make desired_iiwa_kp_gains configurable.
  std::optional<Eigen::VectorXd> desired_iiwa_kp_gains;
  const int control_mode = ParseIiwaControlMode(driver_config.control_mode);
  BuildIiwaControl(
      sim_plant, arm_model.model_instance, *controller_plant, lcm, builder,
      driver_config.ext_joint_filter_tau, desired_iiwa_kp_gains,
      control_mode);
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
