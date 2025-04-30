#include "drake/manipulation/kuka_iiwa/iiwa_driver_functions.h"

#include <optional>

#include "drake/manipulation/kuka_iiwa/build_iiwa_control.h"
#include "drake/manipulation/kuka_iiwa/iiwa_constants.h"
#include "drake/manipulation/kuka_iiwa/sim_iiwa_driver.h"
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
    const IiwaDriver& driver_config, const std::string& model_instance_name,
    const MultibodyPlant<double>& sim_plant,
    const std::map<std::string, ModelInstanceInfo>& models_from_directives,
    const LcmBuses& lcms, DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  const std::string& arm_name = model_instance_name;
  const std::string& hand_name = driver_config.hand_model_name;
  if (!models_from_directives.contains(arm_name)) {
    throw std::runtime_error(fmt::format(
        "IiwaDriver could not find arm model directive '{}' to actuate",
        arm_name));
  }
  ModelInstanceInfo arm_model = models_from_directives.at(arm_name);
  // Substitute frame names if they are provided in `driver_config`.
  if (driver_config.arm_child_frame_name.has_value()) {
    arm_model.child_frame_name = driver_config.arm_child_frame_name.value();
  }
  std::optional<ModelInstanceInfo> hand_model;
  if (!hand_name.empty()) {
    if (!models_from_directives.contains(hand_name)) {
      throw std::runtime_error(fmt::format(
          "IiwaDriver could not find hand model directive '{}' to actuate",
          hand_name));
    }
    hand_model = models_from_directives.at(hand_name);
    if (driver_config.gripper_parent_frame_name.has_value()) {
      hand_model->parent_frame_name =
          driver_config.gripper_parent_frame_name.value();
    }
  }
  DrakeLcmInterface* lcm =
      lcms.Find("Driver for " + arm_name, driver_config.lcm_bus);
  MultibodyPlant<double>* controller_plant =
      SharedPointerSystem<double>::AddToBuilder(
          builder, manipulation::internal::MakeArmControllerModel(
                       sim_plant, arm_model, hand_model));
  builder->GetMutableSystems().back()->set_name(
      fmt::format("{}_controller_plant", arm_name));
  if (lcm->get_lcm_url() == LcmBuses::kLcmUrlMemqNull) {
    SimIiwaDriver<double>::AddToBuilder(builder, sim_plant,
                                        arm_model.model_instance, driver_config,
                                        *controller_plant);
  } else {
    BuildIiwaControl(builder, lcm, sim_plant, arm_model.model_instance,
                     driver_config, *controller_plant);
  }
}

}  // namespace kuka_iiwa
}  // namespace manipulation
}  // namespace drake
