#include "drake/systems/lcm/lcm_config_functions.h"

#include <memory>

#include "drake/common/drake_throw.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/shared_pointer_system.h"

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcmParams;
using drake::systems::DiagramBuilder;
using drake::systems::SharedPointerSystem;
using drake::systems::lcm::LcmInterfaceSystem;

namespace drake {
namespace systems {
namespace lcm {

LcmBuses ApplyLcmBusConfig(
    const std::map<std::string, DrakeLcmParams>& lcm_buses,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  LcmBuses result;
  for (const auto& [bus_name, lcm_params] : lcm_buses) {
    // Create the objects. Note that a diagram will delete its systems in the
    // reverse of the order they were added, so the aliasing here from the
    // pumper_system to owner_system is no problem.
    auto* owner_system = builder->AddSystem<SharedPointerSystem<double>>(
        std::make_shared<DrakeLcm>(lcm_params));
    DrakeLcm* const drake_lcm = owner_system->get<DrakeLcm>();
    auto* pumper_system = builder->AddSystem<LcmInterfaceSystem>(drake_lcm);

    // Given the systems useful names, for debugging.
    const std::string canonical_url = drake_lcm->get_lcm_url();
    owner_system->set_name(fmt::format(
        "DrakeLcm(bus_name={}, lcm_url={})",
        bus_name, canonical_url));
    pumper_system->set_name(fmt::format(
        "LcmInterfaceSystem(bus_name={}, lcm_url={})",
        bus_name, canonical_url));

    // Display an update; provide the interface pointer to our caller.
    drake::log()->info("LCM bus '{}' created for URL {}",
        bus_name, canonical_url);
    result.Add(bus_name, pumper_system);
  }
  return result;
}

DrakeLcmInterface* FindOrCreateLcmBus(
    DrakeLcmInterface* forced_result,
    const LcmBuses* lcm_buses,
    DiagramBuilder<double>* builder,
    std::string_view description_of_caller,
    const std::string& bus_name) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  if (forced_result != nullptr) {
    return forced_result;
  }
  if (lcm_buses == nullptr) {
    if (bus_name != "default") {
      throw std::runtime_error(fmt::format(
          "{} requested a non-default LCM bus '{}' but did not provide an"
          " LcmBuses object to locate it",
          description_of_caller, bus_name));
    }
    auto* owner_system = builder->AddSystem<SharedPointerSystem<double>>(
        std::make_shared<DrakeLcm>());
    return owner_system->get<DrakeLcm>();
  }
  return lcm_buses->Find(description_of_caller, bus_name);
}

}  // namespace lcm
}  // namespace systems
}  // namespace drake
