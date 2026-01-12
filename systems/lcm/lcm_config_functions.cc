#include "drake/systems/lcm/lcm_config_functions.h"

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/shared_pointer_system.h"

using drake::lcm::DrakeLcm;
using drake::lcm::DrakeLcmInterface;
using drake::lcm::DrakeLcmParams;
using drake::lcm::DrakeSubscriptionInterface;
using drake::systems::DiagramBuilder;
using drake::systems::SharedPointerSystem;
using drake::systems::lcm::LcmInterfaceSystem;

namespace drake {
namespace systems {
namespace lcm {
namespace {

/* A no-op implementation of lcm::DrakeSubscriptionInterface. */
class NoopDrakeSubscription final : public DrakeSubscriptionInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NoopDrakeSubscription);
  NoopDrakeSubscription() = default;
  void set_unsubscribe_on_delete(bool) final {}
  void set_queue_capacity(int) final {}
};

/* A no-op implementation of lcm::DrakeLcmInterface. */
class NoopDrakeLcm final : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NoopDrakeLcm);
  NoopDrakeLcm() = default;
  std::string get_lcm_url() const { return LcmBuses::kLcmUrlMemqNull; }
  void Publish(const std::string&, const void*, int,
               std::optional<double>) final {}
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(const std::string&,
                                                        HandlerFunction) final {
    return MakeSubscription();
  }
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeMultichannel(
      std::string_view, MultichannelHandlerFunction) final {
    return MakeSubscription();
  }
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeAllChannels(
      MultichannelHandlerFunction) final {
    return MakeSubscription();
  }
  int HandleSubscriptions(int) final { return 0; }

 private:
  void OnHandleSubscriptionsError(const std::string&) final {}
  std::shared_ptr<DrakeSubscriptionInterface> MakeSubscription() const {
    return std::make_shared<NoopDrakeSubscription>();
  }
};

}  // namespace

LcmBuses ApplyLcmBusConfig(
    const std::map<std::string, DrakeLcmParams>& lcm_buses,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  LcmBuses result;
  for (const auto& [bus_name, lcm_params] : lcm_buses) {
    // Create the DrakeLcm.
    std::shared_ptr<DrakeLcmInterface> drake_lcm =
        std::make_shared<DrakeLcm>(lcm_params);
    const std::string canonical_url = drake_lcm->get_lcm_url();

    // Special case when the user has requested a no-op instance.
    const bool is_noop = (canonical_url == LcmBuses::kLcmUrlMemqNull);
    if (is_noop) {
      drake_lcm = std::make_shared<NoopDrakeLcm>();
    }

    // Stash the DrakeLcm in the DiagramBuilder (so its lifetime matches that
    // of the eventual Diagram).
    auto* owner_system =
        builder->AddSystem<SharedPointerSystem<double>>(drake_lcm);
    owner_system->set_name(fmt::format("DrakeLcm(bus_name={})", bus_name));

    // We should not pump a null LCM, so we should return early now.
    if (is_noop) {
      drake::log()->debug("Dummy LCM bus '{}' created", bus_name);
      result.Add(bus_name, drake_lcm.get());
      continue;
    }

    // Create the LcmInterfaceSystem. Note that a diagram deletes its systems in
    // the reverse of the order they were added, so the aliasing here from the
    // pumper_system to owner_system is no problem.
    auto* pumper_system =
        builder->AddSystem<LcmInterfaceSystem>(drake_lcm.get());
    pumper_system->set_name(
        fmt::format("LcmInterfaceSystem(bus_name={})", bus_name));

    // Display an update; provide the interface pointer to our caller.
    drake::log()->info("LCM bus '{}' created for URL {}", bus_name,
                       canonical_url);
    result.Add(bus_name, pumper_system);
  }
  return result;
}

LcmBuses ApplyLcmBusConfig(
    const std::map<std::string, std::optional<DrakeLcmParams>>& lcm_buses,
    DiagramBuilder<double>* builder) {
  DRAKE_THROW_UNLESS(builder != nullptr);
  std::map<std::string, drake::lcm::DrakeLcmParams> new_map;
  for (const auto& [bus_name, lcm_params] : lcm_buses) {
    if (lcm_params.has_value()) {
      new_map.emplace(bus_name, *lcm_params);
    } else {
      new_map.emplace(bus_name,
                      DrakeLcmParams{.lcm_url = LcmBuses::kLcmUrlMemqNull});
    }
  }
  return ApplyLcmBusConfig(new_map, builder);
}

DrakeLcmInterface* FindOrCreateLcmBus(DrakeLcmInterface* forced_result,
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
