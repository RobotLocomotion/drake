#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * %LcmInterfaceSystem acts within a Diagram to allow LcmSubscriberSystem
 * instances to receive data from the network during a simulation.  When its
 * parent Diagram is run via the Simulator, the %LcmSubscriberSystem sources
 * will output new values based on new data received over LCM.
 *
 * This %System has no inputs nor outputs nor state nor parameters; it declares
 * only an update event that pumps LCM messages into their subscribers iff the
 * LCM stack has message(s) waiting.  The subscribers will then update their
 * outputs using their own declared events.
 *
 * Note that because this class implements DrakeLcmInterface, any subscriber
 * registered on that interface will be serviced during simulation, not just
 * `LcmSubscriberSystem`s.
 *
 * @code{cpp}
 * DiagramBuilder<double> builder;
 * auto lcm = builder.AddSystem<LcmInterfaceSystem>();
 * auto subscriber = builder.AddSystem(
 *     LcmSubscriberSystem::Make<lcmt_drake_signal>("channel", lcm));
 * @endcode
 *
 * @ingroup message_passing
 */
class LcmInterfaceSystem final
    : public LeafSystem<double>, public drake::lcm::DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LcmInterfaceSystem)

  /**
   * Constructs using the given URL.  With no URL, uses the LCM_DEFAULT_URL
   * environment variable iff it is set or else the default hard-coded URL.
   */
  explicit LcmInterfaceSystem(std::string lcm_url = {});

  /**
   * Constructs using the given LCM service.  The pointer is aliased by this
   * class and must remain valid for the lifetime of this object.  Users MUST
   * NOT start the receive thread on this object.
   */
  explicit LcmInterfaceSystem(drake::lcm::DrakeLcmInterface*);

  ~LcmInterfaceSystem() final;

  // DrakeLcmInterface overrides.
  void Publish(const std::string&, const void*, int,
               std::optional<double>) final;
  std::shared_ptr<drake::lcm::DrakeSubscriptionInterface>
      Subscribe(const std::string&, HandlerFunction) final;
  int HandleSubscriptions(int) final;

 private:
  explicit LcmInterfaceSystem(std::unique_ptr<drake::lcm::DrakeLcmInterface>);

  void OnHandleSubscriptionsError(const std::string&) final;

  void DoCalcNextUpdateTime(
      const Context<double>&,
      systems::CompositeEventCollection<double>*,
      double*) const final;

  std::unique_ptr<drake::lcm::DrakeLcmInterface> owned_lcm_;
  drake::lcm::DrakeLcmInterface* const lcm_{};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
