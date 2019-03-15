#pragma once

#include <memory>
#include <string>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * XXX
 *
 * No inputs, no outputs, no state, etc.  Declares _only_ an unrestricted
 * update event that pumps LCM messages into their subscribers iff the LCM
 * stack has message(s) waiting.
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

  /**
   * Returns the LCM URL passed into the constructor, if any; this can be empty.
   */
  std::string get_requested_lcm_url() const;

  // DrakeLcmInterface overrides.
  void Publish(const std::string&, const void*, int, optional<double>) final;
  void Subscribe(const std::string&, HandlerFunction) final;
  int HandleSubscriptions(int) final;

 protected:
  // LeafSystem overrides.
  void DoCalcNextUpdateTime(
      const Context<double>&,
      systems::CompositeEventCollection<double>*,
      double*) const final;

 private:
  explicit LcmInterfaceSystem(std::unique_ptr<drake::lcm::DrakeLcmInterface>);

  std::string url_;
  std::unique_ptr<drake::lcm::DrakeLcmInterface> owned_lcm_;
  drake::lcm::DrakeLcmInterface* lcm_{};
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
