#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace lcm {

/**
 * A *mock* LCM instance. This only manipulates LCM messages in memory, not on
 * the wire.  It is similar to a DrakeLcm object with a "memq://" URL, but is
 * guaranteed to behave deterministically (without a hidden background thread).
 */
class DrakeMockLcm : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeMockLcm);

  DrakeMockLcm();

  void Publish(const std::string&, const void*, int,
               std::optional<double>) override;
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string&, HandlerFunction) override;
  int HandleSubscriptions(int) override;

 private:
  struct LastPublishedMessage {
    std::vector<uint8_t> data{};
    bool handled{};
  };

  // Use an ordered collection so that HandleSubscriptions is deterministic.
  std::map<std::string, LastPublishedMessage> last_published_messages_;

  // Maps the channel name to the subscriber.
  std::multimap<std::string, HandlerFunction> subscriptions_;
};

}  // namespace lcm
}  // namespace drake
