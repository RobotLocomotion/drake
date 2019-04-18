#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace lcm {

DrakeLcmInterface::DrakeLcmInterface() = default;
DrakeLcmInterface::~DrakeLcmInterface() = default;

DrakeSubscriptionInterface::DrakeSubscriptionInterface() = default;
DrakeSubscriptionInterface::~DrakeSubscriptionInterface() = default;

int LcmHandleSubscriptionsUntil(
    DrakeLcmInterface* const lcm,
    const std::function<bool(void)>& finished,
    const int timeout_millis) {
  int result = 0;
  while (!finished()) {
    result += lcm->HandleSubscriptions(timeout_millis);
  }
  return result;
}

}  // namespace lcm
}  // namespace drake
