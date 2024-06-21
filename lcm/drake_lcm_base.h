#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace lcm {

/** A concrete subclass of DrakeInterface that throws for all functions except
the constructor and destructor. This is useful for subclasses that only wish to
implement a subset of the %DrakeLcmInterface features. */
class DrakeLcmBase : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcmBase);
  ~DrakeLcmBase() override;
  std::string get_lcm_url() const override;
  void Publish(const std::string& channel, const void* data, int data_size,
               std::optional<double> time_sec) override;
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string& channel, HandlerFunction) override;
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeMultichannel(
      std::string_view regex, MultichannelHandlerFunction) override;
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeAllChannels(
      MultichannelHandlerFunction) override;
  int HandleSubscriptions(int timeout_millis) override;

 protected:
  DrakeLcmBase();

 private:
  void OnHandleSubscriptionsError(const std::string& error_message) override;
};

}  // namespace lcm
}  // namespace drake
