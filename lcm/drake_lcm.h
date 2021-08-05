#pragma once

#include <memory>
#include <optional>
#include <string>

#include "lcm/lcm-cpp.hpp"

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace lcm {

/**
 * A wrapper around a *real* LCM instance.
 */
class DrakeLcm : public DrakeLcmInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DrakeLcm);

  /**
   * Constructs using LCM's default URL (either the default hard-coded URL, or
   * else LCM_DEFAULT_URL environment variable if it is set).
   */
  DrakeLcm();

  /**
   * Constructs using the given URL.  If empty, it will use the default URL as
   * per the no-argument constructor.
   */
  explicit DrakeLcm(std::string lcm_url);

  /**
   * (Advanced) Constructs using the given URL, but with the ability to defer
   * launching the receive thread.
   *
   * @param defer_initialization controls whether or not LCM's background
   * receive thread will be launched immediately during the constructor (when
   * false) or deferred until the first time it's needed (when true).  This can
   * be useful if the scheduling configuration for new threads varies between
   * the construction time and first use.  For other constructor overloads,
   * this setting defaults to `false` -- the thread is launched immediately.
   */
  DrakeLcm(std::string lcm_url, bool defer_initialization);

  /**
   * A destructor that forces the receive thread to be stopped.
   */
  ~DrakeLcm() override;

  /**
   * (Advanced.) An accessor to the underlying LCM instance. The returned
   * pointer is guaranteed to be valid for the duration of this object's
   * lifetime.
   */
  ::lcm::LCM* get_lcm_instance();


  void Publish(const std::string&, const void*, int,
               std::optional<double>) override;
  std::string get_lcm_url() const override;
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string&, HandlerFunction) override;
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeAllChannels(
      MultichannelHandlerFunction) override;
  int HandleSubscriptions(int) override;

 private:
  void OnHandleSubscriptionsError(const std::string&) override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lcm
}  // namespace drake
