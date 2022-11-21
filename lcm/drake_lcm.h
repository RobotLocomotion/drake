#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_params.h"

#ifndef DRAKE_DOXYGEN_CXX
namespace lcm {
// We don't want to pollute our Drake headers with the include paths for either
// @lcm or @glib, so we forward-declare the `class ::lcm::LCM` for use only by
// DrakeLcm::get_lcm_instance() -- an advanced function that is rarely used.
class LCM;
}  // namespace lcm
#endif

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
   * Constructs using the given parameters.
   */
  explicit DrakeLcm(const DrakeLcmParams& params);

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
