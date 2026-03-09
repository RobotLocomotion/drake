#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "drake/common/drake_copyable.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_lcm_params.h"

namespace drake {
namespace lcm {

/**
 * A wrapper around a *real* LCM instance.
 *
 * See \ref allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
 * option to disable LCM network traffic (i.e., only allowing `memq://` URLs).
 *
 * For network issues on macOS, see
 * https://drake.mit.edu/troubleshooting.html#lcm-macos.
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

  void Publish(const std::string&, const void*, int,
               std::optional<double>) override;
  std::string get_lcm_url() const override;
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string&, HandlerFunction) override;
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeMultichannel(
      std::string_view, MultichannelHandlerFunction) override;
  std::shared_ptr<DrakeSubscriptionInterface> SubscribeAllChannels(
      MultichannelHandlerFunction) override;
  int HandleSubscriptions(int) override;

  /** Returns true if the LCM runtime library is enabled in this build of Drake.
   * When false, functions that require the runtime (e.g., HandleSubscriptions
   * and Publish) will throw an error. See `//tools/flags:with_lcm_runtime`.
   */
  static bool available();

 private:
  friend class DrakeLcmTester;

  void OnHandleSubscriptionsError(const std::string&) override;

  void* get_native_lcm_handle_for_unit_testing();

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lcm
}  // namespace drake
