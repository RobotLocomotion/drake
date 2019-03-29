#pragma once

#include <memory>
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
   * A destructor that forces the receive thread to be stopped.
   */
  ~DrakeLcm() override;

  /**
   * (Advanced.) Starts the receive thread.  Either HandleSubscriptions() xor
   * this method must be called for subscribers to receive any messages.
   *
   * @warning Almost no Drake uses of LCM should require a background thread.
   * Please use HandleSubscriptions() instead.
   *
   * @pre StartReceiveThread() was not called.
   */
  void StartReceiveThread();

  /**
   * (Advanced.) Stops the receive thread. This must be called prior to any
   * subscribers being destroyed.
   *
   * @warning Almost no Drake uses of LCM should require a background thread.
   * Please use HandleSubscriptions() instead.
   *
   * @pre StartReceiveThread() was called.
   */
  void StopReceiveThread();

  /**
   * (Advanced.) Indicates that the receiving thread is running.
   *
   * @warning Almost no Drake uses of LCM should require a background thread.
   * Please use HandleSubscriptions() instead.
   */
  bool IsReceiveThreadRunning() const;

  /**
   * (Advanced.) An accessor to the underlying LCM instance. The returned
   * pointer is guaranteed to be valid for the duration of this object's
   * lifetime.
   */
  ::lcm::LCM* get_lcm_instance();

  /**
   * Returns the LCM URL passed into the constructor; this can be empty.
   */
  std::string get_requested_lcm_url() const;

  /**
   * Returns the LCM URL.
   */
  std::string get_lcm_url() const;

  void Publish(const std::string&, const void*, int, optional<double>) override;
  std::shared_ptr<DrakeSubscriptionInterface> Subscribe(
      const std::string&, HandlerFunction) override;
  int HandleSubscriptions(int) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lcm
}  // namespace drake
