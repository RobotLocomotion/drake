#pragma once

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

#include <mutex>
#include <stdexcept>
#include <thread>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/examples/spring_mass/lcm_basic_vector_translator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {
namespace lcm {

namespace internal {

/**
 * Implements the loop that receives LCM messages.
 */
class DRAKELCMSYSTEM2_EXPORT LcmLoop {
 public:
  /**
   * The constructor.
   *
   * @param[in] lcm The LCM subsystem to loop through.
   */
  explicit LcmLoop(::lcm::LCM& lcm) : stop_(false), lcm_(lcm) {}

  /**
   * Sits in a loop waiting for LCM messages to arrive and processes them as
   * they arrive.
   */
  void LoopWithSelect();

  /**
   * Stops the LCM loop's thread. This stops the receival of LCM messages.
   */
  void Stop();

 private:
  bool stop_{false};
  ::lcm::LCM& lcm_;
};

}  // end namespace internal

/**
 * Receives LCM messages and outputs them to a SystemInterface<double>'s port.
 */
class DRAKELCMSYSTEM2_EXPORT LcmSubscriberSystem :
    public SystemInterface<double> {
 public:
  /**
   * The constructor.
   *
   * @param[in] channel The LCM channel on which to subscribe.
   *
   * @param[in] translator The translator that converts between LCM message
   * objects and `drake::systems::BasicVector` objects.
   *
   * @param[in] lcm The LCM subsystem.
   */
  LcmSubscriberSystem(const std::string& channel,
                      const LcmBasicVectorTranslator& translator,
                      ::lcm::LCM& lcm);

  ~LcmSubscriberSystem() override;

  std::string get_name() const override;

  /**
   * The default context for this system is one that has zero input ports and
   * no state.
   */
  std::unique_ptr<Context<double>> CreateDefaultContext() const override;

  /**
   * The output consists of a single port containing a `BasicVector<double>`.
   */
  std::unique_ptr<SystemOutput<double>> AllocateOutput() const override;

  /**
   * Computes the output for the given context, possibly updating values
   * in the cache. Note that the context is ignored since it contains no
   * information.
   */
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override;

 private:
  // Translates the message contained within the recieve buffer by storing its
  // information in basic_vector_.
  void handleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel);

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // The translator that converts betweeen LCM messages and
  // drake::systems::BasicVector.
  const LcmBasicVectorTranslator& translator_;

  // The thread responsible for receiving LCM messages.
  std::thread lcm_thread_;

  // Implements the loop that receives LCM messages.
  internal::LcmLoop lcm_loop_;

  // A mutex for protecting data that's shared by the LCM receive thread and
  // the thread that calls LcmSubscriberSystem::Output().
  mutable std::mutex data_mutex;

  // Holds the information contained with the latest LCM message. This
  // information is copied into this system's output port when Output(...) is
  // called.
  BasicVector<double> basic_vector_;
};

}  // namespace lcm
}  // namespace systems
}  // namesapce drake
