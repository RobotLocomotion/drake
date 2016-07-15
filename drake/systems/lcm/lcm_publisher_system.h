#pragma once

#include <mutex>
#include <stdexcept>
#include <thread>
#include <iostream>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/framework/vector_interface.h"
#include "drake/systems/lcm/lcm_basic_vector_translator.h"
#include "drake/systems/lcm/lcm_receive_thread.h"

namespace drake {
namespace systems {
namespace lcm {

/**
 * Receives `BasicVector<double>` values from a `SystemInterface<double>`'s
 * output port.
 */
class DRAKELCMSYSTEM2_EXPORT LcmPublisherSystem :
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
   * @param[in] lcm A pointer to the LCM subsystem.
   */
  LcmPublisherSystem(const std::string& channel,
                      const LcmBasicVectorTranslator& translator,
                      ::lcm::LCM* lcm);

  ~LcmPublisherSystem() override;

  std::string get_name() const override;

  /**
   * The default context for this system is one that has one input port and
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
  const int kNumInputPorts = 1;
  const int kPortIndex = 0;

  // Translates the message contained within the receive buffer by storing its
  // information in basic_vector_.
  void handleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel);

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // The translator that converts between LCM messages and
  // drake::systems::BasicVector.
  const LcmBasicVectorTranslator& translator_;

  // Implements the loop that receives LCM messages.
  LcmReceiveThread* lcm_receive_thread_;

  // A mutex for protecting data that's shared by the LCM receive thread and
  // the thread that calls LcmPublisherSystem::Output().
  mutable std::mutex data_mutex;

  // Holds the information contained with the latest LCM message. This
  // information is copied into this system's output port when Output(...) is
  // called.
  BasicVector<double> basic_vector_;

  // Points to the data that holds an encoded LCM message. This is used when
  // publishing LCM messages.
  uint8_t *buffer_;
};

}  // namespace lcm
}  // namespace systems
}  // namespace drake
