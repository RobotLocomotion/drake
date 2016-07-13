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
class LcmSubscriberSystem : public SystemInterface<double> {
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
                      ::lcm::LCM& lcm)
      : channel_(channel),
        translator_(translator),
        lcm_loop_(lcm),
        basic_vector_(translator.get_basic_vector_size()) {
    // Initializes the communication layer.
    ::lcm::Subscription* sub =
        lcm.subscribe(channel_, &LcmSubscriberSystem::handleMessage, this);
    sub->setQueueCapacity(1);

    // Spawns a thread that accepts incomming LCM messages.
    lcm_thread_ = std::thread(&internal::LcmLoop::LoopWithSelect, &lcm_loop_);
  }

  ~LcmSubscriberSystem() override {
    lcm_loop_.Stop();
    lcm_thread_.join();
  }

  std::string get_name() const override {
    return "LcmSubscriberSystem::" + channel_;
  }

  /**
   * The default context for this system is one that has zero input ports and
   * no state.
   */
  std::unique_ptr<Context<double>> CreateDefaultContext() const override {
    // Creates a new context for this system and sets the number of input ports
    // to be zero.
    std::unique_ptr<Context<double>> context(new Context<double>());
    context->SetNumInputPorts(0);

    // Creates a BasicStateVector of size zero for this system.
    std::unique_ptr<BasicStateVector<double>> state(
        new BasicStateVector<double>(0));

    context->get_mutable_state()->continuous_state.reset(
        new ContinuousState<double>(std::move(state), 0 /* size of q */,
                                    0 /* size of v */, 0 /* size of z */));
    return context;
  }

  /**
   * The output consists of a single port containing a `BasicVector<double>`.
   */
  std::unique_ptr<SystemOutput<double>> AllocateOutput() const override {
    std::unique_ptr<SystemOutput<double>> output(new SystemOutput<double>);
    {
      std::unique_ptr<BasicVector<double>> data(
          new BasicVector<double>(translator_.get_basic_vector_size()));
      std::unique_ptr<OutputPort<double>> port(
          new OutputPort<double>(std::move(data)));
      output->ports.push_back(std::move(port));
    }
    return output;
  }

  // Computes the output for the given context, possibly updating values
  // in the cache. Note that the context is ignored since it contains no
  // information.
  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    BasicVector<double>* output_vector = dynamic_cast<BasicVector<double>*>(
        output->ports[0]->GetMutableVectorData());

    data_mutex.lock();
    output_vector->set_value(basic_vector_.get_value());
    data_mutex.unlock();
  }

 private:
  // Translates the message contained within the recieve buffer by storing its
  // information in basic_vector_.
  void handleMessage(const ::lcm::ReceiveBuffer* rbuf,
                     const std::string& channel) {
    if (channel == channel_) {
      data_mutex.lock();
      translator_.TranslateLcmToBasicVector(rbuf, &basic_vector_);
      data_mutex.unlock();
    } else {
      std::cerr << "LcmSubscriberSystem: handleMessage: WARNING: Received a "
                << "message for channel \"" << channel
                << "\" instead of channel \"" << channel_ << "\". Ignoring it."
                << std::endl;
    }
  }

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
