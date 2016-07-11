#pragma once

// TODO(liang.fok) Move this class into a directory that is dedicated to
// LCM-based systems after it is mature and proven useful.

#include <mutex>
#include <stdexcept>
#include <thread>

#include <lcm/lcm-cpp.hpp>

#include "drake/drakeLCMSystem2_export.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/basic_state_vector.h"
#include "drake/systems/framework/system_interface.h"
#include "drake/systems/framework/vector_interface.h"



namespace drake {
namespace systems {

namespace internal {

class DRAKELCMSYSTEM2_EXPORT LCMLoop {
 public:
  explicit LCMLoop(lcm::LCM& lcm) : stop_(false), lcm_(lcm) {}

  void LoopWithSelect();

  // Stops the LCM loop's thread. This stops the receival of LCM messages.
  void Stop();

 private:
  bool stop_{false};
  lcm::LCM& lcm_;
};

}  // end namespace internal

template <typename T, typename LCMVectorInterfaceType, typename LCMMessageType>
class LCMInputSystem : public SystemInterface<T> {
 public:
  LCMInputSystem(const std::string& channel, lcm::LCM& lcm) :
      channel_(channel),
      lcm_loop_(lcm) {
    // Initializes the communication layer.
    lcm::Subscription* sub =
      lcm.subscribe(channel_,
                    &LCMInputSystem<T, LCMVectorInterfaceType,
                                    LCMMessageType>::handleMessage, this);
    sub->setQueueCapacity(1);

    // Spawns a thread that accepts incomming LCM messages.
    lcm_thread_ = std::thread(&internal::LCMLoop::LoopWithSelect,
      &lcm_loop_);
  }

  ~LCMInputSystem() override {
    lcm_loop_.Stop();
    lcm_thread_.join();
  }

  std::string get_name() const override {
    // TODO(liang.fok) Can the name include the templated types?
    return "LCMInputSystem::" + channel_;
  }

  std::unique_ptr<Context<T>> CreateDefaultContext() const override {
    // Creates a new context for this system and sets the number of input ports
    // to be zero.
    std::unique_ptr<Context<T>> context(new Context<T>());
    context->SetNumInputPorts(0);

    // Creates a BasicStateVector of size zero for this system.
    std::unique_ptr<BasicStateVector<T>> state(new BasicStateVector<T>(0));
    context->get_mutable_state()->continuous_state.reset(
        new ContinuousState<double>(std::move(state), 0 /* size of q */,
                                    0 /* size of v */, 0 /* size of z */));
    return context;
  }

  std::unique_ptr<SystemOutput<T>> AllocateOutput() const override {
    std::unique_ptr<SystemOutput<T>> output(new SystemOutput<T>);
    {
      std::unique_ptr<LCMVectorInterfaceType>
        data(new LCMVectorInterfaceType());
      std::unique_ptr<OutputPort<double>> port(
          new OutputPort<T>(std::move(data)));
      output->ports.push_back(std::move(port));
    }
    return output;
  }

  // Computes the output for the given context, possibly updating values
  // in the cache. Note that the context is ignored since it contains no
  // information.
  void Output(const Context<T>& context, SystemOutput<T>* output) const
      override {
    LCMVectorInterfaceType* output_vector =
      dynamic_cast<LCMVectorInterfaceType*>(
        output->ports[0]->GetMutableVectorData());

    data_mutex.lock();
    output_vector->Encode(lcm_message_);
    data_mutex.unlock();
  }

 private:
  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                     const LCMMessageType* msg) {
    data_mutex.lock();
    lcm_message_ = *msg;
    data_mutex.unlock();
  }

  // The channel on which to receive LCM messages.
  const std::string channel_;

  // The thread responsible for receiving LCM messages.
  std::thread lcm_thread_;

  // Implements the loop that receives LCM messages.
  internal::LCMLoop lcm_loop_;

  // A mutex for protecting data that's shared by the LCM receive thread and
  // the thread that calls LCMInputSystem::Output().
  mutable std::mutex data_mutex;

  // A buffer for the most recently received LCM message.
  LCMMessageType lcm_message_;
};

}  // namespace systems
}  // namesapce drake
