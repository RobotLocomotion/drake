#include "drake/examples/spring_mass/lcm_subscriber_system.h"

#include <iostream>

#if defined(_WIN32)
#include <Winsock2.h>
#else
#include <sys/select.h>
#endif

namespace drake {
namespace systems {
namespace lcm {
namespace internal {

// Waits for an LCM message to arrive.
bool WaitForLcm(::lcm::LCM& lcm, double timeout) {
  int lcmFd = lcm.getFileno();

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = timeout * 1e6;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(lcmFd, &fds);

  int status = select(lcmFd + 1, &fds, 0, 0, &tv);
  if (status == -1 && errno != EINTR) {
    // throw std::runtime_error("WaitForLcm: select() returned error: " +
    //   std::to_string(errno));
    std::cout << "WaitForLcm: select() returned error: " << errno << std::endl;
  } else if (status == -1 && errno == EINTR) {
    // throw std::runtime_error("WaitForLcm: select() interrupted.");
    std::cout << "WaitForLcm: select() interrupted." << std::endl;
  }
  return (status > 0 && FD_ISSET(lcmFd, &fds));
}

void LcmLoop::LoopWithSelect() {
  while (!stop_) {
    const double timeoutInSeconds = 0.3;
    bool lcmReady = WaitForLcm(lcm_, timeoutInSeconds);

    if (stop_) break;

    if (lcmReady) {
      if (lcm_.handle() != 0) {
        std::cout << "LoopWithSelect: lcm->handle() returned non-zero"
                  << std::endl;
        break;
      }
    }
  }
}

void LcmLoop::Stop() {
  stop_ = true;
}

}  // namespace internal


LcmSubscriberSystem::LcmSubscriberSystem(const std::string& channel,
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

LcmSubscriberSystem::~LcmSubscriberSystem() {
  lcm_loop_.Stop();
  lcm_thread_.join();
}

std::string LcmSubscriberSystem::get_name() const {
  return "LcmSubscriberSystem::" + channel_;
}

std::unique_ptr<Context<double>> LcmSubscriberSystem::CreateDefaultContext()
    const {
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

std::unique_ptr<SystemOutput<double>> LcmSubscriberSystem::AllocateOutput()
    const {
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

void LcmSubscriberSystem::EvalOutput(const Context<double>& context,
                SystemOutput<double>* output) const {
  BasicVector<double>* output_vector = dynamic_cast<BasicVector<double>*>(
      output->ports[0]->GetMutableVectorData());

  data_mutex.lock();
  output_vector->set_value(basic_vector_.get_value());
  data_mutex.unlock();
}

void LcmSubscriberSystem::handleMessage(const ::lcm::ReceiveBuffer* rbuf,
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

}  // namespace lcm
}  // namespace systems
}  // namesapce drake
