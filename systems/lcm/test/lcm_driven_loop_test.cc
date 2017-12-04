#include "drake/systems/lcm/lcm_driven_loop.h"

#include <gtest/gtest.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

const int kStart = 3;
const int kEnd = 10;

// Converts millisecond timestamp field to second.
class MilliSecTimeStampMessageToSeconds : public LcmMessageToTimeInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MilliSecTimeStampMessageToSeconds)
  MilliSecTimeStampMessageToSeconds() {}
  ~MilliSecTimeStampMessageToSeconds() {}

  double GetTimeInSeconds(const AbstractValue& abstract_value) const override {
    const lcmt_drake_signal& msg = abstract_value.GetValue<lcmt_drake_signal>();
    return static_cast<double>(msg.timestamp) / 1e3;
  }
};

// A dummy test system that outputs the input message's time stamp in seconds.
class DummySys : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySys);
  DummySys() {
    DeclareAbstractInputPort();
    DeclareVectorOutputPort(systems::BasicVector<double>(1),
                            &DummySys::CalcTimestamp);
  }

  void CalcTimestamp(const systems::Context<double>& context,
                     systems::BasicVector<double>* output) const {
    const lcmt_drake_signal* msg =
        EvalInputValue<lcmt_drake_signal>(context, 0);

    auto out_vector = output->get_mutable_value();
    out_vector(0) = static_cast<double>(msg->timestamp) / 1e3;
  }
};

// Dummy publish thread. Usleeps so that the dut thread has enough time to
// process.
void publish() {
  ::lcm::LCM lcm;
  lcmt_drake_signal msg;
  msg.dim = 0;
  msg.val.resize(msg.dim);
  msg.coord.resize(msg.dim);

  const int kSleepMicroSec = 100000;

  // This thread will first sleep for long enough to ensure `sys` will always
  // receive the first message for test consistency. In practice, this is
  // less important if `sys` does not need to receive the very first published
  // message.
  usleep(kSleepMicroSec);

  for (int i = kStart; i <= kEnd; i++) {
    msg.timestamp = 1000 * i;
    lcm.publish("test", &msg);
    usleep(kSleepMicroSec);
  }
}

// The receiving Diagram in this test consists of a LcmSubscriberSystem,
// a DummySys and a SignalLogger. The intended behavior is that every time
// a new Lcm message arrives, its timestamp will be logged by the SignalLogger.
GTEST_TEST(LcmDrivenLoopTest, TestLoop) {
  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Makes the test system.
  auto sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>("test", &lcm));
  sub->set_name("subscriber");
  auto dummy = builder.AddSystem<DummySys>();
  dummy->set_name("dummy");
  auto logger = builder.AddSystem<SignalLogger<double>>(1);
  logger->set_name("logger");
  builder.Connect(*sub, *dummy);
  builder.Connect(*dummy, *logger);
  auto sys = builder.Build();

  // Makes the lcm driven loop.
  lcm::LcmDrivenLoop dut(*sys, *sub, nullptr, &lcm,
      std::make_unique<MilliSecTimeStampMessageToSeconds>());
  // This ensures that dut calls sys->Publish() every time it handles a
  // message, which triggers the logger to save its input (message time stamp)
  // to the log.
  dut.set_publish_on_every_received_message(true);

  // Starts the publishing thread.
  std::thread pub_thread(&publish);

  // Waits for the first message.
  const AbstractValue& first_msg = dut.WaitForMessage();
  double msg_time =
      dut.get_message_to_time_converter().GetTimeInSeconds(first_msg);
  dut.get_mutable_context().set_time(msg_time);

  // Starts the loop.
  dut.RunToSecondsAssumingInitialized(static_cast<double>(kEnd));

  // Reaps the publishing thread.
  pub_thread.join();

  // Makes the expected output: should be [kStart + 1, ... kEnd]. kStart is
  // used to initialize the loop's initial context, so it's not used for
  // computation, thus is not reflected in the output.
  VectorX<double> expected(kEnd - kStart - 1);
  int ctr = 0;
  for (int i = kStart + 1; i < kEnd; i++) {
    expected(ctr++) = i;
  }

  // Compare logger's data, which are the message time stamps. They should
  // match the values set in the publishing thread.
  EXPECT_TRUE(drake::CompareMatrices(expected.transpose(), logger->data(),
                                     1e-12,
                                     drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
