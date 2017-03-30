#include "drake/systems/lcm/lcm_driven_loop.h"
#include <gtest/gtest.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/signal_logger.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace systems {
namespace lcm {
namespace {

const int kStart = 0;
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

// A dummy test system that outputs the input message's timestamp in seconds.
class DummySys : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySys);
  DummySys() {
    DeclareAbstractInputPort();
    DeclareOutputPort(systems::kVectorValued, 1);
  }

  void DoCalcOutput(const systems::Context<double>& context,
                    systems::SystemOutput<double>* output) const override {
    const lcmt_drake_signal* msg =
        EvalInputValue<lcmt_drake_signal>(context, 0);

    auto out_vector = GetMutableOutputVector(output, 0);
    out_vector(0) = static_cast<double>(msg->timestamp) / 1e3;
  }
};

// Dummy publish thread. Usleeps so that the dut thread has enough time to
// process.
void publish() {
  ::lcm::LCM lcm;
  lcmt_drake_signal msg;
  usleep(5000);

  for (int i = kStart; i <= kEnd; i++) {
    msg.timestamp = 1000 * i;
    lcm.publish("test", &msg);
    usleep(1000);
  }
}

GTEST_TEST(LcmDrivenLoopTest, TestLoop) {
  drake::lcm::DrakeLcm lcm;
  DiagramBuilder<double> builder;

  // Makes the test system.
  auto sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>("test", &lcm));
  auto dummy = builder.AddSystem<DummySys>();
  auto logger = builder.AddSystem<SignalLogger<double>>(1);
  builder.Connect(*sub, *dummy);
  builder.Connect(*dummy, *logger);
  auto sys = builder.Build();

  // Makes the converter from timestamp of milliseconds to seconds.
  auto msg_to_time = std::make_unique<MilliSecTimeStampMessageToSeconds>();

  // Makes the lcm driven loop.
  lcm::LcmDrivenLoop dut(*sys, nullptr, &lcm, sub, std::move(msg_to_time));

  // Starts the publishing thread, this will first sleep for long enough so that
  // dut can go sleep first.
  std::thread pub_thread(&publish);

  // Starts the loop.
  dut.RunWithDefaultInitializationTo(static_cast<double>(kEnd));

  // Reaps the publishing thread.
  pub_thread.join();

  // Makes the expected output.
  VectorX<double> expected(kEnd - kStart);
  for (int i = kStart; i <= kEnd; i++) {
    expected(i) = i;
  }
  EXPECT_TRUE(drake::CompareMatrices(expected.transpose(), logger->data(),
                                     1e-12,
                                     drake::MatrixCompareType::absolute));
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
