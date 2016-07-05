#include <chrono>
#include <thread>

#include "gtest/gtest.h"

#include "drake/examples/spring_mass/lcm_input_system.h"
#include "drake/examples/spring_mass/spring_mass_lcm_vector.h"
#include "lcmtypes/drake/lcmt_spring_mass_state_t.hpp"

namespace drake {
namespace examples {
namespace spring_mass {
namespace {

using drake::systems::LCMInputSystem;
using drake::systems::Context;
using drake::systems::SystemOutput;

const int64_t kTimestamp = 123456;
const double kPosition = 3.1415;
const double kVelocity = 2.718;
const int kPositionIndex = 0;
const int kVelocityIndex = 1;

class MessagePublisher {
 public:
  MessagePublisher(const std::string& channel_name, lcm::LCM& lcm) :
      channel_name_(channel_name), lcm_(lcm) {
    message_.timestamp = kTimestamp;
    message_.position = kPosition;
    message_.velocity = kVelocity;
  }

  void Start() {
    thread_.reset(new std::thread(&MessagePublisher::doPublish, this));
  }

  void Stop() {
    stop_ = true;
    thread_->join();
  }

 private:
  void doPublish() {
    while (!stop_) {
      lcm_.publish(channel_name_, &message_);
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  const std::string& channel_name_;

  lcm::LCM& lcm_;

  drake::lcmt_spring_mass_state_t message_;

  bool stop_{false};

  std::unique_ptr<std::thread> thread_;
};

// Tests the functionality of LCMInputSystem.
GTEST_TEST(LCMInputSystemTest, InstantiateTest) {
  // Instantiates LCM.
  lcm::LCM lcm;

  std::string channel_name = "test_channel_name";

  // Instantiates an LCMInputSystem that receives LCM messages of type
  // 'lcmt_spring_mass_state_t' and outputs System 2.0 Vector of type
  // `SpringMassLCMVector<double>`.
  //
  // It then verifies that the LCMInputSystem was successfully stored
  // in a unique_ptr. The unique_ptr variable is called "dut" to indicate it is
  // the "device under test".
  std::unique_ptr<LCMInputSystem<double, SpringMassLCMVector<double>,
    lcmt_spring_mass_state_t>> dut(
      new LCMInputSystem<double, SpringMassLCMVector<double>,
      lcmt_spring_mass_state_t>(channel_name, lcm));

  EXPECT_NE(dut.get(), nullptr);
  EXPECT_EQ(dut->get_name(), "LCMInputSystem::test_channel_name");

  // Instantiates a publisher of lcmt_spring_mass_state_t messages on the LCM
  // network.
  MessagePublisher publisher(channel_name, lcm);
  publisher.Start();

  std::unique_ptr<Context<double>> context = dut->CreateDefaultContext();
  std::unique_ptr<SystemOutput<double>> output = dut->AllocateOutput();

  // Whether the LCMInputSystem successfully received an LCM message and
  // outputted it as a SpringMassLCMVector.
  bool done = false;

  // This is used to prevent this unit test from running indefinitely when
  // the LCMInputSystem fails to output a SpringMassLCMVector.
  int count = 0;

  const int kMaxCount = 10;
  const int kDelayMS = 500;

  while (!done && count++ < kMaxCount) {
    dut->Output(*context.get(), output.get());

    const drake::systems::VectorInterface<double>* vector =
      output->ports[0]->get_vector_data();

    const SpringMassLCMVector<double>* lcm_vector =
        dynamic_cast<const SpringMassLCMVector<double>*>(vector);

    int64_t timestamp = lcm_vector->get_timestamp();
    Eigen::VectorBlock<const VectorX<double>> value = lcm_vector->get_value();

    std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMS));

    if (timestamp == kTimestamp && value[kPositionIndex] == kPosition &&
        value[kVelocityIndex] == kVelocity) {
      done = true;
    }
  }

  EXPECT_TRUE(done);

  publisher.Stop();
}

}  // namespace
}  // namespace spring_mass
}  // namespace examples
}  // namespace drake
