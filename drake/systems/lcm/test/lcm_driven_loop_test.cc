#include "drake/systems/lcm/lcm_driven_loop.h"
#include <gtest/gtest.h>

#include "drake/systems/framework/diagram_builder.h"

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace systems {
namespace lcm {
namespace {

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

class DummySys : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySys);
  DummySys() {
    DeclareAbstractInputPort();
    DeclareOutputPort(systems::kVectorValued, 1);
  }

  void DoCalcOutput(
      const systems::Context<double>& context,
      systems::SystemOutput<double>* output) const override {
    const lcmt_drake_signal* msg = EvalInputValue<lcmt_drake_signal>(
      context, 0);

    auto out_vector = GetMutableOutputVector(output, 0);
    out_vector(0) = static_cast<double>(msg->timestamp) / 1e3;
  }
};


void test() {
  drake::lcm::DrakeMockLcm lcm;
  DiagramBuilder<double> builder;
  auto sub = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>("test", &lcm));
  auto dummy = builder.AddSystem<DummySys>();

  builder.Connect(*sub, *dummy);

  auto sys = builder.Build();

  auto msg_to_time = std::make_unique<MilliSecTimeStampMessageToSeconds>();

  lcm::LcmDrivenLoop dut(&lcm, *sys, nullptr, sub, std::move(msg_to_time));

  dut.RunWithDefaultInitialization();
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake

int main() {
  drake::systems::lcm::test();

  return 0;
}
