#include "drake/systems/lcm/lcm_log_playback_system.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcmt_drake_signal.hpp"


namespace drake {
namespace systems {
namespace lcm {
namespace {

void SetInput(Context<double>* context, int dim) {
  lcmt_drake_signal msg;
  msg.dim = 1;
  msg.val.push_back(context->get_time() * 10);
  msg.coord.push_back("test");
  msg.timestamp = context->get_time() * 1e6;
  context->FixInputPort(0, AbstractValue::Make<lcmt_drake_signal>(msg));
}

class DummySys : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySys)

  DummySys() {
    DeclareAbstractInputPort();
    DeclarePeriodicPublish(0.01);
  }

  void DoPublish(
      const Context<double>& context,
      const std::vector<const systems::PublishEvent<double>*>& events) const override {

    for (const systems::PublishEvent<double>* event : events) {
      // Only pay attention to the periodic events, which is declared by us.
      // ignore ther per step events, which could be forced upon us.
      if (event->get_trigger_type() != Event<double>::TriggerType::kPeriodic)
        continue;

      const lcmt_drake_signal* msg =
        EvalInputValue<lcmt_drake_signal>(context, 0);

      if (msg->timestamp != 0) {
        std::cout << "t: " << context.get_time() << ", " << msg->timestamp << ", " << msg->val[0] << "\n";
      }
    }
  }
};

GTEST_TEST(haha, testhaha) {
  auto w_log = std::make_unique<drake::lcm::DrakeLcmLog>("test.log", true);
  const std::string channel_name("haha");
  const int kDim = 1;

  auto dut = LcmPublisherSystem::Make<lcmt_drake_signal>(channel_name, w_log.get());

  auto context = dut->CreateDefaultContext();
  std::vector<double> pub_time = {0.11, 0.22, 0.3};

  for (double time : pub_time) {
    context->set_time(time);
    SetInput(context.get(), kDim);
    dut->Publish(*context.get());
  }

  // Destroy the log to cause it to save to disk.
  w_log.reset();

  auto r_log = std::make_unique<drake::lcm::DrakeLcmLog>("test.log", false);

  DiagramBuilder<double> builder;
  // Need to add this additional block to actually play the log.
  builder.AddSystem<LcmLogPlaybackSystem>(r_log.get());
  auto sub = builder.AddSystem(LcmSubscriberSystem::Make<lcmt_drake_signal>(channel_name, r_log.get()));
  auto printer = builder.AddSystem<DummySys>();
  builder.Connect(sub->get_output_port(0), printer->get_input_port(0));
  auto diagram = builder.Build();

  Simulator<double> sim(*diagram);
  sim.StepTo(0.5);
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
