#include <gtest/gtest.h>

#include "drake/lcm/drake_lcm_log.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_log_playback_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

const int kDim = 1;

void SetInput(double time, const std::string& name, double val,
              const InputPort<double>& input_port, Context<double>* context) {
  lcmt_drake_signal msg;
  msg.dim = kDim;
  msg.val.resize(kDim, val);
  msg.coord.resize(kDim, name);
  msg.timestamp = time * 1e6;
  context->SetTime(time);
  input_port.FixValue(context, msg);
}

class DummySys : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySys)

  DummySys() {
    DeclareAbstractInputPort("lcmt_drake_signal", Value<lcmt_drake_signal>());
    DeclarePeriodicPublishEvent(1.0 / publish_freq_, 0.0 /* no time offset */,
        &DummySys::SaveMessage);
  }

  const std::vector<lcmt_drake_signal>& get_received_msgs() const {
    return received_msgs_;
  }

  // This returns the context's time when a new message has arrived.
  // Note that this time in general is not the same as the timestamp
  // in the message itself.
  const std::vector<double> get_received_times() const {
    return std::vector<double>(received_time_.begin() + 1,
                               received_time_.end());
  }

 private:
  EventStatus SaveMessage(const Context<double>& context) const {
    const lcmt_drake_signal& msg =
        this->get_input_port(0).Eval<lcmt_drake_signal>(context);

    bool is_new_msg = false;
    if (received_msgs_.empty() && msg.timestamp != 0) is_new_msg = true;
    if (!received_msgs_.empty() &&
        (msg.timestamp != received_msgs_.back().timestamp)) {
      is_new_msg = true;
    }

    if (is_new_msg) {
      received_msgs_.push_back(msg);

      // The diagram that this system is embedded in works the following way:
      // The LCM Subscriber system receives a message and then requests an
      // unrestricted update. The unrestricted update causes the input to this
      // system (DummySys) to change on its next publish operation, which
      // happens exactly one "tick" after the unrestricted update (Simulator
      // docs indicate that the sequence of simulation actions is unrestricted
      // event, discrete update event, continuous state update [integration],
      // publish, meaning that time is advanced between the unrestricted update
      // and the publish). Therefore, publish times are expected to be one
      // "tick" behind.
      received_time_.push_back(context.get_time() - 1.0 / publish_freq_);
    }
    return EventStatus::Succeeded();
  }

  const double publish_freq_{100.0};  // In Hz.
  mutable std::vector<lcmt_drake_signal> received_msgs_;
  mutable std::vector<double> received_time_{0};
};

// Generates the log file and populates it using LCM outputs.
void GenerateLog() {
  drake::lcm::DrakeLcmLog w_log("test.log", true);
  auto pub0 = LcmPublisherSystem::Make<lcmt_drake_signal>("Ch0", &w_log);
  auto context0 = pub0->CreateDefaultContext();

  auto pub1 = LcmPublisherSystem::Make<lcmt_drake_signal>("Ch1", &w_log);
  auto context1 = pub1->CreateDefaultContext();

  SetInput(0.1, "Ch0", 1, pub0->get_input_port(), context0.get());
  pub0->Publish(*context0);

  SetInput(0.22, "Ch1", 2, pub1->get_input_port(), context1.get());
  pub1->Publish(*context1);

  // Testing multiple messages sent to the same channel at the same time.
  // Only the last one should be visible from the Subscriber's point of view.
  SetInput(0.3, "Ch0", 3, pub0->get_input_port(), context0.get());
  pub0->Publish(*context0);
  SetInput(0.3, "Ch0", 4, pub0->get_input_port(), context0.get());
  pub0->Publish(*context0);
  SetInput(0.3, "Ch0", 5, pub0->get_input_port(), context0.get());
  pub0->Publish(*context0);

  // Testing sending a message to a different channel at the same time.
  SetInput(0.3, "Ch1", 6, pub1->get_input_port(), context1.get());
  pub1->Publish(*context1);

  SetInput(0.4, "Ch1", 7, pub1->get_input_port(), context1.get());
  pub1->Publish(*context1);
}

void CheckLog(const std::vector<double>& expected_times,
              const std::vector<double>& expected_vals,
              const std::string& expected_name,
              const std::vector<lcmt_drake_signal>& msgs,
              const std::vector<double>& times) {
  EXPECT_EQ(msgs.size(), expected_times.size());
  EXPECT_EQ(times.size(), expected_times.size());
  DRAKE_DEMAND(expected_vals.size() == expected_times.size());
  for (size_t i = 0; i < expected_times.size(); i++) {
    const lcmt_drake_signal& msg = msgs[i];
    EXPECT_EQ(msg.dim, kDim);
    EXPECT_EQ(msg.val.size(), kDim);
    EXPECT_EQ(msg.val[0], expected_vals[i]);
    EXPECT_EQ(msg.coord.size(), kDim);
    EXPECT_EQ(msg.coord[0], expected_name);

    // msg.timestamp is generated with the shifted time.
    EXPECT_NEAR(msg.timestamp, expected_times[i] * 1e6, 1e-12);
    EXPECT_NEAR(times[i], expected_times[i], 1e-12);
  }
}

void CheckLog() {
  // Since time shifting is taken care of in log generation, we don't need to
  // worry about it here.
  drake::lcm::DrakeLcmLog r_log("test.log", false);

  DiagramBuilder<double> builder;
  builder.AddSystem<LcmLogPlaybackSystem>(&r_log);
  auto sub0 = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>("Ch0", &r_log));
  sub0->set_name("sub0");
  auto sub1 = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>("Ch0", &r_log));
  sub1->set_name("sub1");
  auto sub2 = builder.AddSystem(
      LcmSubscriberSystem::Make<lcmt_drake_signal>("Ch1", &r_log));
  sub2->set_name("sub2");
  auto printer0 = builder.AddSystem<DummySys>();
  auto printer1 = builder.AddSystem<DummySys>();
  auto printer2 = builder.AddSystem<DummySys>();
  builder.Connect(sub0->get_output_port(), printer0->get_input_port(0));
  builder.Connect(sub1->get_output_port(), printer1->get_input_port(0));
  builder.Connect(sub2->get_output_port(), printer2->get_input_port(0));

  auto diagram = builder.Build();

  Simulator<double> sim(*diagram);
  sim.AdvanceTo(0.5);

  // printer0 should have msg at t = [0.1, 0.3], with val = [1, 5].
  CheckLog({0.1, 0.3}, {1, 5}, "Ch0", printer0->get_received_msgs(),
           printer0->get_received_times());

  // printer1 should have the exact same msg as print0.
  CheckLog({0.1, 0.3}, {1, 5}, "Ch0", printer1->get_received_msgs(),
           printer1->get_received_times());

  // printer2 should have msg at t = [0.1, 0.3], with val = [2, 6, 7].
  CheckLog({0.22, 0.3, 0.4}, {2, 6, 7}, "Ch1", printer2->get_received_msgs(),
           printer2->get_received_times());
}

// Generates a log named test.log that contains two channels: "Ch0" and "Ch1".
// For Ch0, publish calls happen at t = [0.1, 0.3, 0.3, 0.3]. Although
// publishing to the same channel at the exact same moment is unlikely and
// indicates a degenerate use case, it is still possible.
// For Ch1, publish calls happen at t = [0.22, 0.3, 0.4].
//
// When playing back the log, the diagram under test consists of 3 subscribers:
// sub0 and sub1 are listening to Ch0, and sub2 is listening to Ch1. For each
// subscriber, the content of each distinct message and its receive time are
// recorded and compared against expected values.
GTEST_TEST(TestLcmLogPlayback, TestLcmLogPlayback) {
  GenerateLog();
  CheckLog();
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
