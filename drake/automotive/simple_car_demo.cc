#include "drake/automotive/simple_car.h"

#include <memory>

#include "drake/automotive/gen/driving_command_translator.h"
#include "drake/automotive/gen/euler_floating_joint_state_translator.h"
#include "drake/automotive/gen/simple_car_state_translator.h"
#include "drake/automotive/simple_car_to_euler_floating_joint.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "lcmtypes/drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace cars {
namespace {

// Publish hand-crafted BotVisualizer LCM messages so we can see this demo.
// TODO(jwnimmer-tri) Replace with actual use of BotVisualizer.
template <typename T>
class BotVisualizerHack : public systems::LeafSystem<T> {
 public:
  explicit BotVisualizerHack(::lcm::LCM* lcm)
      : lcm_(lcm) {
    this->DeclareInputPort(systems::kVectorValued,
                           SimpleCarStateIndices::kNumCoordinates,
                           systems::kContinuousSampling);
  }

  std::string get_name() const override { return "bot_visualizer_hack"; }

  void EvalOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override {}

 protected:
  void DoPublish(const systems::Context<double>& context) const override {
    // Before any draw commands, we need to send the load_robot message.
    if (context.get_time() == 0.0) {
      PublishLoadRobot();
    }
    DRAKE_DEMAND(sent_load_robot_);

    auto input = context.get_vector_input(0)->get_value().cast<float>();
    DRAKE_DEMAND(input.size() == 6);

    drake::lcmt_viewer_draw draw_msg;
    draw_msg.num_links = 1;
    std::vector<float> position{input(0), input(1), input(2)};
    std::vector<float> quaternion{0, 0, 0, 1};
    draw_msg.link_name.push_back("");
    draw_msg.robot_num.push_back(0);
    draw_msg.position.push_back(position);
    draw_msg.quaternion.push_back(quaternion);

    lcm_->publish("DRAKE_VIEWER_DRAW", &draw_msg);
  }

 private:
  void PublishLoadRobot() const {
    drake::lcmt_viewer_geometry_data gdata;
    gdata.type = gdata.BOX;
    gdata.num_float_data = 3;
    gdata.float_data = std::vector<float>{2, 2, 1};
    gdata.position[0] = 0.0;
    gdata.position[1] = 0.0;
    gdata.position[2] = 0.0;
    gdata.quaternion[0] = 0.0;
    gdata.quaternion[1] = 0.0;
    gdata.quaternion[2] = 0.0;
    gdata.quaternion[3] = 1.0;
    gdata.color[0] = 0.5;
    gdata.color[1] = 0.5;
    gdata.color[2] = 0.5;
    gdata.color[3] = 1.0;

    drake::lcmt_viewer_link_data link;
    link.name = "";
    link.robot_num = 0;
    link.num_geom = 1;
    link.geom.push_back(gdata);

    drake::lcmt_viewer_load_robot vr;
    vr.num_links = 1;
    vr.link.push_back(link);

    lcm_->publish("DRAKE_VIEWER_LOAD_ROBOT", &vr);

    sent_load_robot_ = true;
  }

  ::lcm::LCM* const lcm_;
  // Using 'mutable' here is OK because it's only used for assertion checking.
  mutable bool sent_load_robot_{false};
};

int do_main(int argc, const char* argv[]) {
  auto lcm = std::make_unique<lcm::LCM>();

  const DrivingCommandTranslator driving_command_translator;
  auto command_subscriber = std::make_unique<systems::lcm::LcmSubscriberSystem>(
      "DRIVING_COMMAND", driving_command_translator, lcm.get());

  auto simple_car = std::make_unique<SimpleCar<double>>();
  auto coord_transform =
      std::make_unique<SimpleCarToEulerFloatingJoint<double>>();

  const SimpleCarStateTranslator simple_car_state_translator;
  auto simple_car_state_publisher =
      std::make_unique<systems::lcm::LcmPublisherSystem>(
          "SIMPLE_CAR_STATE", simple_car_state_translator, lcm.get());

  const EulerFloatingJointStateTranslator euler_floating_joint_state_translator;
  auto euler_floating_joint_state_publisher =
      std::make_unique<systems::lcm::LcmPublisherSystem>(
          "FLOATING_JOINT_STATE", euler_floating_joint_state_translator,
          lcm.get());

  auto bot_visualizer_hack = std::make_unique<BotVisualizerHack<double>>(
      lcm.get());

  auto builder = std::make_unique<systems::DiagramBuilder<double>>();
  builder->Connect(*command_subscriber, *simple_car);
  builder->Connect(*simple_car, *simple_car_state_publisher);
  builder->Connect(*simple_car, *coord_transform);
  builder->Connect(*coord_transform, *bot_visualizer_hack);
  builder->Connect(*coord_transform, *euler_floating_joint_state_publisher);

  auto diagram = builder->Build();

  auto lcm_receive_thread = std::make_unique<systems::lcm::LcmReceiveThread>(
      lcm.get());

  auto simulator = std::make_unique<systems::Simulator<double>>(*diagram);
  simulator->Initialize();
  while (true) {
    const double time = simulator->get_context().get_time();
    SPDLOG_TRACE(drake::log(), "Time is now {}", time);
    simulator->StepTo(time + 0.01);
  }

  return 0;
}

}  // namespace
}  // namespace cars
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::cars::do_main(argc, argv);
}
