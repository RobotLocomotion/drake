#include "drake/automotive/simple_car.h"

#include <memory>

#include "drake/automotive/automotive_simulator.h"
#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "lcmtypes/drake/lcmt_viewer_load_robot.hpp"
#include "lcmtypes/drake/lcmt_viewer_draw.hpp"

namespace drake {
namespace automotive {
namespace {

// Publish hand-crafted BotVisualizer LCM messages so we can see this demo.
// TODO(jwnimmer-tri) Replace with actual use of BotVisualizer.
template <typename T>
class BotVisualizerHack : public systems::LeafSystem<T> {
 public:
  explicit BotVisualizerHack(::lcm::LCM* lcm)
      : lcm_(lcm) {
    this->set_name("BotVisualizerHack");
    this->DeclareInputPort(systems::kVectorValued,
                           EulerFloatingJointStateIndices::kNumCoordinates,
                           systems::kContinuousSampling);
  }

  void EvalOutput(const systems::Context<double>& context,
                  systems::SystemOutput<double>* output) const override {}

 protected:
  void DoPublish(const systems::Context<double>& context) const override {
    // Before any draw commands, we need to send the load_robot message.
    if (context.get_time() == 0.0) {
      PublishLoadRobot();
    }
    DRAKE_DEMAND(sent_load_robot_);

    const systems::VectorBase<double>* const input_base =
        context.get_vector_input(0);
    DRAKE_DEMAND(input_base != nullptr);
    const EulerFloatingJointState<double>* const input =
        dynamic_cast<const EulerFloatingJointState<double>*>(input_base);
    DRAKE_DEMAND(input != nullptr);

    const Eigen::Vector3f pos =
        (Eigen::Vector3d() << input->x(), input->y(), input->z())
        .finished().cast<float>();
    const Eigen::Vector3d rpy =
        (Eigen::Vector3d() << input->roll(), input->pitch(), input->yaw())
        .finished();
    const Eigen::Vector4f quat = math::rpy2quat(rpy).cast<float>();

    drake::lcmt_viewer_draw draw_msg;
    draw_msg.num_links = 1;
    std::vector<float> position{pos(0), pos(1), pos(2)};
    std::vector<float> quaternion{quat(0), quat(1), quat(2), quat(3)};
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
    gdata.float_data = std::vector<float>{2, 1, 1};
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
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  simulator->AddSimpleCar();
  simulator->AddSystem(std::make_unique<BotVisualizerHack<double>>(
      simulator->get_lcm()));
  auto joint_system_name = SimpleCarToEulerFloatingJoint<double>().get_name();
  simulator->get_builder()->Connect(
      simulator->GetBuilderSystemByName(joint_system_name),
      simulator->GetBuilderSystemByName("BotVisualizerHack"));
  simulator->Start();

  while (true) {
    simulator->StepBy(0.01);
  }

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, const char* argv[]) {
  return drake::automotive::do_main(argc, argv);
}
