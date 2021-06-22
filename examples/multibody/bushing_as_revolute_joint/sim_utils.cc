#include "drake/examples/multibody/bushing_as_revolute_joint/sim_utils.h"

#include "drake/geometry/scene_graph.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bushing_as_revolute_joint {

using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;
using Eigen::Vector3d;

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<RigidTransformd>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm) {
  DRAKE_DEMAND(poses.size() == names.size());
  drake::lcmt_viewer_draw frame_msg{};
  frame_msg.timestamp = 0;
  int32_t vsize = poses.size();
  frame_msg.num_links = vsize;
  frame_msg.link_name.resize(vsize);
  frame_msg.robot_num.resize(vsize, 0);

  for (size_t i = 0; i < poses.size(); i++) {
    drake::math::RigidTransform<float> pose = poses[i].cast<float>();
    // Create a frame publisher
    Eigen::Vector3f goal_pos = pose.translation();
    auto goal_quat = Eigen::Quaternion<float>(pose.rotation().matrix());
    frame_msg.link_name[i] = names[i];
    frame_msg.position.push_back({goal_pos(0), goal_pos(1), goal_pos(2)});
    frame_msg.quaternion.push_back(
        {goal_quat.w(), goal_quat.x(), goal_quat.y(), goal_quat.z()});
  }

  const int num_bytes = frame_msg.getEncodedSize();
  const auto size_bytes = static_cast<size_t>(num_bytes);
  std::vector<uint8_t> bytes(size_bytes);
  frame_msg.encode(bytes.data(), 0, num_bytes);
  dlcm->Publish("DRAKE_DRAW_FRAMES_" + channel_name, bytes.data(), num_bytes,
                {});
}

/// Publishes pre-defined body frames once.
void PublishBodyFrames(const drake::systems::Context<double>& plant_context,
                       const MultibodyPlant<double>& plant,
                       drake::lcm::DrakeLcm* lcm) {
  std::vector<std::string> body_names;
  std::vector<RigidTransformd> poses;

  // list the body names that we want to visualize.
  body_names.emplace_back("frameA");
  body_names.emplace_back("frameC");
  body_names.emplace_back("base");
  body_names.emplace_back("ySlider_body");
  body_names.emplace_back("rod_body");
  body_names.emplace_back("zSlider_body");

  for (auto& body_name : body_names) {
    const auto& frame = plant.GetFrameByName(
        body_name, plant.GetModelInstanceByName("gripper"));
    const drake::math::RigidTransformd& X_WB =
        plant.CalcRelativeTransform(plant_context, plant.world_frame(), frame);

    poses.push_back(X_WB);
  }

  PublishFramesToLcm("", poses, body_names, lcm);
}

/// A system that publishes frames at a specified period.
FrameViz::FrameViz(const MultibodyPlant<double>& plant,
                   drake::lcm::DrakeLcm* lcm, double period, bool frames_input)
    : plant_(plant), lcm_(lcm), frames_input_(frames_input) {
  this->DeclareVectorInputPort(
      "x", drake::systems::BasicVector<double>(plant.num_multibody_states()));
  // if true, then we create an additional input port which takes arbitrary
  // frames to visualize (a vector of type RigidTransform).
  if (frames_input_) {
    this->DeclareAbstractInputPort(
        "poses", drake::Value<std::vector<RigidTransformd>>());
  }
  this->DeclarePeriodicPublishEvent(period, 0., &FrameViz::PublishFramePose);
  plant_context_ = plant.CreateDefaultContext();
}

drake::systems::EventStatus FrameViz::PublishFramePose(
    const drake::systems::Context<double>& context) const {
  if (frames_input_) {
    const auto frames_vec =
        this->GetInputPort("poses").Eval<std::vector<RigidTransformd>>(context);
    std::vector<std::string> frames_names;
    for (auto iter = frames_vec.begin(); iter != frames_vec.end(); iter++) {
      // TODO(rcory) Support user configurable frame names.
      std::string name =
          "sim_frame_" + std::to_string(iter - frames_vec.begin());
      frames_names.push_back(name);
    }
    PublishFramesToLcm("SIM_FRAMES", frames_vec, frames_names, lcm_);
  } else {
    auto state = this->EvalVectorInput(context, 0)->get_value();
    plant_.SetPositionsAndVelocities(plant_context_.get(), state);
    PublishBodyFrames(*plant_context_, plant_, lcm_);
  }
  return drake::systems::EventStatus::Succeeded();
}

}  // bushing_as_revolute_joint
}  // namespace multibody
}  // namespace examples
}  // namespace drake