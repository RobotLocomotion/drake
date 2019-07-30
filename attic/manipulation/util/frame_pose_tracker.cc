#include "drake/manipulation/util/frame_pose_tracker.h"

#include <algorithm>
#include <cstddef>
#include <utility>

#include "drake/multibody/rigid_body_plant/kinematics_results.h"

namespace drake {
namespace manipulation {
namespace util {

using drake::systems::KinematicsResults;
using drake::math::RigidTransformd;

FramePoseTracker::FramePoseTracker(
    const RigidBodyTree<double>& tree,
    std::vector<std::unique_ptr<RigidBodyFrame<double>>>* frames)
    : tree_(&tree),
      source_id_(geometry::SourceId::get_new_id()) {

  if (frames == nullptr) {
    throw std::runtime_error("FramePoseTracker::FramePoseTracker: ERROR: "
                                 "frames vector is a nullptr.");
  } else if (frames->size() == 0) {
    throw std::runtime_error("FramePoseTracker::FramePoseTracker: ERROR: "
                                 "frame vector is size 0.");
  }

  // Confirm all rigid bodies and frame names are valid.
  for (auto& frame : *frames) {
    RigidBody<double>* body = frame->get_mutable_rigid_body();
    std::string frame_name = frame->get_name();
    if (body == nullptr) {
      throw std::runtime_error(
          "FramePoseTracker::FramePoseTracker: ERROR: found nullptr to "
              "RigidBody in RigidBodyFrame object.");
    } else if (frame_name_to_frame_map_.find(frame_name) !=
        frame_name_to_frame_map_.end()) {
      throw std::runtime_error(
          "FramePoseTracker::FramePoseTracker: ERROR: found duplicate frame "
              "name. Frame names must be unique.");
    } else {
      frame_name_to_frame_map_[frame_name] = std::move(frame);
    }
  }
  FramePoseTracker::Init();
  frames->clear();
}

FramePoseTracker::FramePoseTracker(
    const RigidBodyTree<double>& tree,
    const std::map<std::string, std::pair<std::string, int>> frames_info,
    std::vector<Eigen::Isometry3d> frame_poses)
    : tree_(&tree),
      source_id_(geometry::SourceId::get_new_id()) {

  // If the frame vector is empty, set all frame poses to the identity pose.
  if (frame_poses.empty()) {
    frame_poses = std::vector<Eigen::Isometry3d>(frames_info.size(),
                                                 Eigen::Isometry3d::Identity());
  }

  DRAKE_DEMAND(frames_info.size() == frame_poses.size());

  // Create and store the related frames. Note that having the frame name as the
  // map key for @p frame_info ensures that frame names coming in are unique.
  auto frame_info = frames_info.begin();
  auto frame_pose = frame_poses.begin();
  while (frame_info != frames_info.end() && frame_pose != frame_poses.end()) {
    std::string frame_name = frame_info->first;
    RigidBody<double>* parent_body = nullptr;
    Eigen::Isometry3d X_BF;
    auto parent_frame = tree_->findFrame(
        frame_info->second.first, frame_info->second.second);
    parent_body = parent_frame->get_mutable_rigid_body();
    X_BF = parent_frame->get_transform_to_body();
    frame_name_to_frame_map_[frame_name] =
        std::make_unique<RigidBodyFrame<double>>(
          frame_name, parent_body, X_BF * *frame_pose);
    ++frame_info;
    ++frame_pose;
  }
  FramePoseTracker::Init();
}

void FramePoseTracker::Init() {
  // Abstract input port of type KinematicsResults
  kinematics_input_port_index_ = this->DeclareAbstractInputPort(
      Value<KinematicsResults<double>>(tree_)).get_index();

  // Make our frame ids and declare the output port.
  std::vector<geometry::FrameId> frame_ids;

  for (auto it = frame_name_to_frame_map_.begin();
       it != frame_name_to_frame_map_.end(); ++it) {
    geometry::FrameId id = geometry::FrameId::get_new_id();
    frame_name_to_id_map_[it->first] = id;
    frame_ids.push_back(id);
  }

  pose_vector_output_port_index_ =
      this->DeclareAbstractOutputPort(
          &FramePoseTracker::OutputStatus).get_index();
}

void FramePoseTracker::OutputStatus(
    const systems::Context<double>& context,
    geometry::FramePoseVector<double>* output) const {

  // Extract the input port for the KinematicsResults object, get the
  // transformation of the bodies we care about, and fill in the
  // FramePoseVector.
  const KinematicsResults<double>* kinematic_results =
      this->EvalInputValue<KinematicsResults<double>>(
          context, kinematics_input_port_index_);

  output->clear();

  for (auto it = frame_name_to_frame_map_.begin();
       it != frame_name_to_frame_map_.end(); ++it) {
    output->set_value(
        frame_name_to_id_map_.at(it->first),
        RigidTransformd(tree_->CalcFramePoseInWorldFrame(
            kinematic_results->get_cache(), *(it->second.get()))));
  }
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
