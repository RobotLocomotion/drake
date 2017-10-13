#include "drake/manipulation/util/frame_pose_extractor.h"

#include <algorithm>
#include <utility>
#include <cstddef>

#include "drake/multibody/rigid_body_plant/kinematics_results.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/multibody/multibody_tree/math/spatial_velocity.h"
#include "drake/systems/rendering/frame_velocity.h"


namespace drake {
namespace systems {
namespace sensors {

using std::string;
using drake::systems::KinematicsResults;
using systems::rendering::PoseBundle;
using systems::rendering::FrameVelocity;
using multibody::SpatialVelocity;

FramePoseExtractor::FramePoseExtractor(
    const RigidBodyTree<double>& tree,
    const std::vector<RigidBodyFrame<double>*>& frames) : tree_(&tree){

  // Confirm all rigid bodies and frame names are valid.
  for (auto it = frames.begin(); it != frames.end(); ++it) {
    RigidBody<double>* body = (*it)->get_mutable_rigid_body();
    std::string frame_name = (*it)->get_name();
    if (body == nullptr) {
      throw std::runtime_error(
          "FramePoseExtractor::Init: ERROR: found nullptr to "
              "RigidBody in RigidBodyFrame object.");
    } else if (frame_name_to_frame_map_.find(frame_name) !=
        frame_name_to_frame_map_.end()) {
      throw std::runtime_error(
          "FramePoseExtractor::Init: ERROR: found duplicate frame name. "
              "Frame names must be unique.");
    } else {
      frame_name_to_frame_map_[frame_name] = *it;
    }
  }
  FramePoseExtractor::Init();
}

FramePoseExtractor::FramePoseExtractor(
    const RigidBodyTree<double>& tree,
    const std::map<std::string, std::pair<std::string, int>> frame_info,
    std::vector<Eigen::Isometry3d>& frame_poses) : tree_(&tree){

  // If the frame vector is empty, set all frame poses to the identity pose.
  if (frame_poses.empty()) {
    frame_poses = std::vector<Eigen::Isometry3d>(frame_info.size(),
                                                 Eigen::Isometry3d::Identity());
  }

  DRAKE_DEMAND(frame_info.size() == frame_poses.size());

  // Create and store the related frames. Note that having the frame name as the
  // map key for @p frame_info ensures that frame names coming in are unique.
  auto m_it = frame_info.begin();
  auto v_it = frame_poses.begin();
  while (m_it != frame_info.end() && v_it != frame_poses.end()) {
    std::string frame_name = m_it->first;
    RigidBody<double>* body = tree_->FindBody(
        m_it->second.first, "", m_it->second.second);
    frame_name_to_frame_map_[frame_name] =
        new RigidBodyFrame<double>(frame_name, body, *v_it);
    ++m_it;
    ++v_it;
  }
  FramePoseExtractor::Init();
}

void FramePoseExtractor::Init() {
  // Abstract input port of type KinematicsResults
  kinematics_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // Abstract output port of type systems::rendering::PoseBundle
  pose_bundle_output_port_index_ =
      this->DeclareAbstractOutputPort(
          &FramePoseExtractor::MakeOutputStatus,
          &FramePoseExtractor::OutputStatus).get_index();
}

PoseBundle<double> FramePoseExtractor::MakeOutputStatus() const {
  PoseBundle<double> frame_pose_bundle(
      static_cast<int>(frame_name_to_frame_map_.size()));
  return frame_pose_bundle;
}

void FramePoseExtractor::OutputStatus(const systems::Context<double>& context,
                                        PoseBundle<double>* output) const {
  PoseBundle<double>& frame_pose_bundle = *output;

  // Extract the input port for the KinematicsResults object, get the
  // transformation of the bodies we care about, and fill in the
  // frame_pose_bundle object.
  const KinematicsResults<double>* kres =
      this->EvalInputValue<KinematicsResults<double>>(
          context, kinematics_input_port_index_);

  DRAKE_DEMAND(frame_pose_bundle.get_num_poses() ==
      static_cast<int>(frame_name_to_frame_map_.size()));
  int frame_index = 0;
  for (auto it = frame_name_to_frame_map_.begin();
       it != frame_name_to_frame_map_.end(); ++frame_index, ++it) {

    // Set the pose.
    frame_pose_bundle.set_name(frame_index, it->first);
    frame_pose_bundle.set_model_instance_id(
        frame_index, it->second->get_model_instance_id());
    frame_pose_bundle.set_pose(frame_index, tree_->CalcFramePoseInWorldFrame(
        kres->get_cache(), *(it->second)));

    // Set the velocities.
    SpatialVelocity<double> svel(tree_->CalcFrameSpatialVelocityInWorldFrame(
        kres->get_cache(), *(it->second)));
    FrameVelocity<double> fvel;
    fvel.set_velocity(svel);
    frame_pose_bundle.set_velocity(frame_index, fvel);
  }
}

}  // namespace drake
}  // namespace systems
}  // namespace sensors
