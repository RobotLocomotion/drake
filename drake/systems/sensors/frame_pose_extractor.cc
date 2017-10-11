#include "drake/systems/sensors/frame_pose_extractor.h"

#include <algorithm>
#include <utility>

#include "drake/multibody/rigid_body_plant/kinematics_results.h"

namespace drake {
namespace systems {
namespace sensors {

using std::string;
using drake::systems::KinematicsResults;

FramePoseExtractor::FramePoseExtractor(
    const std::map<RigidBodyFrame<double>*, int>& body_frame_to_id_map,
    double optitrack_lcm_publish_period)
    : body_frame_to_id_map_(body_frame_to_id_map) {
  FramePoseExtractor::Init(optitrack_lcm_publish_period);
}

FramePoseExtractor::FramePoseExtractor(
    const RigidBodyTree<double>& tree,
    const std::map<std::string, int>& body_name_to_id_map,
    std::vector<Eigen::Isometry3d>& frame_poses,
    double optitrack_lcm_publish_period) {
  // If the frame vector is empty, set all frames poses to identity.
  if (frame_poses.empty()) {
    frame_poses = std::vector<Eigen::Isometry3d>(body_name_to_id_map.size(),
                                                 Eigen::Isometry3d::Identity());
  }

  DRAKE_DEMAND(body_name_to_id_map.size() == frame_poses.size());

  // Create and store the related frames.
  auto m_it = body_name_to_id_map.begin();
  auto v_it = frame_poses.begin();
  while (m_it != body_name_to_id_map.end() && v_it != frame_poses.end()) {
    RigidBody<double>* body = tree.FindBody(m_it->first);
    body_frame_to_id_map_[new RigidBodyFrame<double>(m_it->first + "_f", body,
                                                     *v_it)] = m_it->second;
    ++m_it;
    ++v_it;
  }

  FramePoseExtractor::Init(optitrack_lcm_publish_period);
}

void FramePoseExtractor::Init(double optitrack_lcm_publish_period) {
  // Abstract input port of type KinematicsResults
  kinematics_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // Abstract output port of type vector<TrackedObjects>
  tracked_objects_output_port_index_ =
      this->DeclareAbstractOutputPort(&FramePoseExtractor::MakeOutputStatus,
                                      &FramePoseExtractor::OutputStatus).get_index();

  for (auto it = body_frame_to_id_map_.begin();
       it != body_frame_to_id_map_.end(); ++it) {
    RigidBody<double>* body = it->first->get_mutable_rigid_body();
    if (body == nullptr) {
      throw std::runtime_error(
          "FramePoseExtractor::Init: ERROR: found nullptr to "
          "RigidBody in RigidBodyFrame object.");
    } else if (!CheckIdValidity(it->second)) {
      throw std::runtime_error(
          "FramePoseExtractor::Init: ERROR: found invalid "
          "body frame id.");
    } else {
      id_to_body_map_[it->second] = body;
    }
  }

  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_publish_period, 0);
}

bool FramePoseExtractor::CheckIdValidity(const int id) {
  auto it =
      std::find_if(id_to_body_map_.begin(), id_to_body_map_.end(),
                   [&id](const std::pair<int, const RigidBody<double>*>& pair) {
                     return pair.first == id;
                   });
  if (it != id_to_body_map_.end()) {
    return false;
  } else {
    return (id >= 0);
  }
}

std::vector<TrackedObject> FramePoseExtractor::MakeOutputStatus() const {
  std::vector<TrackedObject> optitrack_objects(body_frame_to_id_map_.size());
  return optitrack_objects;
}

void FramePoseExtractor::OutputStatus(const systems::Context<double>& context,
                                std::vector<TrackedObject>* output) const {
  std::vector<TrackedObject>& optitrack_objects = *output;

  // Extract the input port for the KinematicsResults object, get the
  // transformation of the bodies we care about, and fill in the
  // optitrack_objects vector.
  const KinematicsResults<double>* kres =
      this->EvalInputValue<KinematicsResults<double>>(
          context, kinematics_input_port_index_);

  DRAKE_DEMAND(optitrack_objects.size() == body_frame_to_id_map_.size());
  int mocap_obj_index = 0;
  for (auto it = body_frame_to_id_map_.begin();
       it != body_frame_to_id_map_.end(); ++mocap_obj_index, ++it) {
    optitrack_objects[mocap_obj_index].frame_name = it->first->get_name();
    optitrack_objects[mocap_obj_index].optitrack_id = it->second;

    auto T_WB = kres->get_pose_in_world(*(id_to_body_map_.at(it->second)));
    auto T_BF = it->first->get_transform_to_body();

    optitrack_objects[mocap_obj_index].T_WF = T_WB * T_BF;
  }
  // Sort the tracked objects by Optitrack body ID
  std::sort(optitrack_objects.begin(), optitrack_objects.end());
}

}  // namespace drake
}  // namespace systems
}  // namespace sensors
