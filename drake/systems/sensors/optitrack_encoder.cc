#include "drake/systems/sensors/optitrack_encoder.h"
#include "drake/systems/rendering/pose_bundle.h"

#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <utility>

namespace drake {
namespace systems {
namespace sensors {

using systems::sensors::TrackedObject;
using systems::rendering::PoseBundle;

OptitrackEncoder::OptitrackEncoder(
    const std::map<std::string, int>& frame_name_to_id_map,
    std::vector<std::string> rigid_body_names,
    double optitrack_lcm_publish_period) {

  // Abstract input port of type PoseBundle
  pose_bundle_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // Abstract output port of type vector<TrackedObjects>
  tracked_objects_output_port_index_ = this->DeclareAbstractOutputPort(
      &OptitrackEncoder::MakeOutputStatus,
      &OptitrackEncoder::OutputStatus).get_index();

  if (!rigid_body_names.empty()
      && (rigid_body_names.size() != frame_name_to_id_map.size())) {
    throw std::runtime_error("OptitrackEncoder::OptitrackEncoder Error: "
                                 "Input parameters frame_name_to_id_map and "
                                 "rigid_body_names must have the same size.");
  } else if (rigid_body_names.empty()) {
    for (auto it = frame_name_to_id_map.begin(); it != frame_name_to_id_map.end(); ++it) {
      rigid_body_names.push_back(it->first);
    }
  }

  auto v_it = rigid_body_names.begin();
  auto m_it = frame_name_to_id_map.begin();
  while (m_it != frame_name_to_id_map.end() && v_it != rigid_body_names.end()) {
    if (!CheckIdValidity(m_it->second)) {
      throw std::runtime_error("OptitrackEnc::OptitrackEnc: ERROR: "
                                   "found invalid Optitrack rigid body id.");
    }
    rigid_body_info_map_[m_it->first] =
        std::pair<std::string, int>(*v_it, m_it->second);
    ++v_it;
    ++m_it;
  }

  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_publish_period, 0);
}

bool OptitrackEncoder::CheckIdValidity(const int id) {
  auto it = std::find_if(
      rigid_body_info_map_.begin(), rigid_body_info_map_.end(),
      [&id](const std::pair<std::string, std::pair<std::string, int>>& obj) {
        return obj.second.second == id;
      });
  if (it != rigid_body_info_map_.end()) {
    return false;
  } else {
    return (id >= 0);
  }
}

std::vector<TrackedObject> OptitrackEncoder::MakeOutputStatus() const {
  std::vector<TrackedObject> optitrack_objects(rigid_body_info_map_.size());
  return optitrack_objects;
}

void OptitrackEncoder::OutputStatus(const systems::Context<double>& context,
                                    std::vector<TrackedObject>* output) const {
  std::vector<TrackedObject>& optitrack_objects = *output;

  // Extract the input port for the PoseBundle object, get the
  // transformation of the bodies we care about, and fill in the
  // optitrack_objects vector.
  const PoseBundle<double>* pose_bundle =
      this->EvalInputValue<PoseBundle<double>>(
          context, pose_bundle_input_port_index_);

  // Create a frame name to index map for easy indexing within the PoseBundle.
  std::map<std::string, int> frame_name_to_index_map;
  for (int i = 0; i < pose_bundle->get_num_poses(); i++) {
    frame_name_to_index_map[pose_bundle->get_name(i)] = i;
  }

  DRAKE_DEMAND(optitrack_objects.size() == rigid_body_info_map_.size());
  int optitrack_obj_index = 0;
  for (auto it = rigid_body_info_map_.begin();
       it != rigid_body_info_map_.end(); ++optitrack_obj_index, ++it) {
    optitrack_objects[optitrack_obj_index].body_name_ = it->second.first;
    optitrack_objects[optitrack_obj_index].id_ = it->second.second;
    int pose_index = frame_name_to_index_map[it->first];
    optitrack_objects[optitrack_obj_index].T_WF_ =
        pose_bundle->get_pose(pose_index);
  }
  // Sort the tracked objects by Optitrack body ID
  std::sort(optitrack_objects.begin(), optitrack_objects.end());
}

}  // namespace drake
}  // namespace systems
}  // namespace sensors