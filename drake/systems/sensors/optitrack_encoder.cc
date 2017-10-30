#include "drake/systems/sensors/optitrack_encoder.h"

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace systems {
namespace sensors {

using systems::sensors::TrackedBody;
using systems::rendering::PoseBundle;

OptitrackEncoder::OptitrackEncoder(
    const std::map<std::string, int>& frame_name_to_id_map,
    std::vector<std::string> rigid_body_names,
    double optitrack_lcm_publish_period) {

  // Abstract input port of type PoseBundle.
  pose_bundle_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  // Abstract output port of type vector<TrackedBody>.
  optitrack_output_port_index_ = this->DeclareAbstractOutputPort(
      &OptitrackEncoder::MakeOutputValue,
      &OptitrackEncoder::CalcOutputValue).get_index();

  if (!rigid_body_names.empty()
      && (rigid_body_names.size() != frame_name_to_id_map.size())) {
    throw std::runtime_error("OptitrackEncoder::OptitrackEncoder Error: "
                                 "Input parameters frame_name_to_id_map and "
                                 "rigid_body_names must have the same size.");
  } else if (rigid_body_names.empty()) {
    for (auto it = frame_name_to_id_map.begin();
         it != frame_name_to_id_map.end(); ++it) {
      rigid_body_names.push_back(it->first);
    }
  }

  auto body_name = rigid_body_names.begin();
  auto name_to_id = frame_name_to_id_map.begin();
  while (name_to_id != frame_name_to_id_map.end()
      && body_name != rigid_body_names.end()) {
    if (!CheckIdValidity(name_to_id->second)) {
      throw std::runtime_error("OptitrackEnc::OptitrackEnc: ERROR: "
                                   "Found invalid Optitrack rigid body id.");
    }
    rigid_body_info_map_[name_to_id->first] =
        std::make_pair(*body_name, name_to_id->second);
    ++body_name;
    ++name_to_id;
  }

  this->DeclarePeriodicUnrestrictedUpdate(optitrack_lcm_publish_period, 0);
}

bool OptitrackEncoder::CheckIdValidity(const int id) {
  auto it = std::find_if(
      rigid_body_info_map_.begin(), rigid_body_info_map_.end(),
      [id](const std::pair<std::string, std::pair<std::string, int>>& obj) {
        return obj.second.second == id;
      });
  if (it != rigid_body_info_map_.end()) {
    return false;
  } else {
    return (id >= 0);
  }
}

std::vector<TrackedBody> OptitrackEncoder::MakeOutputValue() const {
  return std::vector<TrackedBody>{rigid_body_info_map_.size()};
}

void OptitrackEncoder::CalcOutputValue(const systems::Context<double>& context,
                                       std::vector<TrackedBody>* output) const {
  std::vector<TrackedBody>& optitrack_objects = *output;

  // Extract the input port for the PoseBundle object, get the transformation of
  // the bodies we care about, and fill in the optitrack_objects vector.
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
  for (const auto& pair : rigid_body_info_map_) {
    optitrack_objects[optitrack_obj_index].body_name = pair.second.first;
    optitrack_objects[optitrack_obj_index].id = pair.second.second;
    int pose_index = frame_name_to_index_map[pair.first];
    optitrack_objects[optitrack_obj_index].T_WF =
        pose_bundle->get_pose(pose_index);
    ++optitrack_obj_index;
  }
  // Sort the tracked objects by Optitrack body ID.
  std::sort(optitrack_objects.begin(), optitrack_objects.end(),
            [](const TrackedBody& body1, const TrackedBody& body2) {
              return body1.id < body2.id;
            });
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
