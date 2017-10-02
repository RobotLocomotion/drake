#include "drake/manipulation/perception/optitrack_sim.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"

namespace drake {
namespace manipulation {
namespace perception {

using std::string;
using drake::systems::KinematicsResults;

OptitrackSim::OptitrackSim(const RigidBodyTree<double>& tree,
                           std::map<RigidBodyFrame<double>, int> body_frame_to_id_map)
                           : body_frame_to_id_map_(body_frame_to_id_map) {

  /// Abstract input port of type KinematicsResults
  kinematics_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  /// Abstract output port of type vector<TrackedObjects>
  tracked_objects_output_port_index_ = this->DeclareAbstractOutputPort(
      &OptitrackSim::MakeOutputStatus, &OptitrackSim::OutputStatus).get_index();

  for (auto it = body_frame_to_id_map_.begin(); it != body_frame_to_id_map_.end(); ++it) {
    id_to_body_index_map_[it->second] = tree.FindBodyIndex(it->first.get_rigid_body().get_name());
  }
}

std::vector<TrackedObject> OptitrackSim::MakeOutputStatus() const {
  std::vector<TrackedObject> optitrack_objects(body_frame_to_id_map_.size());
  return optitrack_objects;
}

void OptitrackSim::OutputStatus(const systems::Context<double>& context,
                  std::vector<TrackedObject>* output) const {

  std::vector<TrackedObject>& optitrack_objects = *output;

  // in here we extract the input port for KinematicsResults object
  // get the transformation of the bodies we care about
  // and fill in the optitrack_objectss vector for sending out
  const KinematicsResults<double>* kres =
      this->EvalInputValue<KinematicsResults<double>>(context, kinematics_input_port_index_);

  DRAKE_DEMAND(optitrack_objects.size() == body_frame_to_id_map_.size());
  int mocap_obj_index = 0;
  for (auto it = body_frame_to_id_map_.begin(); it != body_frame_to_id_map_.end();
       ++mocap_obj_index, ++it) {

    optitrack_objects[mocap_obj_index].frame_name = it->first.get_name();
    optitrack_objects[mocap_obj_index].optitrack_id = it->second;

    auto T_WB = kres->get_body_orientation(id_to_body_index_map_.at(it->second));
    auto T_BF = it->first.get_transform_to_body();

    optitrack_objects[mocap_obj_index].T_WF = T_WB*T_BF;
  }
}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake