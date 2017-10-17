#include "drake/manipulation/perception/optitrack_sim.h"
#include "drake/multibody/rigid_body_plant/kinematics_results.h"

namespace drake {
namespace manipulation {
namespace perception {

using std::string;
using drake::systems::KinematicsResults;

OptitrackSim::OptitrackSim(const RigidBodyTree<double>& tree,
                           std::map<std::string, int> body_name_to_id_map)
                           : body_name_to_id_map_(body_name_to_id_map) {

  /// Abstract input port of type KinematicsResults
  kinematics_input_port_index_ = this->DeclareAbstractInputPort().get_index();

  /// Abstract output port of type vector<TrackedObjects>
  tracked_objects_output_port_index_ = this->DeclareAbstractOutputPort(
      &OptitrackSim::MakeOutputStatus, &OptitrackSim::OutputStatus).get_index();

  for (auto it = body_name_to_id_map_.begin(); it != body_name_to_id_map_.end(); ++it) {
    id_to_body_index_map_[it->second] = tree.FindBodyIndex(it->first);
  }
}

std::vector<TrackedObject> OptitrackSim::MakeOutputStatus() const {
  std::vector<TrackedObject> optitrack_objects(body_name_to_id_map_.size());
  return optitrack_objects;
}

void OptitrackSim::OutputStatus(const systems::Context<double>& context,
                  std::vector<TrackedObject>* output) const {

  std::vector<TrackedObject>& optitrack_objectss = *output;

  // in here we extract the input port for KinematicsResults object
  // get the transformation of the bodies we care about
  // and fill in the optitrack_objectss vector for sending out
  const KinematicsResults<double>* kres =
      this->EvalInputValue<KinematicsResults<double>>(context, kinematics_input_port_index_);

  DRAKE_DEMAND(optitrack_objectss.size() == body_name_to_id_map_.size());
  int mocap_obj_index = 0;
  for (auto it = body_name_to_id_map_.begin(); it != body_name_to_id_map_.end();
       ++mocap_obj_index, ++it) {

    optitrack_objectss[mocap_obj_index].link_name = it->first;
    optitrack_objectss[mocap_obj_index].optitrack_id = it->second;

    optitrack_objectss[mocap_obj_index].rotation = kres->get_body_orientation(
        id_to_body_index_map_.at(it->second));

    optitrack_objectss[mocap_obj_index].translation = kres->get_body_position(
        id_to_body_index_map_.at(it->second));
  }

}

}  // namespace perception
}  // namespace manipulation
}  // namespace drake