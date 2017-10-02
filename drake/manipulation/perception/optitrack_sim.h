#pragma once

#include <memory>
#include <vector>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body.h"

namespace drake {
namespace manipulation {
namespace perception {

/// The info required for tracking an object
struct TrackedObject {
  int optitrack_id;
  std::string frame_name;
  Eigen::Isometry3d T_WF;
};

/**
 * Creates an vector of TrackedObject for a set of RigidBodies, to send out
 * via LCM on the channel OPTITRACK_FRAMES_T
 */
class OptitrackSim : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackSim)
  /**
   * Constructs an OptitrackSim object from a list of body names
   * @param tree The RigidBodyTree
   * @param body_frame_to_id_map A mapping of RB frames to Motive ID's
   */
  OptitrackSim(const RigidBodyTree<double>& tree,
               std::map<RigidBodyFrame<double>, int> body_frame_to_id_map);

  const systems::InputPortDescriptor<double>& get_kinematics_input_port() const {
    return this->get_input_port(0);
  }

  const systems::OutputPort<double>& get_optitrack_output_port() const {
    return this->get_output_port(0);
  }

 private:
  std::vector<TrackedObject> MakeOutputStatus() const;

  void OutputStatus(const systems::Context<double>& context,
                    std::vector<TrackedObject>* output) const;

  int kinematics_input_port_index_;
  int tracked_objects_output_port_index_;
  std::map<RigidBodyFrame<double>, int> body_frame_to_id_map_; // maps urdf body name to Optitrack ID
  std::map<int, int> id_to_body_index_map_; // maps Optitrack id to body index in the RBT
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
