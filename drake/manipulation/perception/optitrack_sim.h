#pragma once

#include <memory>
#include <vector>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace perception {

/// The info required for tracking an object
struct TrackedObject {
  int optitrack_id;
  std::string link_name;
  Eigen::Quaterniond rotation;
  Eigen::Vector3d translation;
};

/**
 * Creates an vector of TrackedObject for a set of RigidBodies, to send out
 * via LCM on the channel OPTITRACK_FRAMES_T
 */
class OptitrackSim : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackSim)
  /**
   * Constructs the OptitrackSim.
   * @param tree The RigidBodyTree
   * @param name_to_id_map A mapping of body names (from urdf) to Motive ID's
   * @param optitrack_lcm_status_period The discrete update period of the
   * OptitrackSim.
   */
  OptitrackSim(const RigidBodyTree<double>& tree,
               std::map<std::string, int> link_name_to_id_map);

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
  std::map<std::string, int> body_name_to_id_map_; // maps urdf link name to Optitrack ID
  std::map<int, int> id_to_body_index_map_; // maps Optitrack id to body index in the RBT
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
