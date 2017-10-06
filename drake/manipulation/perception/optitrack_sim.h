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

  bool operator < (const TrackedObject obj) const {
    return (optitrack_id < obj.optitrack_id);
  }
};

/**
 * Outputs a vector of type OptitrackSim::TrackedObject from an input of type
 * KinematicsResults<double>.
 */
class OptitrackSim : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackSim)
  /**
   * Constructs an OptitrackSim object from a map of RigidBodyFrames and
   * corresponding optitrack (Motive) IDs.
   * @param tree The RigidBodyTree
   * @param body_frame_to_id_map A mapping of RigidBodyFrames to Motive ID's
   */
  OptitrackSim(const RigidBodyTree<double>& tree,
               std::map<RigidBodyFrame<double>*, int> body_frame_to_id_map,
               double optitrack_lcm_status_period = 1/120);

  /**
   * Constructs an OptitrackSim object from a list of body names
   * @param tree The RigidBodyTree
   * @param body_name_to_id_map A mapping of body names to Motive ID's
   * @param frame_poses A vector containing each frame's pose relative to the
   *        parent body. If this vector is empty, it assumes identity for all
   *        poses.
   * @param optitrack_lcm_status_period The publish period of the lcm message.
   */
  OptitrackSim(const RigidBodyTree<double>& tree,
               std::map<std::string, int> body_name_to_id_map,
               std::vector<Eigen::Isometry3d> frame_poses =
               std::vector<Eigen::Isometry3d>(),
               double optitrack_lcm_status_period = 1/120);

  const systems::InputPortDescriptor<double>& get_kinematics_input_port() const {
    return this->get_input_port(0);
  }

  const systems::OutputPort<double>& get_optitrack_output_port() const {
    return this->get_output_port(0);
  }

 private:
  std::vector<TrackedObject> MakeOutputStatus() const;

  void Init(const RigidBodyTree<double>& tree,
            double optitrack_lcm_status_period);

  void OutputStatus(const systems::Context<double>& context,
                    std::vector<TrackedObject>* output) const;

  int kinematics_input_port_index_{-1};
  int tracked_objects_output_port_index_{-1};
  std::map<RigidBodyFrame<double>*, int> body_frame_to_id_map_; // maps urdf body name to Optitrack ID
  std::map<int, const RigidBody<double>*> id_to_body_map_;
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
