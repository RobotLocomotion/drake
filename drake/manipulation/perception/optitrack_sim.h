#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace perception {

// A structure containing the info required to track an object.
struct TrackedObject {
  int optitrack_id;
  std::string frame_name;
  Eigen::Isometry3d T_WF;

  bool operator < (const TrackedObject obj) const {
    return (optitrack_id < obj.optitrack_id);
  }
};

/**
 * Implements an interface for defining frames that are to be treated as
 * Optitrack rigid bodies. This is useful when implementing systems that are
 * meant to interface with actual Optitrack hardware, but are to be tested in
 * simulation. This interface provides a description of each tracked object that
 * is less complete than what Motive would provide. Namely, each tracked object
 * in this interface is described by its Optitrack ID, it's name, and it's pose
 * w.r.t. the world. No information about markers, marker sets, etc. is
 * provided in this interface.
 * Optitrack bodies can be created in one of two ways: (1) by providing the
 * name of each RigidBody (in the RigidBodyTree) to be tracked, or (2) by
 * providing a set of RigidBodyFrames directly. The former simply searches the
 * RigidBodyTree and assigns a RigidBodyFrame to the specified body. Frame pose
 * w.r.t. the RigidBody can also be specified.
 * This system takes an abstract value input of type KinematicResults<double>
 * and generates an abstract value output of type
 * std::vector<OptitrackSim::TrackedObject>. The output of this system can be
 * connected to an OptitrackFrameSender system which populates an
 * optitrack_frame_t object.
 */
class OptitrackSim : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OptitrackSim)
  /**
   * Constructs an OptitrackSim object that tracks a set of RigidBodyFrames.
   * Each RigidBodyFrame has an associated Optitrack ID.
   * @param tree The RigidBodyTree
   * @param body_frame_to_id_map A mapping of RigidBodyFrames to Motive ID's
   */
  OptitrackSim(const RigidBodyTree<double>& tree,
               std::map<RigidBodyFrame<double>*, int> body_frame_to_id_map,
               double optitrack_lcm_status_period = 1/120);

  /**
   * Constructs an OptitrackSim object from body names that exist in the
   * @param tree.
   * @param tree The RigidBodyTree containing the named bodies
   * @param body_name_to_id_map A mapping of body names to Optitrack ID's
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

  const systems::InputPortDescriptor<double>& get_kinematics_input_port()
  const {
    return this->get_input_port(0);
  }

  const systems::OutputPort<double>& get_optitrack_output_port() const {
    return this->get_output_port(0);
  }

 private:
  void Init(const RigidBodyTree<double>& tree,
            double optitrack_lcm_status_period);

  // Checks whether the Optitrack ID is valid, i.e., is greater than zero
  // and not repeated.
  bool CheckValidId(const int id);

  std::vector<TrackedObject> MakeOutputStatus() const;

  void OutputStatus(const systems::Context<double>& context,
                    std::vector<TrackedObject>* output) const;

  int kinematics_input_port_index_{-1};
  int tracked_objects_output_port_index_{-1};
  std::map<RigidBodyFrame<double>*, int> body_frame_to_id_map_;
  std::map<int, const RigidBody<double>*> id_to_body_map_;
};

}  // namespace perception
}  // namespace manipulation
}  // namespace drake
