#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace systems {
namespace sensors {

using systems::rendering::PoseBundle;

/**
 * Implements a class that maintains pose and velocity information for a set of
 * specified RigidBodyFrames. Frame information is communicated via a PoseBundle object.
 * Frames can be specified at construction in one of two ways: (1) by providing a set of RigidBody
 * names (of bodies already existing in the RigidBodyTree), which specify which
 * RigidBodies these newly created
 * frames should be attached to, or (2) by
 * providing a set of RigidBodyFrames directly. The former simply searches the
 * RigidBodyTree and assigns a RigidBodyFrame to the specified body. Frame pose
 * w.r.t. the base RigidBody frame can also be specified.
 * This system takes an abstract value input of type KinematicResults<double>
 * and generates an abstract value output of type
 * systems::rendering::PoseBundle.
 */
class FramePoseExtractor : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FramePoseExtractor)
  /**
   * Constructs a FramePoseExtractor object by taking in the RigidBodyFrames to
   * track directly as a parameter.
   * @param tree The RigidBodyTree containing the named bodies. FramePoseExtractor
   *        keeps a reference to this tree to calculate frame poses in the world frame.
   * @param frames a std::vector of RigidBodyFrames to track. Each RigidBodyFrame
   *        in this vector should have a name that is unique, i.e., the std::string returned
   *        by RigidBodyFrame::get_name() should be unique.
   * @throws std::runtime_error if any frame has a non-unique name or the frame is not
   *         attached to a RigidBody (i.e., it's RigidBody pointer is nullptr).
   */
  FramePoseExtractor(const RigidBodyTree<double>& tree,
                     const std::vector<RigidBodyFrame<double>*>& frames);

  /**
   * Constructs a FramePoseExtractor object from the information contained in
   * @p frame_info, which is a std::map whose keys denote unique frame names. Each key is
   * mapped to a std::pair that includes the RigidBody name (std::string)
   * specifying which body this frame should be attached to, and the model instance
   * id in the @p tree that contains the corresponding body.
   *
   * @param tree The RigidBodyTree containing the named bodies. FramePoseExtractor
   *        keeps a reference to this tree to calculate frame poses in the world frame.
   * @param frame_info A mapping from a unique frame name to a pair consisting of
   *        body name and model instance id.
   * @param frame_poses A vector containing each frame's pose relative to the
   *        parent body. If this vector is empty, it assumes identity for all
   *        poses.
   */
  FramePoseExtractor(
      const RigidBodyTree<double>& tree,
      const std::map<std::string, std::pair<std::string, int>> frame_info,
      std::vector<Eigen::Isometry3d>& frame_poses =
      *(new std::vector<Eigen::Isometry3d>()));

  /**
   * This InputPortDescriptor represents an abstract valued input port of type
   * KinematicsResults<double>.
   */
  const systems::InputPortDescriptor<double>& get_kinematics_input_port()
  const {
    return this->get_input_port(kinematics_input_port_index_);
  }

  int get_kinematics_input_port_index() const {
    return this->kinematics_input_port_index_;
  }


  /**
   * This OutputPort represents an abstract valued output port of type
   * systems::rendering::PoseBundle.
   */
  const systems::OutputPort<double>& get_pose_bundle_output_port() const {
    return this->get_output_port(0);
  }

  int get_pose_bundle_output_port_index() const {
    return this->pose_bundle_output_port_index_;
  }

 private:
  void Init();

  PoseBundle<double> MakeOutputStatus() const;

  void OutputStatus(const systems::Context<double>& context,
                    PoseBundle<double>* output) const;

  int kinematics_input_port_index_{-1};
  int pose_bundle_output_port_index_{-1};
  const RigidBodyTree<double>* tree_;
  std::map<std::string, RigidBodyFrame<double>*> frame_name_to_frame_map_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
