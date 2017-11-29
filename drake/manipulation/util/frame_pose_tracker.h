#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"

namespace drake {
namespace manipulation {
namespace util {

using systems::rendering::PoseBundle;

/**
 * Implements a class that maintains pose and velocity information for a set of
 * specified RigidBodyFrames. Frame information is communicated via a PoseBundle
 * object. Frames can be specified at construction in one of two ways: (1) by
 * providing a set of RigidBody names (of bodies already existing in the
 * RigidBodyTree), which specify which RigidBodies these newly created frames
 * should be attached to, or (2) by providing a set of RigidBodyFrames directly.
 * The former simply searches the RigidBodyTree and assigns a RigidBodyFrame to
 * the specified body. Frame pose w.r.t. the base RigidBody frame can also be
 * specified.
 * This system takes an abstract valued input of type KinematicResults<double>
 * and generates an abstract value output of type
 * systems::rendering::PoseBundle<double>.
 */
// TODO(rcory): Template FramePoseTracker on type T
class FramePoseTracker : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FramePoseTracker)
  /**
   * Constructs a FramePoseTracker object by directly taking in the
   * RigidBodyFrames to track.
   * @param tree The RigidBodyTree containing the named bodies. FramePoseTracker
   *        keeps a reference to this tree to calculate frame poses in the world
   *        frame.
   * @param frames a std::vector of unique pointers to RigidBodyFrames that are
   *        to be tracked. Pointer ownership is transferred to this class and
   *        @p frames is cleared before returning. Each RigidBodyFrame in this
   *        vector should have a name that is unique, i.e., the std::string
   *        returned by RigidBodyFrame::get_name() should not match any other
   *        frame name.
   * @throws std::runtime_error if any frame has a non-unique name or the frame
   *         is not attached to a RigidBody (i.e., its RigidBody pointer is
   *         nullptr) or @p frames is a nullptr or points to an empty
   *         std::vector.
   */
  FramePoseTracker(
      const RigidBodyTree<double>& tree,
      std::vector<std::unique_ptr<RigidBodyFrame<double>>>* frames);

  /**
   * Constructs a FramePoseTracker object from the information contained in
   * @p frames_info, which is a std::map whose keys denote unique frame names.
   * Each key is mapped to a std::pair that includes the RigidBody name
   * (std::string) specifying which body this frame should be attached to, and
   * the model instance id in the @p tree that contains the corresponding body.
   *
   * @param tree The RigidBodyTree containing the named bodies. FramePoseTracker
   *        keeps a reference to this tree to calculate frame poses in the world
   *        frame.
   * @param frames_info A mapping from a unique frame name to a std::pair
   *        consisting of body name and model instance id. If model instance id
   *        is -1, every model is searched.
   * @param frame_poses A vector containing each frame's pose relative to the
   *        parent body. If this vector is empty, it assumes identity for all
   *        poses.
   */
  FramePoseTracker(
      const RigidBodyTree<double>& tree,
      const std::map<std::string, std::pair<std::string, int>> frames_info,
      std::vector<Eigen::Isometry3d> frame_poses =
      std::vector<Eigen::Isometry3d>());

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

  std::size_t get_num_tracked_frames() const {
    return this->frame_name_to_frame_map_.size();
  }

  std::vector<std::string> get_tracked_frame_names();

  RigidBodyFrame<double>* get_mutable_frame(std::string frame_name) {
    return this->frame_name_to_frame_map_[frame_name].get();
  }

 private:
  void Init();

  PoseBundle<double> MakeOutputStatus() const;

  void OutputStatus(const systems::Context<double>& context,
                    PoseBundle<double>* output) const;

  int kinematics_input_port_index_{-1};
  int pose_bundle_output_port_index_{-1};
  const RigidBodyTree<double>* tree_;
  std::map<std::string,
           std::unique_ptr<RigidBodyFrame<double>>> frame_name_to_frame_map_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
