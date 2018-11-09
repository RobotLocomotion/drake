#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/multibody/rigid_body.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace manipulation {
namespace util {


// TODO(rcory): Template FramePoseTracker on type T
/**
 * Implements a class that maintains pose information for a set of specified
 * RigidBodyFrames. Frame information is communicated via a FramePoseVector
 * object. Frames can be specified at construction in one of two ways: (1) by
 * providing a set of RigidBody names (of bodies already existing in the
 * RigidBodyTree), which specify which RigidBodies these newly created frames
 * should be attached to, or (2) by providing a set of RigidBodyFrames directly.
 * The former simply searches the RigidBodyTree and assigns a RigidBodyFrame to
 * the specified body. Frame pose w.r.t. the base RigidBody frame can also be
 * specified. This system takes an abstract valued input of type
 * KinematicResults<double> and generates an abstract value output of type
 * geometry::FramePoseVector<double>.
 *
 * @ingroup manipulation_systems
 */
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
   * @p frames_info, which is a std::map whose keys denote unique names.
   * Each key is mapped to a std::pair that includes a RigidBody or
   * RigidBodyFrame name (std::string) specifying which parent this frame
   * should be attached to, and the model instance id in the @p tree that
   * contains the corresponding parent frame.
   *
   * @param tree The RigidBodyTree containing the named bodies. FramePoseTracker
   *        keeps a reference to this tree to calculate frame poses in the world
   *        frame.
   * @param frames_info A mapping from a name (unique in this mapping) to a
   *        std::pair consisting of the parent name and model instance id. The
   *        parent is either a frame or body. If model instance id is -1, every
   *        model is searched.
  * @param frame_poses A vector containing each frame's pose relative to the
   *        parent body or frame. If this vector is empty, it assumes identity
   *        for all poses.
   */
  FramePoseTracker(
      const RigidBodyTree<double>& tree,
      const std::map<std::string, std::pair<std::string, int>> frames_info,
      std::vector<Eigen::Isometry3d> frame_poses =
      std::vector<Eigen::Isometry3d>());

  /**
   * This InputPort represents an abstract valued input port of type
   * KinematicsResults<double>.
   */
  const systems::InputPort<double>& get_kinematics_input_port()
  const {
    return this->get_input_port(kinematics_input_port_index_);
  }

  int get_kinematics_input_port_index() const {
    return this->kinematics_input_port_index_;
  }

  /**
   * This OutputPort represents an abstract valued output port of type
   * geometry::FramePoseVector. This port cannot be connected to
   * geometry::SceneGraph (an exception will be thrown). This port exists only
   * to connect to other systems which expect a FramePoseVector
   * (e.g. systems::sensors::OptitrackLcmFrameSender).
   */
  const systems::OutputPort<double>& get_pose_vector_output_port() const {
    return this->get_output_port(pose_vector_output_port_index_);
  }

  int get_pose_vector_output_port_index() const {
    return this->pose_vector_output_port_index_;
  }

  const std::map<std::string, geometry::FrameId>&
  get_frame_name_to_id_map() const {
    return frame_name_to_id_map_;
  }

  RigidBodyFrame<double>* get_mutable_frame(std::string frame_name) {
    return this->frame_name_to_frame_map_[frame_name].get();
  }

 private:
  void Init();

  void OutputStatus(const systems::Context<double>& context,
                    geometry::FramePoseVector<double>* output) const;

  int kinematics_input_port_index_{-1};
  int pose_vector_output_port_index_{-1};
  const RigidBodyTree<double>* tree_;
  std::map<std::string,
           std::unique_ptr<RigidBodyFrame<double>>> frame_name_to_frame_map_;
  const geometry::SourceId source_id_;
  std::map<std::string, geometry::FrameId> frame_name_to_id_map_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
