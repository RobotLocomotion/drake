#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/rigid_body.h"

// TODO(amcastro-tri): There should be no tinyxml2 dependence in this file.
namespace tinyxml2 {
class XMLElement;
}

/// Multibody systems typically have distinguished frames of interest that
/// need to be monitored. A frame is fully described by the body it is rigidly
/// attached to and the pose of this frame with respect to that body.
/// RigidBodyFrame provides an abstraction to describe these frames.
///
/// @tparam T The type being integrated. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RigidBodyFrame final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RigidBodyFrame)

  /**
   * A constructor where the transform-to-body is specified using an
   * Eigen::Isometry3d matrix.
   *
   */
  // TODO(amcastro-tri): Remove this constructor. The appropriate signature
  // is described in #4407.
  RigidBodyFrame(const std::string& name, RigidBody<T>* body,
                 const Eigen::Isometry3d& transform_to_body,
                 int model_instance_id = -1);

  /**
   * A constructor where the transform-to-body is specified using
   * Euler angles.
   */
  // TODO(amcastro-tri): Remove this constructor. The appropriate signature
  // is described in #4407.
  RigidBodyFrame(const std::string& name, RigidBody<T>* body,
                 const Eigen::Vector3d& xyz = Eigen::Vector3d::Zero(),
                 const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /**
   * The default constructor.
   */
  // TODO(amcastro-tri): Remove this constructor. The frame concept does not
  // make sense without referencing a RigidBody. The appropriate signature is
  // described in #4407.
  RigidBodyFrame()
      : RigidBodyFrame("", nullptr, Eigen::Isometry3d::Identity()) {}

  /**
   * Returns the ID of the model instance to which this rigid body frame
   * belongs.
   */
  int get_model_instance_id() const;

  /**
   * Returns the name of this frame.
   */
  const std::string& get_name() const { return name_; }

  /**
   * Returns the rigid body to which this frame is attached.
   */
  const RigidBody<T>& get_rigid_body() const { return *body_; }

  /**
   * Returns the rigid body to which this frame is attached.
   */
  // TODO(amcastro-tri): Users should not be accessing mutable references to
  // a RigidBody with this class.
  // Mutable references can be requested with
  // RigidBodyTree::get_mutable_body(int body_id).
  RigidBody<T>* get_mutable_rigid_body();

  /**
   * Returns the transform between the coordinate frame that belongs to this
   * `RigidBodyFrame` and the coordinate frame that belongs to the
   * `RigidBody` to which this frame is attached (which is obtainable by calling
   * RigidBodyFrame::get_rigid_body()).
   *
   * Let `B` be the coordinate frame of the `RigidBody` to which this
   * `RigidBodyFrame` is attached and `F` be the coordinate frame of this
   * `RigidBodyFrame`. Furthermore, let `p_B` be the location of a point
   * measured from `B`'s origin and expressed in coordinate frame `B`, and `p_F`
   * be the location of the same point but measured from `F`'s origin and
   * expressed in coordinate frame `F`.
   *
   * The returned value is `T_BF` where:
   *
   * <pre>
   * p_B = T_BF * p_F;
   * </pre>
   */
  const Eigen::Isometry3d& get_transform_to_body() const {
    return transform_to_body_;
  }

  // TODO(liang.fok) Remove this method once it's no longer needed by
  // drake/systems/plants/constructModelmex.cpp.
  // Frames' poses should only be specified at construction as described in
  // #4407.
  /**
   * Returns the transform between the coordinate frame that belongs to this
   * `RigidBodyFrame` and the coordinate frame that belongs to the `RigidBody`
   * to which this frame is attached.
   */
  Eigen::Isometry3d* get_mutable_transform_to_body();

  /**
   * Returns the index of this `RigidBodyFrame` within the vector of
   * `RigidBodyFrame` objects in the `RigidBodyTree`.
   */
  int get_frame_index() const { return frame_index_; }

  /**
   * Sets the name of this `RigidBodyFrame`.
   */
  void set_name(const std::string& name);

  /**
   * Sets the rigid body to which this frame is attached. Parameter
   * @p rigid_body must remain valid for the lifetime of this `RigidBodyFrame`
   * object.
   */
  // TODO(amcastro-tri): Remove this method. The rigid body to which this
  // frame is attached should only be specified at construction as described
  // in #4407.
  void set_rigid_body(RigidBody<T>* rigid_body);

  /**
   * Returns true if this frame's rigid body is equal to the @p rigid_body.
   */
  // TODO(amcastro-tri): Remove this method once parsers are fixed.
  // Change to: bool is_attached_to(const RigidBody<T>& body).
  bool has_as_rigid_body(RigidBody<T>* rigid_body);

  /**
   * Sets the index of this frame. This is the index in the vector of
   * `RigidBodyFrames` within the `RigidBodyTree`.
   */
  // TODO(amcastro-tri): Remove this method. Fix use cases to call
  // RBT::add_frame(RigidBodyFrame("the_frame_name", the_body, the_pose)).
  // The horrible idexing of frames can be fixed on a later PR, but at least
  // it would only live in one single place, RBT::add_frame().
  void set_frame_index(int frame_index);

  /**
   * Sets the transform to body of this `RigidBodyFrame`. This transform must
   * be `T_BF` as described in method RigidBodyFrame::get_transform_to_body().
   *
   * @see RigidBodyFrame::get_transform_to_body
   */
  // Frames' poses should only be specified at construction as described in
  // #4407. Fix parsers to use constructor instead.
  void set_transform_to_body(const Eigen::Isometry3d& transform_to_body);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  std::string name_;
  // TODO(amcastro-tri): make this a RigidBody<T>&. RigidBodyFrames ALWAYS
  // are attached to a body, see #4407.
  RigidBody<T>* body_{nullptr};
  Eigen::Isometry3d transform_to_body_;
  int frame_index_ = 0;  // this will be negative, but will also be gone soon!
  int model_instance_id_ = -1;
};
