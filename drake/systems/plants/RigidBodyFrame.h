#pragma once

#include <string>

#include <Eigen/Dense>

#include "drake/common/drake_export.h"
#include "drake/systems/plants/RigidBody.h"

namespace tinyxml2 {
class XMLElement;
}

class DRAKE_EXPORT RigidBodyFrame {
 public:
  /**
   * A constructor where the transform-to-body is specified using an
   * Eigen::Isometry3d matrix.
   *
   */
  RigidBodyFrame(const std::string& name, RigidBody* body,
                 const Eigen::Isometry3d& transform_to_body);

  /**
   * A constructor where the transform-to-body is specified using
   * Euler angles.
   */
  RigidBodyFrame(const std::string& name, RigidBody* body,
                 const Eigen::Vector3d& xyz = Eigen::Vector3d::Zero(),
                 const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /**
   * The default constructor.
   */
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
  const std::string& get_name() const;

  /**
   * Returns the rigid body to which this frame is attached.
   */
  const RigidBody& get_rigid_body() const;

  /**
   * Returns the rigid body to which this frame is attached.
   */
  RigidBody* get_mutable_rigid_body();

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
  const Eigen::Isometry3d& get_transform_to_body() const;

  // TODO(liang.fok) Remove this method once it's no longer needed by
  // drake-distro/drake/systems/plants/constructModelmex.cpp.
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
  int get_frame_index() const;

  /**
   * Sets the name of this `RigidBodyFrame`.
   */
  void set_name(const std::string& name);

  /**
   * Sets the rigid body to which this frame is attached. Parameter
   * @p rigid_body must remain valid for the lifetime of this `RigidBodyFrame`
   * object.
   */
  void set_rigid_body(RigidBody* rigid_body);

  /**
   * Returns true if this frame's rigid body is equal to the @p rigid_body.
   */
  bool has_as_rigid_body(RigidBody* rigid_body);

  /**
   * Sets the index of this frame. This is the index in the vector of
   * `RigidBodyFrames` within the `RigidBodyTree`.
   */
  void set_frame_index(int frame_index);

  /**
   * Sets the transform to body of this `RigidBodyFrame`. This transform must
   * be `T_BF` as described in method RigidBodyFrame::get_transform_to_body().
   *
   * @see RigidBodyFrame::get_transform_to_body
   */
  void set_transform_to_body(const Eigen::Isometry3d& transform_to_body);

#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

 private:
  std::string name_;
  RigidBody* body_{nullptr};
  Eigen::Isometry3d transform_to_body_;
  int frame_index_ = 0;  // this will be negative, but will also be gone soon!
};
