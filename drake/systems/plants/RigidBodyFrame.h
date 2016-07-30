#pragma once

#include <Eigen/Dense>

#include "drake/drakeRBM_export.h"
#include "drake/systems/plants/RigidBody.h"

namespace tinyxml2 {
class XMLElement;
}

class DRAKERBM_EXPORT RigidBodyFrame {
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
   * Returns the ID of the model to which this rigid body frame belongs.
   */
  int get_model_id() const;

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
   * `RigidBodyFrame` and the coordinate frame that belongs to the `RigidBody`
   * to which this frame is attached.
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
   * RigidBodyFrame` defined in the `RigidBodyTree`.
   */
  int get_frame_index() const;

  /**
   * Sets the name of this `RigidBodyFrame`.
   */
  void set_name(const std::string& name);

  /**
   * Sets the rigid body to which this frame is attached.
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
   * Sets the transform to body of this `RigidBodyFrame`.
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
