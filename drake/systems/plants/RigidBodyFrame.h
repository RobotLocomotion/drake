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
  RigidBodyFrame(const std::string& _name, RigidBody* _body,
                 const Eigen::Isometry3d& _transform_to_body)
      : name(_name),
        body(_body),
        transform_to_body(_transform_to_body),
        frame_index(0) {}

  /**
   * A constructor where the transform-to-body is specified using
   * Euler angles.
   */
  RigidBodyFrame(const std::string& _name, RigidBody* _body,
                 const Eigen::Vector3d& xyz = Eigen::Vector3d::Zero(),
                 const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero())
      : name(_name), body(_body), frame_index(0) {
    transform_to_body.matrix() << rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
  }

  /**
   * The default constructor.
   */
  RigidBodyFrame()
      : name(""),
        body(nullptr),
        transform_to_body(Eigen::Isometry3d::Identity()),
        frame_index(0) {}

  /**
   * Returns the ID of the model to which this rigid body frame belongs.
   */
  int get_model_id() { return body->get_model_id(); }

  std::string name;
  RigidBody* body;
  Eigen::Isometry3d transform_to_body;
  int frame_index;  // this will be negative, but will also be gone soon!

 public:
#ifndef SWIG
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
