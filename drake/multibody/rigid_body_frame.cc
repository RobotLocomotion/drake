#include "drake/multibody/rigid_body_frame.h"

#include "drake/math/roll_pitch_yaw.h"

RigidBodyFrame::RigidBodyFrame(const std::string& name, RigidBody<double>* body,
                 const Eigen::Isometry3d& transform_to_body)
      : name_(name), body_(body), transform_to_body_(transform_to_body) {}

RigidBodyFrame::RigidBodyFrame(const std::string& name, RigidBody<double>* body,
                               const Eigen::Vector3d& xyz,
                               const Eigen::Vector3d& rpy)
    : name_(name), body_(body) {
  transform_to_body_.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

int RigidBodyFrame::get_model_instance_id() const {
  return body_->get_model_instance_id();
}

const std::string& RigidBodyFrame::get_name() const { return name_; }

const RigidBody<double>& RigidBodyFrame::get_rigid_body() const {
  return *body_;
}

RigidBody<double>* RigidBodyFrame::get_mutable_rigid_body() {
  return body_;
}

const Eigen::Isometry3d& RigidBodyFrame::get_transform_to_body() const {
  return transform_to_body_;
}

Eigen::Isometry3d* RigidBodyFrame::get_mutable_transform_to_body() {
  return &transform_to_body_;
}

int RigidBodyFrame::get_frame_index() const {
  return frame_index_;
}

void RigidBodyFrame::set_name(const std::string& name) {
  name_ = name;
}

void RigidBodyFrame::set_rigid_body(RigidBody<double>* rigid_body) {
  body_ = rigid_body;
}

bool RigidBodyFrame::has_as_rigid_body(RigidBody<double>* rigid_body) {
  return body_ == rigid_body;
}

void RigidBodyFrame::set_frame_index(int frame_index) {
  frame_index_ = frame_index;
}

void RigidBodyFrame::set_transform_to_body(const Eigen::Isometry3d&
    transform_to_body) {
  transform_to_body_ = transform_to_body;
}
