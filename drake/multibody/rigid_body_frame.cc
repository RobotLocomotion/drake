#include "drake/multibody/rigid_body_frame.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/math/roll_pitch_yaw.h"

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(const std::string& name, RigidBody<T>* body,
                 const Eigen::Isometry3d& transform_to_body)
      : name_(name), body_(body), transform_to_body_(transform_to_body) {}

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(const std::string& name, RigidBody<T>* body,
                               const Eigen::Vector3d& xyz,
                               const Eigen::Vector3d& rpy)
    : name_(name), body_(body) {
  transform_to_body_.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0, 0, 0, 1;
}

template <typename T>
int RigidBodyFrame<T>::get_model_instance_id() const {
  return body_->get_model_instance_id();
}

template <typename T>
const std::string& RigidBodyFrame<T>::get_name() const { return name_; }

template <typename T>
const RigidBody<T>& RigidBodyFrame<T>::get_rigid_body() const {
  return *body_;
}

template <typename T>
RigidBody<T>* RigidBodyFrame<T>::get_mutable_rigid_body() {
  return body_;
}

template <typename T>
const Eigen::Isometry3d& RigidBodyFrame<T>::get_transform_to_body() const {
  return transform_to_body_;
}

template <typename T>
Eigen::Isometry3d* RigidBodyFrame<T>::get_mutable_transform_to_body() {
  return &transform_to_body_;
}

template <typename T>
int RigidBodyFrame<T>::get_frame_index() const {
  return frame_index_;
}

template <typename T>
void RigidBodyFrame<T>::set_name(const std::string& name) {
  name_ = name;
}

template <typename T>
void RigidBodyFrame<T>::set_rigid_body(RigidBody<T>* rigid_body) {
  body_ = rigid_body;
}

template <typename T>
bool RigidBodyFrame<T>::has_as_rigid_body(RigidBody<T>* rigid_body) {
  return body_ == rigid_body;
}

template <typename T>
void RigidBodyFrame<T>::set_frame_index(int frame_index) {
  frame_index_ = frame_index;
}

template <typename T>
void RigidBodyFrame<T>::set_transform_to_body(const Eigen::Isometry3d&
    transform_to_body) {
  transform_to_body_ = transform_to_body;
}

// Explicitly instantiates on the most common scalar types.
template class RigidBodyFrame<double>;
template class RigidBodyFrame<drake::AutoDiffXd>;
template class RigidBodyFrame<drake::AutoDiffUpTo73d>;
