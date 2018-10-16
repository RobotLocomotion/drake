#include "drake/multibody/rigid_body_frame.h"

#include <memory>
#include <utility>

#include "drake/common/autodiff.h"
#include "drake/math/rotation_matrix.h"

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(const std::string& name, RigidBody<T>* body,
                 const Eigen::Isometry3d& transform_to_body,
                 int model_instance_id)
      : name_(name), body_(body), transform_to_body_(transform_to_body),
        model_instance_id_(model_instance_id) {}

template <typename T>
RigidBodyFrame<T>::RigidBodyFrame(const std::string& name, RigidBody<T>* body,
                               const Eigen::Vector3d& xyz,
                               const Eigen::Vector3d& rpy)
    : name_(name), body_(body) {
  const drake::math::RollPitchYaw<double> roll_pitch_yaw(rpy);
  transform_to_body_.matrix() << roll_pitch_yaw.ToMatrix3ViaRotationMatrix(),
                                 xyz, 0, 0, 0, 1;
}

template <typename T>
int RigidBodyFrame<T>::get_model_instance_id() const {
  if (model_instance_id_ == -1) {
    return body_->get_model_instance_id();
  } else {
    return model_instance_id_;
  }
}

template <typename T>
RigidBody<T>* RigidBodyFrame<T>::get_mutable_rigid_body() {
  return body_;
}

template <typename T>
Eigen::Isometry3d* RigidBodyFrame<T>::get_mutable_transform_to_body() {
  return &transform_to_body_;
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
