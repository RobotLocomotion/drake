#include "drake/systems/rendering/frame_velocity.h"

#include "drake/common/default_scalars.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
FrameVelocity<T>::FrameVelocity()
    : FrameVelocity<T>::FrameVelocity(
          multibody::SpatialVelocity<T>(Vector6<T>::Zero())) {}

template <typename T>
FrameVelocity<T>::~FrameVelocity() {}

template <typename T>
FrameVelocity<T>::FrameVelocity(const multibody::SpatialVelocity<T>& velocity)
    : BasicVector<T>(kSize) {
  set_velocity(velocity);
}

template <typename T>
FrameVelocity<T>::FrameVelocity(const FrameVelocity<T>& other)
    : BasicVector<T>(kSize) {
  this->set_value(other.get_value());
}

template <typename T>
FrameVelocity<T>& FrameVelocity<T>::operator=(const FrameVelocity<T>& other) {
  if (this == &other) return *this;
  this->set_value(other.get_value());
  return *this;
}

template <typename T>
multibody::SpatialVelocity<T> FrameVelocity<T>::get_velocity() const {
  return multibody::SpatialVelocity<T>(this->get_value());
}

template <typename T>
void FrameVelocity<T>::set_velocity(
    const multibody::SpatialVelocity<T>& velocity) {
  this->set_rotational(velocity.rotational());
  this->set_translational(velocity.translational());
}

template <typename T>
void FrameVelocity<T>::set_translational(const Vector3<T>& translational) {
  this->SetAtIndex(3, translational[0]);
  this->SetAtIndex(4, translational[1]);
  this->SetAtIndex(5, translational[2]);
}

template <typename T>
void FrameVelocity<T>::set_rotational(const Vector3<T>& rotational) {
  this->SetAtIndex(0, rotational[0]);
  this->SetAtIndex(1, rotational[1]);
  this->SetAtIndex(2, rotational[2]);
}

template <typename T>
FrameVelocity<T>* FrameVelocity<T>::DoClone() const {
  return new FrameVelocity<T>();
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::rendering::FrameVelocity)

#pragma GCC diagnostic pop
