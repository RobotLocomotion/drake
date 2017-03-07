#include "drake/systems/rendering/frame_velocity.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"

namespace drake {
namespace systems {
namespace rendering {

// Linkage for kSize.
template <typename T> constexpr int FrameVelocity<T>::kSize;

template <typename T>
FrameVelocity<T>::FrameVelocity() : BasicVector<T>(kSize) {
  this->set_value(Vector6<T>::Zero());
}

template <typename T>
FrameVelocity<T>::~FrameVelocity() {}

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

template class FrameVelocity<double>;
template class FrameVelocity<AutoDiffXd>;
template class FrameVelocity<symbolic::Expression>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
