#include "drake/systems/rendering/pose_vector.h"

#include "drake/common/default_scalars.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

namespace drake {
namespace systems {
namespace rendering {

template <typename T>
PoseVector<T>::PoseVector()
    : PoseVector<T>::PoseVector(Eigen::Quaternion<T>::Identity(),
                                Eigen::Translation<T, 3>::Identity()) {}

template <typename T>
PoseVector<T>::~PoseVector() {}

template <typename T>
PoseVector<T>::PoseVector(const Eigen::Quaternion<T>& rotation,
                          const Eigen::Translation<T, 3>& translation)
    : BasicVector<T>(kSize) {
  set_rotation(rotation);
  set_translation(translation);
}

template <typename T>
math::RigidTransform<T> PoseVector<T>::get_transform() const {
  const auto& data = *this;
  return math::RigidTransform<T>{get_rotation(),
                                 Vector3<T>{data[0], data[1], data[2]}};
}

template <typename T>
void PoseVector<T>::set_transform(const math::RigidTransform<T>& transform) {
  this->set_translation(Eigen::Translation<T, 3>(transform.translation()));
  this->set_rotation(transform.rotation().ToQuaternion());
}

template <typename T>
Eigen::Translation<T, 3> PoseVector<T>::get_translation() const {
  return Eigen::Translation<T, 3>((*this)[0], (*this)[1], (*this)[2]);
}

template <typename T>
Eigen::Quaternion<T> PoseVector<T>::get_rotation() const {
  return Eigen::Quaternion<T>((*this)[3], (*this)[4], (*this)[5], (*this)[6]);
}

template <typename T>
void PoseVector<T>::set_translation(const Eigen::Translation<T, 3>& t) {
  (*this)[0] = t.x();
  (*this)[1] = t.y();
  (*this)[2] = t.z();
}

template <typename T>
void PoseVector<T>::set_rotation(const Eigen::Quaternion<T>& q) {
  (*this)[3] = q.w();
  (*this)[4] = q.x();
  (*this)[5] = q.y();
  (*this)[6] = q.z();
}

template <typename T>
PoseVector<T>* PoseVector<T>::DoClone() const {
  return new PoseVector<T>();
}

}  // namespace rendering
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::rendering::PoseVector)

#pragma GCC diagnostic pop
