#include "drake/systems/rendering/pose_vector.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_expression.h"
#include "drake/math/rotation_matrix.h"

namespace drake {
namespace systems {
namespace rendering {

// Linkage for kSize.
template <typename T> constexpr int PoseVector<T>::kSize;

template <typename T>
PoseVector<T>::PoseVector() : BasicVector<T>(kSize) {
  set_translation(Eigen::Translation<T, 3>::Identity());
  set_rotation(Eigen::Quaternion<T>::Identity());
}

template <typename T>
PoseVector<T>::~PoseVector() {}

template <typename T>
Isometry3<T> PoseVector<T>::get_isometry() const {
  Isometry3<T> isometry = Isometry3<T>::Identity();
  isometry.translation().x() = (*this)[0];
  isometry.translation().y() = (*this)[1];
  isometry.translation().z() = (*this)[2];
  isometry.rotate(this->get_rotation());
  return isometry;
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

template class PoseVector<double>;
template class PoseVector<AutoDiffXd>;
template class PoseVector<symbolic::Expression>;

}  // namespace rendering
}  // namespace systems
}  // namespace drake
