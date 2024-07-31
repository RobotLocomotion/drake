#include "drake/math/rigid_transform.h"

namespace drake {
namespace math {

template <typename T>
Isometry3<T> RigidTransform<T>::GetAsIsometry3() const {
  // pose.linear() returns a mutable reference to the 3x3 rotation matrix part
  // of Isometry3 and pose.translation() returns a mutable reference to the
  // 3x1 position vector part of the Isometry3.
  Isometry3<T> pose;
  pose.linear() = rotation().matrix();
  pose.translation() = translation();
  pose.makeAffine();
  return pose;
}

template <typename T>
void RigidTransform<T>::ThrowInvalidMultiplyVector4(const Vector4<T>& vec_B) {
  throw std::logic_error(
      fmt::format("The 4th element in vector [{}, {}, {}, {}] passed to "
                  "RigidTransform::operator* is not 0 or 1.",
                  vec_B(0), vec_B(1), vec_B(2), vec_B(3)));
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const RigidTransform<T>& X) {
  const RollPitchYaw<T> rpy(X.rotation());
  const Vector3<T>& p = X.translation();
  out << fmt::format("{} xyz = {} {} {}", rpy, p.x(), p.y(), p.z());
  return out;
}

// clang-format off
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const RigidTransform<T>&)>(
        &operator<< )
));
// clang-format on

}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RigidTransform);
