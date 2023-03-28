#include "drake/math/rigid_transform.h"

namespace drake {
namespace math {

template <typename T>
RigidTransform<T>::RigidTransform(const Matrix4<T>& pose) {
  DRAKE_ASSERT_VOID(ThrowIfInvalidBottomRow(pose));
  set_rotation(RotationMatrix<T>(pose.template block<3, 3>(0, 0)));
  set_translation(pose.template block<3, 1>(0, 3));
}

template <typename T>
Matrix4<T> RigidTransform<T>::GetAsMatrix4() const {
  Matrix4<T> pose;
  pose.template topLeftCorner<3, 3>() = rotation().matrix();
  pose.template topRightCorner<3, 1>() = translation();
  pose.row(3) = Vector4<T>(0, 0, 0, 1);
  return pose;
}

template <typename T>
Eigen::Matrix<T, 3, 4> RigidTransform<T>::GetAsMatrix34() const {
  Eigen::Matrix<T, 3, 4> pose;
  pose.template topLeftCorner<3, 3>() = rotation().matrix();
  pose.template topRightCorner<3, 1>() = translation();
  return pose;
}

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
RigidTransform<T> RigidTransform<T>::inverse() const {
  // @internal This method's name was chosen to mimic Eigen's inverse().
  RigidTransform<T> X_BA(internal::DoNotInitializeMemberFields{});
  X_BA.set_rotation(R_AB_.inverse());
  const RotationMatrix<T>& R_BA = X_BA.rotation();
  X_BA.set_translation(R_BA * (-p_AoBo_A_));
  return X_BA;
}

template <typename T>
void RigidTransform<T>::ThrowInvalidMultiplyVector4(const Vector4<T>& vec_B) {
  throw std::logic_error(fmt::format(
      "The 4th element in vector [{}, {}, {}, {}] passed to "
      "RigidTransform::operator* is not 0 or 1.",
      vec_B(0), vec_B(1), vec_B(2), vec_B(3)));
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const RigidTransform<T>& X) {
  const RollPitchYaw<T> rpy(X.rotation());
  const Vector3<T>& p = X.translation();
  out << fmt::format("{} xyz = {} {} {}", rpy, p.x(), p.y(), p.z());;
  return out;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const RigidTransform<T>&)>(
        &operator<< )
))

}  // namespace math
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::math::RigidTransform)
