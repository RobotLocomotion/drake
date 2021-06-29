#include "drake/math/rigid_transform.h"

namespace drake {
namespace math {

template <typename T>
std::ostream& operator<<(std::ostream& out, const RigidTransform<T>& X) {
  const Vector3<T>& p = X.translation();
  if constexpr (scalar_predicate<T>::is_bool) {
    const RotationMatrix<T>& R = X.rotation();
    const RollPitchYaw<T> rpy(R);
    out << rpy;
    out << fmt::format(" xyz = {} {} {}", p.x(), p.y(), p.z());;
  } else {
    // TODO(14927) For symbolic type T, stream roll, pitch, yaw if conversion
    //  from RotationMatrix to RollPitchYaw can be done in a way that provides
    //  meaningful output to the end-user or developer (it is not trivial how
    //  to do this symbolic conversion) and does not require an Environment.
    out << "rpy = symbolic (not supported)";
    out << " xyz = " << p.x() << " " << p.y() << " " << p.z();
  }
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
