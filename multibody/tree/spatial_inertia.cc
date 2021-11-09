#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
std::ostream& operator<<(std::ostream& out,
                         const SpatialInertia<T>& M) {
  const T& mass = M.get_mass();
  const Vector3<T>& p_BoBcm_B = M.get_com();
  const T& x = p_BoBcm_B.x();
  const T& y = p_BoBcm_B.y();
  const T& z = p_BoBcm_B.z();
  // The next line is similar to M.CalcRotationalInertia(), but without checks
  // such as IsPhysicallyValid() so that operator<< works for error messages.
  const Matrix3<T> I_BBcm_B = mass * M.get_unit_inertia().CopyToFullMatrix3();
  if constexpr (scalar_predicate<T>::is_bool) {
    out << std::endl
        << fmt::format(" mass = {}", mass) << std::endl
        << fmt::format(" com = [{}  {}  {}]ᵀ", x, y, z) << std::endl
        << " I =" << std::endl
        << I_BBcm_B << std::endl;
  } else {
    // Print symbolic results.
    out << " mass = " << mass << std::endl
        << " com = [" << p_BoBcm_B.transpose() << "]ᵀ" << std::endl
        << " I =" << std::endl
        << I_BBcm_B << std::endl;
  }
  return out;
}

#if 0
  return o << std::endl
      << " mass = " << M.get_mass() << std::endl
      << " com = [" << M.get_com().transpose() << "]ᵀ" << std::endl
      << " I =" << std::endl
      // Like M.CalcRotationalInertia(), but without the IsPhysicallyValid
      // checks, so that we can use operator<< in error messages.
      << (M.get_mass() * M.get_unit_inertia().CopyToFullMatrix3())
      << std::endl;

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
#endif

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const SpatialInertia<T>&)>(
        &operator<< )
))

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::SpatialInertia)
