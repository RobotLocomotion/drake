#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
std::ostream& operator<<(std::ostream& out, const SpatialInertia<T>& M) {
  // Write the data associated with the spatial inertia M of a body
  // (or composite body) B about a point P, expressed in a frame E.
  // Typically, point P is either Bo (B's origin) or Bcm (B's center of mass)
  // and frame E is usually the body frame B.
  const T& mass = M.get_mass();
  const Vector3<T>& p_BoBcm_B = M.get_com();
  const T& x = p_BoBcm_B.x();
  const T& y = p_BoBcm_B.y();
  const T& z = p_BoBcm_B.z();

  if constexpr (scalar_predicate<T>::is_bool) {
    out << std::endl
        << fmt::format(" mass = {}", mass) << std::endl
        << fmt::format(" Center of mass = [{}  {}  {}]", x, y, z) << std::endl;
  } else {
    // Print symbolic results.
    out << " mass = " << mass << std::endl
        << " Center of mass = [" << p_BoBcm_B.transpose() << "]" << std::endl;
  }

  // Get M's unit inertia about P (which may be Bo or Bcm) expressed in frame E.
  // Similarly, get M's unit inertia about Bcm expressed in frame E (usually B).
  const UnitInertia<T>& G_BBo_B = M.get_unit_inertia();
  const UnitInertia<T> G_BBcm_B = G_BBo_B.ShiftToCenterOfMass(p_BoBcm_B);

  // The next lines are similar to M.CalcRotationalInertia(), but without checks
  // such as IsPhysicallyValid() so that operator<< works for error messages.
  const Matrix3<T> IBo = mass * G_BBo_B.CopyToFullMatrix3();
  const Matrix3<T> IBcm = mass * G_BBcm_B.CopyToFullMatrix3();

  // Set the rotational inertia from the lower-triangular part of the matrix.
  const RotationalInertia<T> I_BBo_B =
      RotationalInertia<T>::MakeFromMomentsAndProductsOfInertia(
          IBo(0, 0), IBo(1, 1), IBo(2, 2),
          IBo(1, 0), IBo(2, 0), IBo(2, 1), true);
  const RotationalInertia<T> I_BBcm_B =
      RotationalInertia<T>::MakeFromMomentsAndProductsOfInertia(
          IBcm(0, 0), IBcm(1, 1), IBcm(2, 2),
          IBcm(1, 0), IBcm(2, 0), IBcm(2, 1), true);

  // Write B's rotational inertia about point P if P is not at Bcm.
  const boolean<T> is_position_zero = (p_BoBcm_B == Vector3<T>::Zero());
  if (!is_position_zero) {
    out << " Inertia I_BBo_B =" << std::endl
        << I_BBo_B;
  }

  // Write B's rotational inertia about Bcm.
  out << " Inertia I_BBcm_B =" << std::endl
      << I_BBcm_B;

  // Write B's principal moments of inertia about Bcm.
  if constexpr (scalar_predicate<T>::is_bool) {
    const Vector3<double> eig = I_BBcm_B.CalcPrincipalMomentsOfInertia();;
    const double Imin = eig(0), Imed = eig(1), Imax = eig(2);
    out << fmt::format(" Principal moments of inertia for Bcm =") << std::endl
        << fmt::format("[{}  {}  {}]", Imin, Imed, Imax) << std::endl;
  }

  return out;
}


DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS((
    static_cast<std::ostream&(*)(std::ostream&, const SpatialInertia<T>&)>(
        &operator<< )
))

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::SpatialInertia)
