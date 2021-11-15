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
  const Vector3<T>& p_PBcm_B = M.get_com();
  const T& x = p_PBcm_B.x();
  const T& y = p_PBcm_B.y();
  const T& z = p_PBcm_B.z();

  if constexpr (scalar_predicate<T>::is_bool) {
    out << std::endl
        << fmt::format(" mass = {}", mass) << std::endl
        << fmt::format(" Center of mass = [{}  {}  {}]", x, y, z) << std::endl;
  } else {
    // Print symbolic results.
    out << " mass = " << mass << std::endl
        << " Center of mass = [" << p_PBcm_B.transpose() << "]ᵀ" << std::endl;
  }


  // Get M's unit inertia about P (which may be Bo or Bcm) expressed in frame E.
  // Similarly, get M's unit inertia about Bcm expressed in frame E.
  const UnitInertia<T>& G_BP_B = M.get_unit_inertia();
  const UnitInertia<T> G_BBcm_E = G_BP_B.ShiftToCenterOfMass(p_PBcm_B);

  // The next lines are similar to M.CalcRotationalInertia(), but without checks
  // such as IsPhysicallyValid() so that operator<< works for error messages.
  const Matrix3<T> I_BP_B = mass * G_BP_B.CopyToFullMatrix3();
  const Matrix3<T> I_BBcm_B = mass * G_BBcm_E.CopyToFullMatrix3();

  // Write B's rotational inertia about point P if P is not at Bcm.
  const boolean<T> is_position_zero = (p_PBcm_B == Vector3<T>::Zero());
  if (!is_position_zero) {
    out << " Inertia I_BP_B =" << std::endl
        << I_BP_B << std::endl;
  }

  // Write B's rotational inertia about Bcm.
  out << " Inertia I_BBcm_B =" << std::endl
      << I_BBcm_B << std::endl;

  // Write B's principal moments of inertia about Bcm.
  if constexpr (scalar_predicate<T>::is_bool) {
    const RotationalInertia<T> I(I_BBcm_B, /* skip_validity_check =*/ true);
    const Vector3<double> eig = I.CalcPrincipalMomentsOfInertia();;
    const double Imin = eig(0), Imed = eig(1), Imax = eig(2);
    out << std::endl
        << fmt::format(" Principal central moments of inertia ="
                       " {}  {}  {}", Imin, Imed, Imax) << std::endl;
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
