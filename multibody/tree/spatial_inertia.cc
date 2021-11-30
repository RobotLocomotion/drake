#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
void SpatialInertia<T>::WriteExtraCentralInertiaProperties(
    std::string* msg) const {
  DRAKE_DEMAND(msg != nullptr);
  const T& mass = get_mass();
  const Vector3<T>& p_PBcm = get_com();

  // Get G_BP, unit inertia about point P, expressed in frame E.  Use G_BP to
  // calculate G_BBcm, unit inertia about Bcm.  Then use G_BBcm to calculate
  // IBcm rotational inertia about Bcm, but without validity checks such as
  // IsPhysicallyValid() so that this method works for error messages.
  const UnitInertia<T>& G_BP = get_unit_inertia();
  const UnitInertia<T> G_BBcm = G_BP.ShiftToCenterOfMass(p_PBcm);
  const Matrix3<T> IBcm = mass * G_BBcm.CopyToFullMatrix3();

  // Set the rotational inertia from the lower-triangular part of the matrix.
  const RotationalInertia<T> I_BBcm =
      RotationalInertia<T>::MakeFromMomentsAndProductsOfInertia(
          IBcm(0, 0), IBcm(1, 1), IBcm(2, 2), IBcm(1, 0), IBcm(2, 0),
          IBcm(2, 1), /* skip_validity_check = */ true);

  // If point P is not at Bcm, write B's rotational inertia about Bcm.
  const boolean<T> is_position_zero = (p_PBcm == Vector3<T>::Zero());
  if (!is_position_zero)
    *msg += fmt::format(" Inertia about center of mass, I_BBcm =\n{}", I_BBcm);

  // Write B's principal moments of inertia about Bcm.
  if constexpr (scalar_predicate<T>::is_bool) {
    const Vector3<double> eig = I_BBcm.CalcPrincipalMomentsOfInertia();
    const double Imin = eig(0), Imed = eig(1), Imax = eig(2);
    *msg += fmt::format(
        " Principal moments of inertia about Bcm (center of mass) ="
        "\n[{}  {}  {}]\n", Imin, Imed, Imax);
  }
}

template <typename T>
std::ostream& operator<<(std::ostream& out, const SpatialInertia<T>& M) {
  // Write the data associated with the spatial inertia M of a body
  // (or composite body) B about a point P, expressed in a frame E.
  // Typically point P is either Bo (B's origin) or Bcm (B's center of mass)
  // and frame E is usually the body frame B.  More spatial inertia information
  // can be written via SpatialInertia::WriteExtraCentralInertiaProperties().
  const T& mass = M.get_mass();
  const Vector3<T>& p_PBcm = M.get_com();
  const T& x = p_PBcm.x();
  const T& y = p_PBcm.y();
  const T& z = p_PBcm.z();

  if constexpr (scalar_predicate<T>::is_bool) {
    out << std::endl
        << fmt::format(" mass = {}", mass) << std::endl
        << fmt::format(" Center of mass = [{}  {}  {}]", x, y, z) << std::endl;
  } else {
    // Print symbolic results.
    out << " mass = " << mass << std::endl
        << " Center of mass = [" << p_PBcm.transpose() << "]" << std::endl;
  }

  // Get unit inertia about point P, expressed in frame E.  Then use G_BP to
  // calculate rotational inertia about point P, but without validity checks
  // such as IsPhysicallyValid() so that operator<< works for error messages.
  const UnitInertia<T>& G_BP = M.get_unit_inertia();
  const Matrix3<T> IBP = mass * G_BP.CopyToFullMatrix3();

  // Set the rotational inertia from the lower-triangular part of the matrix.
  const RotationalInertia<T> I_BP =
      RotationalInertia<T>::MakeFromMomentsAndProductsOfInertia(
          IBP(0, 0), IBP(1, 1), IBP(2, 2),
          IBP(1, 0), IBP(2, 0), IBP(2, 1), /* skip_validity_check = */ true);

  // Write B's rotational inertia about point P.
  out << " Inertia about point P, I_BP =" << std::endl << I_BP;

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
