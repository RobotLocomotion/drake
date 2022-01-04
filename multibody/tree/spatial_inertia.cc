#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
void SpatialInertia<T>::ThrowNotPhysicallyValid() const {
  std::string error_message = fmt::format(
      "Spatial inertia fails SpatialInertia::IsPhysicallyValid()."
      "{}", *this);
  WriteExtraCentralInertiaProperties(&error_message);
  throw std::runtime_error(error_message);
}

template <typename T>
void SpatialInertia<T>::WriteExtraCentralInertiaProperties(
    std::string* message) const {
  DRAKE_DEMAND(message != nullptr);
  const T& mass = get_mass();
  const Vector3<T>& p_PBcm = get_com();

  // Get G_BP (unit inertia about point P) and use it to calculate G_BBcm (unit
  // inertia about Bcm).  Use G_BBcm to calculate I_BBcm (rotational inertia
  // about Bcm) without validity checks such as IsPhysicallyValid().
  // Hence, this method works for error messages.
  const UnitInertia<T>& G_BP = get_unit_inertia();
  const UnitInertia<T> G_BBcm = G_BP.ShiftToCenterOfMass(p_PBcm);
  const RotationalInertia<T> I_BBcm =
      G_BBcm.MultiplyByScalarSkipValidityCheck(mass);

  // If point P is not at Bcm, write B's rotational inertia about Bcm.
  const boolean<T> is_position_zero = (p_PBcm == Vector3<T>::Zero());
  if (!is_position_zero) {
    *message += fmt::format(
        " Inertia about center of mass, I_BBcm =\n{}", I_BBcm);
  }

  // Write B's principal moments of inertia about Bcm.
  if constexpr (scalar_predicate<T>::is_bool) {
    const Vector3<double> eig = I_BBcm.CalcPrincipalMomentsOfInertia();
    const double Imin = eig(0), Imed = eig(1), Imax = eig(2);
    *message += fmt::format(
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
    out << "\n"
        << fmt::format(" mass = {}\n", mass)
        << fmt::format(" Center of mass = [{}  {}  {}]\n", x, y, z);
  } else {
    // Print symbolic results.
    out << " mass = " << mass << "\n"
        << " Center of mass = [" << p_PBcm.transpose() << "]\n";
  }

  // Get G_BP (unit inertia about point P) and use it to calculate I_BP
  // (rotational inertia about P) without validity checks such as
  // IsPhysicallyValid().  Hence, this method works for error messages.
  const UnitInertia<T>& G_BP = M.get_unit_inertia();
  const RotationalInertia<T> I_BP =
      G_BP.MultiplyByScalarSkipValidityCheck(mass);

  // Write B's rotational inertia about point P.
  out << " Inertia about point P, I_BP =\n" << I_BP;

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
