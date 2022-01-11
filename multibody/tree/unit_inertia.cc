#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCapsule(const T& r, const T& L) {
  DRAKE_THROW_UNLESS(r >= 0);
  DRAKE_THROW_UNLESS(L >= 0);
  // A special case is required for r = 0 because r = 0 corresponds to a zero
  // volume capsule (and we divide by volume later on). No special case for L =
  // 0 is needed because the capsule degenerates into a sphere (non-zero
  // volume).
  if (r == 0.0) {
    return UnitInertia<T>::ThinRod(L, Vector3<T>::UnitZ());
  }
  const T r2 = r * r;
  const T r3 = r2 * r;
  const T mass_cylinder = M_PI * r2 * L;
  const T mass_half_sphere = M_PI * r3 * 2.0 / 3.0;
  // The unit inertia for a unit mass solid half sphere about the center of
  // of the sphere.
  RotationalInertia<T> I_half_sphere =
      mass_half_sphere * UnitInertia<T>::SolidSphere(r);
  // Denoting the half-sphere as H and point Ho as the geometrically significant
  // point at the center of the connection between the half-sphere and the
  // cylinder, the position vector from Ho to Hcm (H's center of mass),
  // expressed-in the capsule frame C as:
  const Vector3<T> p_HoHcm_C(0, 0, -3.0 * r / 8.0);
  // The position vector from Hcm to CCm (the capsule's center of mass) is
  const Vector3<T> p_HcmCcm_C(0, 0, 3.0 * r / 8.0 + 0.5 * L);
  // Shift the inertia (of the lower half sphere) from about the center of the
  // sphere to the center of the capsule. Notice that the inertia of the other
  // half-sphere is the same.
  I_half_sphere.ShiftToThenAwayFromCenterOfMassInPlace(mass_half_sphere,
                                                       p_HoHcm_C, p_HcmCcm_C);
  const RotationalInertia<T> I_cylinder =
      mass_cylinder * UnitInertia<T>::SolidCylinder(r, L);
  RotationalInertia<T> I_capsule = I_cylinder + 2 * I_half_sphere;
  // Divide by mass to get the unit inertia.
  I_capsule /= (mass_cylinder + 2.0 * mass_half_sphere);
  return UnitInertia(I_capsule);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::UnitInertia)
