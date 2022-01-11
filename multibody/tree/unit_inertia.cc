#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCapsule(const T& r, const T& L) {
  DRAKE_THROW_UNLESS(r >= 0);
  DRAKE_THROW_UNLESS(L >= 0);
  // The capsule degenerates into a thin rod if the radius is zero.
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
  // Shift the inertia (of the lower half sphere) from about the center of the
  // sphere to the center of the capsule. Notice that the inertia of the other
  // half-sphere is the same.
  I_half_sphere.ShiftToThenAwayFromCenterOfMassInPlace(
      mass_half_sphere, Vector3<T>(0, 0, -3.0 * r / 8.0),
      Vector3<T>(0, 0, 3.0 * r / 8.0 + 0.5 * L));
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
