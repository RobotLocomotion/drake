#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCapsule(const T& r, const T& L) {
  DRAKE_THROW_UNLESS(r >= 0);
  DRAKE_THROW_UNLESS(L >= 0);
  // A special case is required for r = 0 because r = 0 creates a zero volume
  // capsule (and we divide by volume later on). No special case for L = 0 is
  // needed because the capsule degenerates into a sphere (non-zero volume).
  if (r == 0.0) {
    return UnitInertia<T>::ThinRod(L, Vector3<T>::UnitZ());
  }
  // The capsule is regarded as a cylinder C of length L and radius r and two
  // half-spheres (each of radius r). The first half-sphere H is rigidly fixed
  // to one end of cylinder C so that the intersection between H and C forms
  // a circle centered at point Ho.  Similarly, the other half-sphere is rigidly
  // fixed to the other end of cylinder C.
  // The capsule's unit inertia about its center of mass is calculated using
  // tabulated analytical expressions from [Kane] "Dynamics: Theory and
  // Applications," McGraw-Hill, New York, 1985, by T.R. Kane and D.A. Levinson,
  // available for electronic download from Cornell digital library.

  // Calculate vc (the volume of cylinder C) and vh (volume of half-sphere H).
  const T r2 = r * r;
  const T r3 = r2 * r;
  const T vc = M_PI * r2 * L;          // vc = π r² L
  const T vh = 2.0 / 3.0 * M_PI * r3;  // vh = 2/3 π r³

  // Denoting mc as the mass of cylinder C and mh as the mass of half-sphere H,
  // and knowing the capsule has a uniform density and the capsule's mass is 1
  // (for unit inertia), calculate mc and mh.
  const T v = vc + 2 * vh;    // Volume of capsule.
  const T mc = vc / v;        // Mass in the cylinder (relates to volume).
  const T mh = vh / v;        // Mass in each half-sphere (relates to volume).

  // Form cylinder C's moments of inertia about Ccm (C's center of mass).
  // Ic_xx = Ic_yy = mc(L²/12 + r²/4)  From [Kane, Figure A20, pg. 368].
  // Ic_zz = mc r²/2                   From [Kane, Figure A20, pg. 368].
  const T Ic_xx = mc * (L*L / 12.0 + r2 / 4.0);
  const T Ic_zz = mc * r2 / 2.0;

  // Form half-sphere H's moments of inertia about Hcm (H's center of mass).
  // Ih_xx = Ih_yy = 83/320 mh r²      From [Kane, Figure A23, pg. 369].
  // Ih_zz = 2/5 mh r²                 From [Kane, Figure A23, pg. 369].
  // Note: H's inertia matrix about Ho is diag(2/5 mh r², 2/5 mh r², 2/5 mh r²).
  const T Ih_xx = 83.0 / 320.0 * mh * r2;
  const T Ih_zz = 2.0 / 5.0 * mh * r2;

  // The distance dH between Hcm and Ccm (using [Kane, Figure A23, pg. 369]) is:
  const T dH = 3.0 / 8.0 * r + L / 2.0;

  // The capsule's inertia matrix about Ccm is calculated with the shift theorem
  // (parallel axis theorem) in terms of dH (distance between Hcm and Ccm) as
  // I = Ic + 2 Ih + 2 mh dH^2 (the factor of 2 is due to the two half-spheres).
  // Note: The capsule's center of mass is coincident with Ccm.
  const T Ixx = Ic_xx + 2 * Ih_xx + 2 * mh * dH * dH;
  const T Izz = Ic_zz + 2 * Ih_zz;
  return UnitInertia(Ixx, Ixx, Izz);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::UnitInertia)
