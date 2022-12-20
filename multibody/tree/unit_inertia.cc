#include "drake/multibody/tree/unit_inertia.h"

namespace drake {
namespace multibody {

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidBox(const T& Lx, const T& Ly, const T& Lz) {
  if (Lx < T(0) || Ly < T(0) || Lz < T(0)) {
    const std::string msg =
        "A length argument to UnitInertia::SolidBox() "
        "is negative.";
    throw std::logic_error(msg);
  }
  const T one_twelfth = T(1) / T(12);
  const T Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
  return UnitInertia(one_twelfth * (Ly2 + Lz2), one_twelfth * (Lx2 + Lz2),
                     one_twelfth * (Lx2 + Ly2));
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCapsule(const T& r, const T& L,
    const Vector3<T>& unit_vector) {
  // Ensure r and L are non-negative and ‖unit_vector‖ is within 6 bits of 1.0.
  // Note: UnitInertia::AxiallySymmetric() normalizes unit_vector before use.
  using std::abs;
  DRAKE_THROW_UNLESS(r >= 0);
  DRAKE_THROW_UNLESS(L >= 0);
  DRAKE_THROW_UNLESS(abs(unit_vector.norm() - 1) <=
    64 * std::numeric_limits<double>::epsilon());

  // A special case is required for r = 0 because r = 0 creates a zero volume
  // capsule (and we divide by volume later on). No special case for L = 0 is
  // needed because the capsule degenerates into a sphere (non-zero volume).
  if (r == 0.0) {
    return UnitInertia<T>::ThinRod(L, unit_vector);
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

  // The distance dH between Hcm (the half-sphere H's center of mass) and Ccm
  // (the cylinder C's center of mass) is from [Kane, Figure A23, pg. 369].
  // dH = 3.0 / 8.0 * r + L / 2.0;
  const T dH = 0.375 * r + 0.5 * L;

  // The discussion that follows assumes Ic_zz is the axial moment of inertia
  // and Ix_xx = Ic_yy is the transverse moment of inertia.
  // Form cylinder C's moments of inertia about Ccm (C's center of mass).
  // Ic_xx = Ic_yy = mc(L²/12 + r²/4)  From [Kane, Figure A20, pg. 368].
  // Ic_zz = mc r²/2                   From [Kane, Figure A20, pg. 368].

  // Form half-sphere H's moments of inertia about Hcm (H's center of mass).
  // Ih_xx = Ih_yy = 83/320 mh r²      From [Kane, Figure A23, pg. 369].
  // Ih_zz = 2/5 mh r²                 From [Kane, Figure A23, pg. 369].
  // Pedagogical note: H's inertia matrix about Ho (as compared with about Hcm)
  // is instead diag(2/5 mh r², 2/5 mh r², 2/5 mh r²).

  // The capsule's inertia about Ccm is calculated with the shift theorem
  // (parallel axis theorem) in terms of dH (distance between Hcm and Ccm) as
  // follows where the factor of 2 is due to the 2 half-spheres.
  // Note: The capsule's center of mass is coincident with Ccm.
  // Ixx = Ic_xx + 2 Ih_xx + 2 mh dH²
  // Izz = Ic_zz + 2 Ih_zz;

  // The previous algorithm for Ixx and Izz is algebraically manipulated to a
  // more efficient result by factoring on mh and mc and computing numbers as
  const T Ixx = mc * (L*L/12.0 + 0.25*r2) + mh * (0.51875*r2 + 2*dH*dH);
  const T Izz = (0.5*mc + 0.8*mh) * r2;  // Axial moment of inertia.

  return UnitInertia<T>::AxiallySymmetric(Izz, Ixx, unit_vector);
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidTetrahedronAboutVertex(
      const Vector3<T>& p, const Vector3<T>& q, const Vector3<T>& r) {
  // Note: Tetrahedon volume, mass, center of mass, and inertia formulas are
  // from the mass/inertia appendix in
  // [Mitiguy, 2017]: "Advanced Dynamics and Motion Simulation,
  //                   For professional engineers and scientists,"
  //                   Available at www.MotionGenesis.com
  const Vector3<T> q_plus_r = q + r;
  const T p_dot_pqr = p.dot(p + q_plus_r);
  const T q_dot_qr  = q.dot(q_plus_r);
  const T r_dot_r   = r.dot(r);
  const T scalar = 0.1 * (p_dot_pqr + q_dot_qr + r_dot_r);
  const Vector3<T> p_half = 0.5 * p;
  const Vector3<T> q_half = 0.5 * q;
  const Vector3<T> r_half = 0.5 * r;
  // TODO(Mitiguy) The G matrix below can be calculated more efficiently since
  //  G is symmetric (so we only need its upper or lower triangular part).
  const Matrix3<T> G = p * (p + q_half + r_half).transpose()
                     + q * (p_half + q + r_half).transpose()
                     + r * (p_half + q_half + r).transpose();
  const T Ixx = scalar - 0.1 * G(0, 0);
  const T Iyy = scalar - 0.1 * G(1, 1);
  const T Izz = scalar - 0.1 * G(2, 2);
  const T Ixy = -0.1 * G(0, 1);
  const T Ixz = -0.1 * G(0, 2);
  const T Iyz = -0.1 * G(1, 2);
  return UnitInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
}


}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::UnitInertia)
