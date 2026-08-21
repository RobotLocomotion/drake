#include "drake/multibody/tree/unit_inertia.h"

#include "drake/common/fmt_eigen.h"
#include "drake/common/text_logging.h"
#include "drake/math/unit_vector.h"

namespace drake {
namespace multibody {

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidEllipsoid(const T& a, const T& b,
                                              const T& c) {
  const T a2 = a * a;
  const T b2 = b * b;
  const T c2 = c * c;
  return UnitInertia<T>(0.2 * (b2 + c2), 0.2 * (a2 + c2), 0.2 * (a2 + b2));
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCylinder(const T& radius, const T& length,
                                             const Vector3<T>& unit_vector) {
  DRAKE_THROW_UNLESS(radius >= 0);
  DRAKE_THROW_UNLESS(length >= 0);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const T rsq = radius * radius;
  const T lsq = length * length;
  const T J = 0.5 * rsq;                // Axial moment of inertia J = ½ r².
  const T K = 0.25 * rsq + lsq / 12.0;  // Transverse moment K = ¼ r² + ¹⁄₁₂ l².
  return AxiallySymmetric(J, K, unit_vector);
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCylinderAboutEnd(
    const T& radius, const T& length, const Vector3<T>& unit_vector) {
  DRAKE_THROW_UNLESS(radius >= 0);
  DRAKE_THROW_UNLESS(length >= 0);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const T rsq = radius * radius;
  const T lsq = length * length;
  const T J = 0.5 * rsq;               // Axial moment of inertia J = ½ r².
  const T K = 0.25 * rsq + lsq / 3.0;  // Transverse moment K = ¼ r² + ⅓ l².
  return AxiallySymmetric(J, K, unit_vector);
}

template <typename T>
UnitInertia<T> UnitInertia<T>::AxiallySymmetric(const T& moment_parallel,
                                                const T& moment_perpendicular,
                                                const Vector3<T>& unit_vector) {
  const T& J = moment_parallel;
  const T& K = moment_perpendicular;
  DRAKE_THROW_UNLESS(moment_parallel >= 0.0);       // Ensure J ≥ 0.
  DRAKE_THROW_UNLESS(moment_perpendicular >= 0.0);  // Ensure K ≥ 0.

  // When the about-point Bp is Bcm, the moment of inertia triangle inequality
  // simplifies to J ≤ 2 K. If Bp is not Bcm, J ≤ 2 K is worth verifying because
  // K_Bp (perpendicular moment of inertia about Bp) relates to
  // K_Bcm (perpendicular moment of inertia about Bcm) as K_Bp = K_Bcm + dist²,
  // where dist is the distance between points Bp and Bcm.
  // This test has a tolerance of 5 bits (5 bits = 2^5 * epsilon).
  constexpr double two_plus_tiny =
      2.0 + 32 * std::numeric_limits<double>::epsilon();
  DRAKE_THROW_UNLESS(moment_parallel <= two_plus_tiny * moment_perpendicular);

  // TODO(Mitiguy) Consider a new UnitVector class to ensure the unit_vector
  //  argument to this function is either already normalized by the calling
  //  function (so a const reference to a UnitVector is passed) or if the
  //  calling function passes a Vector3, the Vector3 is automatically converted
  //  to a UnitVector (throwing an exception if the Vector3 contains NaN or
  //  infinite elements or its magnitude is incredulously small).
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);

  // Form B's unit inertia about a point Bp on B's symmetry axis,
  // expressed in the same frame E as the unit_vector is expressed.
  const Matrix3<T> G_BBp_E = K * Matrix3<T>::Identity() +
                             (J - K) * unit_vector * unit_vector.transpose();
  return UnitInertia<T>(G_BBp_E(0, 0), G_BBp_E(1, 1), G_BBp_E(2, 2),
                        G_BBp_E(0, 1), G_BBp_E(0, 2), G_BBp_E(1, 2));
}

template <typename T>
UnitInertia<T> UnitInertia<T>::StraightLine(const T& moment_perpendicular,
                                            const Vector3<T>& unit_vector) {
  DRAKE_THROW_UNLESS(moment_perpendicular > 0.0);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  return AxiallySymmetric(0.0, moment_perpendicular, unit_vector);
}

template <typename T>
UnitInertia<T> UnitInertia<T>::ThinRod(const T& length,
                                       const Vector3<T>& unit_vector) {
  DRAKE_THROW_UNLESS(length > 0.0);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  return StraightLine(length * length / 12.0, unit_vector);
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidBox(const T& Lx, const T& Ly, const T& Lz) {
  DRAKE_THROW_UNLESS(Lx >= 0);
  DRAKE_THROW_UNLESS(Ly >= 0);
  DRAKE_THROW_UNLESS(Lz >= 0);
  const T one_twelfth = 1.0 / 12.0;
  const T Lx2 = Lx * Lx, Ly2 = Ly * Ly, Lz2 = Lz * Lz;
  return UnitInertia(one_twelfth * (Ly2 + Lz2), one_twelfth * (Lx2 + Lz2),
                     one_twelfth * (Lx2 + Ly2));
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidCapsule(const T& radius, const T& length,
                                            const Vector3<T>& unit_vector) {
  DRAKE_THROW_UNLESS(radius >= 0);
  DRAKE_THROW_UNLESS(length >= 0);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);

  // A special case is required for radius = 0 because it creates a zero volume
  // capsule (and we divide by volume later on). No special case for length = 0
  // is needed because the capsule degenerates into a sphere (non-zero volume).
  if (radius == 0.0) {
    return UnitInertia<T>::ThinRod(length, unit_vector);
  }

  // The capsule is regarded as a cylinder C (with given length and radius) and
  // two half-spheres. The first half-sphere H is rigidly fixed to one end of
  // cylinder C so that the intersection between H and C forms a circle centered
  // at point Ho. Similarly, the other half-sphere is rigidly fixed to the other
  // end of cylinder C. The capsule's unit inertia about its center of mass is
  // calculated using tabulated expressions from [Kane] "Dynamics: Theory and
  // Applications," McGraw-Hill, New York, 1985, by T.R. Kane and D.A. Levinson,
  // available for electronic download from Cornell digital library.

  // Calculate vc (the volume of cylinder C) and vh (volume of half-sphere H).
  const T rsq = radius * radius;
  const T rcubed = rsq * radius;
  const T vc = M_PI * rsq * length;        // vc = π r² L
  const T vh = 2.0 / 3.0 * M_PI * rcubed;  // vh = 2/3 π r³

  // Denoting mc as the mass of cylinder C and mh as the mass of half-sphere H,
  // and knowing the capsule has a uniform density and the capsule's mass is 1
  // (for unit inertia), calculate mc and mh.
  const T v = vc + 2 * vh;  // Volume of capsule.
  const T mc = vc / v;      // Mass in the cylinder (relates to volume).
  const T mh = vh / v;      // Mass in each half-sphere (relates to volume).

  // The distance dH between Hcm (half-sphere H's center of mass) and Ccm
  // (cylinder C's center of mass) is given in [Kane, Figure A23, pg. 369] as
  // dH = ⅜ radius + ½ length.
  const T dH = 0.375 * radius + 0.5 * length;

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
  const T lsq = length * length;
  const T Ixx =
      mc * (lsq / 12.0 + 0.25 * rsq) + mh * (0.51875 * rsq + 2 * dH * dH);
  const T Izz = (0.5 * mc + 0.8 * mh) * rsq;  // Axial moment of inertia.
  return UnitInertia<T>::AxiallySymmetric(Izz, Ixx, unit_vector);
}

namespace {
// Returns a 3x3 matrix whose upper-triangular part contains the outer product
// of vector a * vector b [i.e., a * b.transpose()] and whose lower-triangular
// part is uninitialized.
// @param[in] a vector expressed in a frame E.
// @param[in] b vector expressed in a frame E.
// @note This function is an efficient way to calculate outer-products that
//   contribute via a sum to a symmetric matrix.
template <typename T>
Matrix3<T> UpperTriangularOuterProduct(const Eigen::Ref<const Vector3<T>>& a,
                                       const Eigen::Ref<const Vector3<T>>& b) {
  Matrix3<T> M;
  M(0, 0) = a(0) * b(0);
  M(0, 1) = a(0) * b(1);
  M(0, 2) = a(0) * b(2);
  M(1, 1) = a(1) * b(1);
  M(1, 2) = a(1) * b(2);
  M(2, 2) = a(2) * b(2);
  return M;
}
}  // namespace

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidTetrahedronAboutPoint(
    const Vector3<T>& p0, const Vector3<T>& p1, const Vector3<T>& p2,
    const Vector3<T>& p3) {
  // This method calculates a tetrahedron B's unit inertia G_BA about a point A
  // by first calculating G_BB0 (B's unit inertia about vertex B0 of B).
  // To calculate G_BB0, 3 new position vectors are formed, namely the position
  // vectors from vertex B0 to vertices B1, B2, B3 (B's other three vertices).
  const Vector3<T> p_B0B1 = p1 - p0;  // Position from vertex B0 to vertex B1.
  const Vector3<T> p_B0B2 = p2 - p0;  // Position from vertex B0 to vertex B2.
  const Vector3<T> p_B0B3 = p3 - p0;  // Position from vertex B0 to vertex B3.
  const UnitInertia<T> G_BB0 =
      UnitInertia<T>::SolidTetrahedronAboutVertex(p_B0B1, p_B0B2, p_B0B3);

  // Shift unit inertia from about point B0 to about point A.
  const Vector3<T> p_B0Bcm = 0.25 * (p_B0B1 + p_B0B2 + p_B0B3);
  const Vector3<T>& p_AB0 = p0;  // Alias with monogram notation to clarify.
  const Vector3<T> p_ABcm = p_AB0 + p_B0Bcm;
  const RotationalInertia<T> I_BA = G_BB0.ShiftToThenAwayFromCenterOfMass(
      /* mass = */ 1, p_B0Bcm, p_ABcm);
  return UnitInertia<T>(I_BA);  // Returns G_BA (B's unit inertia about A).
}

template <typename T>
UnitInertia<T> UnitInertia<T>::SolidTetrahedronAboutVertex(
    const Vector3<T>& p1, const Vector3<T>& p2, const Vector3<T>& p3) {
  // This method calculates G_BB0 (a tetrahedron B's unit inertia about a
  // vertex B0 of B) using the position vectors from vertex B0 to B's three
  // other vertices, herein named P, Q, R to be consistent with the
  // tetrahedron inertia formulas from the mass/inertia appendix in
  // [Mitiguy, 2017]: "Advanced Dynamics and Motion Simulation,
  //                   For professional engineers and scientists,"
  const Vector3<T>& p = p1;  // Position from vertex B0 to vertex P.
  const Vector3<T>& q = p2;  // Position from vertex B0 to vertex Q.
  const Vector3<T>& r = p3;  // Position from vertex B0 to vertex R.
  const Vector3<T> q_plus_r = q + r;
  const T p_dot_pqr = p.dot(p + q_plus_r);
  const T q_dot_qr = q.dot(q_plus_r);
  const T r_dot_r = r.dot(r);
  const T scalar = 0.1 * (p_dot_pqr + q_dot_qr + r_dot_r);
  const Vector3<T> p_half = 0.5 * p;
  const Vector3<T> q_half = 0.5 * q;
  const Vector3<T> r_half = 0.5 * r;
  const Matrix3<T> G = UpperTriangularOuterProduct<T>(p, p + q_half + r_half) +
                       UpperTriangularOuterProduct<T>(q, p_half + q + r_half) +
                       UpperTriangularOuterProduct<T>(r, p_half + q_half + r);
  const T Ixx = scalar - 0.1 * G(0, 0);
  const T Iyy = scalar - 0.1 * G(1, 1);
  const T Izz = scalar - 0.1 * G(2, 2);
  const T Ixy = -0.1 * G(0, 1);
  const T Ixz = -0.1 * G(0, 2);
  const T Iyz = -0.1 * G(1, 2);
  return UnitInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);
}

template <typename T>
std::pair<Vector3<double>, math::RotationMatrix<double>>
UnitInertia<T>::CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
    double inertia_shape_factor) const {
  DRAKE_THROW_UNLESS(inertia_shape_factor > 0 && inertia_shape_factor <= 1);
  // The formulas below are derived for a shape D whose principal unit moments
  // of inertia Gmin, Gmed, Gmax about Dcm (D's center of mass) have the form:,
  // Gmin = inertia_shape_factor * (b² + c²)
  // Gmed = inertia_shape_factor * (a² + c²)
  // Gmax = inertia_shape_factor * (a² + b²)
  // e.g., where a, b, c are ½ lengths of boxes or semi-axes of ellipsoids.
  // Casting these equations into matrix form, gives
  // ⌈0  1  1⌉ ⌈a²⌉   ⌈Gmin ⌉
  // |1  0  1⌉ |b²⌉ = |Gmed ⌉ / inertia_shape_factor
  // ⌊1  1  0⌋ ⌊c²⌋   ⌊Gmax ⌉
  // Inverting the coefficient matrix and solving for a², b², c² leads to
  // ⌈a²⌉   ⌈-1  1  1⌉ ⌈Gmin ⌉
  // |b²⌉ = | 1 -1  1⌉ |Gmed ⌉ * 0.5 / inertia_shape_factor
  // ⌊c²⌋   ⌊ 1  1 -1⌋ ⌊Gmax ⌉
  // Since Gmin ≤ Gmed ≤ Gmax, we can deduce a² ≥ b² ≥ c², so we designate
  // lmax² = a² = 0.5 / inertia_shape_factor * (Gmed + Gmax - Gmin)
  // lmed² = b² = 0.5 / inertia_shape_factor * (Gmin + Gmax - Gmed)
  // lmin² = c² = 0.5 / inertia_shape_factor * (Gmin + Gmed - Gmax)

  // Form principal moments Gmoments and principal axes stored in R_EA.
  auto [Gmoments, R_EA] = this->CalcPrincipalMomentsAndAxesOfInertia();
  const double Gmin = Gmoments(0);
  const double Gmed = Gmoments(1);
  const double Gmax = Gmoments(2);
  DRAKE_ASSERT(Gmin <= Gmed && Gmed <= Gmax);
  const double coef = 0.5 / inertia_shape_factor;
  using std::max;  // Avoid round-off issues that result in e.g., sqrt(-1E-15).
  const double lmax_squared = max(coef * (Gmed + Gmax - Gmin), 0.0);
  const double lmed_squared = max(coef * (Gmin + Gmax - Gmed), 0.0);
  const double lmin_squared = max(coef * (Gmin + Gmed - Gmax), 0.0);
  const double lmax = std::sqrt(lmax_squared);
  const double lmed = std::sqrt(lmed_squared);
  const double lmin = std::sqrt(lmin_squared);
  return std::pair(Vector3<double>(lmax, lmed, lmin), R_EA);
}

template <typename T>
std::string to_string(const UnitInertia<T>& I) {
  return fmt::to_string(static_cast<const RotationalInertia<T>&>(I));
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    (static_cast<std::string (*)(const UnitInertia<T>&)>(&to_string)));

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::UnitInertia);
