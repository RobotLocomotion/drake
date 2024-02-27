#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include "drake/common/fmt_eigen.h"
#include "drake/math/unit_vector.h"

namespace drake {
namespace multibody {

namespace {

template <typename T>
const boolean<T> is_positive_finite(const T& value) {
  using std::isfinite;
  return isfinite(value) && value > 0;
}

template <typename T>
const boolean<T> is_nonnegative_finite(const T& value) {
  using std::isfinite;
  return isfinite(value) && value >= 0;
}

template<typename T>
void ThrowUnlessValueIsPositiveFinite(const T& value,
    std::string_view value_name, std::string_view function_name) {
  if (!is_positive_finite(value)) {
    DRAKE_DEMAND(!value_name.empty());
    DRAKE_DEMAND(!function_name.empty());
    const std::string error_message = fmt::format(
    "{}(): {} is not positive and finite: {}.",
    function_name, value_name, value);
  throw std::logic_error(error_message);
  }
}

}  // namespace

template <typename T>
SpatialInertia<T> SpatialInertia<T>::MakeFromCentralInertia(const T& mass,
    const Vector3<T>& p_PScm_E, const RotationalInertia<T>& I_SScm_E) {
  UnitInertia<T> G_SScm_E;
  G_SScm_E.SetFromRotationalInertia(I_SScm_E, mass);
  // The next line checks that M_SScm_E is physically valid.
  const SpatialInertia<T> M_SScm_E(mass, Vector3<T>::Zero(), G_SScm_E);
  return M_SScm_E.ShiftFromCenterOfMass(-p_PScm_E);  // Shift to M_SP_E.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::MakeUnitary() {
  const T mass = 1;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();  // Position from Bo to Bcm.
  const UnitInertia<T> G_BBo_B(/* Ixx = */ 1, /* Iyy = */ 1, /* Izz = */ 1);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::PointMass(
    const T& mass, const Vector3<T>& position) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);

  // Upgrade to monogram notation: position is the position vector from
  // point P to particle Q expressed in a frame B.
  const Vector3<T> p_PQ_B = position;

  // Form particle Q's unit inertia about point P, expressed in frame B.
  const UnitInertia<T> G_QP_B = UnitInertia<T>::PointMass(p_PQ_B);

  // Return particle Q's spatial inertia about point P, expressed in frame B.
  return SpatialInertia<T>(mass, p_PQ_B, G_QP_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidBoxWithDensity(
    const T& density, const T& lx, const T& ly, const T& lz) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(lx, "x-length", __func__);
  ThrowUnlessValueIsPositiveFinite(ly, "y-length", __func__);
  ThrowUnlessValueIsPositiveFinite(lz, "z-length", __func__);
  const T volume = lx * ly * lz;
  const T mass = density * volume;
  return SpatialInertia<T>::SolidBoxWithMass(mass, lx, ly, lz);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidBoxWithMass(
    const T& mass, const T& lx, const T& ly, const T& lz) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(lx, "x-length", __func__);
  ThrowUnlessValueIsPositiveFinite(ly, "y-length", __func__);
  ThrowUnlessValueIsPositiveFinite(lz, "z-length", __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidBox(lx, ly, lz);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCubeWithDensity(
    const T& density, const T& length) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  const T volume = length * length * length;
  const T mass = density * volume;
  return SolidCubeWithMass(mass, length);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCubeWithMass(
    const T& mass, const T& length) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidCube(length);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCapsuleWithDensity(
    const T& density, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);

  // Volume = π r² L + 4/3 π r³
  const T pi_r_squared = M_PI * radius * radius;
  const T volume = pi_r_squared * length + (4.0 / 3.0) * pi_r_squared * radius;
  const T mass = density * volume;
  return SolidCapsuleWithMass(mass, radius, length, unit_vector);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCapsuleWithMass(
    const T& mass, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B =
      UnitInertia<T>::SolidCapsule(radius, length, unit_vector);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithDensity(
    const T& density, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const T volume = M_PI * radius * radius * length;  // π r² l
  const T mass = density * volume;
  return SolidCylinderWithMass(mass, radius, length, unit_vector);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithMass(
    const T& mass, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B =
      UnitInertia<T>::SolidCylinder(radius, length, unit_vector);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithDensityAboutEnd(
    const T& density, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const T volume = M_PI * radius * radius * length;  // π r² l
  const T mass = density * volume;
  return SolidCylinderWithMassAboutEnd(mass, radius, length, unit_vector);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithMassAboutEnd(
    const T& mass, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const Vector3<T> p_BpBcm_B = 0.5 * length * unit_vector;
  const UnitInertia<T> G_BBp_B =
      UnitInertia<T>::SolidCylinderAboutEnd(radius, length, unit_vector);
  return SpatialInertia(mass, p_BpBcm_B, G_BBp_B);  // Returns M_BBp_B.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::ThinRodWithMass(
    const T& mass, const T& length, const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  const UnitInertia<T> G_BBcm_B =
      UnitInertia<T>::ThinRod(length, unit_vector);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBcm_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::ThinRodWithMassAboutEnd(
    const T& mass, const T& length, const Vector3<T>& unit_vector) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(length, "length", __func__);
  math::internal::ThrowIfNotUnitVector(unit_vector, __func__);
  SpatialInertia<T> M_BBcm_B =
      SpatialInertia<T>::ThinRodWithMass(mass, length, unit_vector);
  const Vector3<T> p_BcmBp_B = -0.5 * length * unit_vector;
  M_BBcm_B.ShiftFromCenterOfMassInPlace(p_BcmBp_B);
  return M_BBcm_B;  // Due to shift, this actually returns M_BBp_B.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidEllipsoidWithDensity(
    const T& density, const T& a, const T& b, const T& c) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(a, "semi-axis a", __func__);
  ThrowUnlessValueIsPositiveFinite(b, "semi-axis b", __func__);
  ThrowUnlessValueIsPositiveFinite(c, "semi-axis c", __func__);
  const T volume = (4.0 / 3.0) * M_PI * a * b * c;  // 4/3 π a b c
  const T mass = density * volume;
  return SolidEllipsoidWithMass(mass, a, b, c);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidEllipsoidWithMass(
    const T& mass, const T& a, const T& b, const T& c) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(a, "semi-axis a", __func__);
  ThrowUnlessValueIsPositiveFinite(b, "semi-axis b", __func__);
  ThrowUnlessValueIsPositiveFinite(c, "semi-axis c", __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidEllipsoid(a, b, c);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidSphereWithDensity(
    const T& density, const T& radius) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  const T volume = (4.0 / 3.0) * M_PI * radius * radius * radius;  // 4/3 π r³
  const T mass = density * volume;
  return SolidSphereWithMass(mass, radius);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidSphereWithMass(
    const T& mass, const T& radius) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidSphere(radius);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::HollowSphereWithDensity(
    const T& area_density, const T& radius) {
  ThrowUnlessValueIsPositiveFinite(area_density, "area_density", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  const T area = 4.0 * M_PI * radius * radius;  // 4 π r²
  const T mass = area_density * area;
  return HollowSphereWithMass(mass, radius);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::HollowSphereWithMass(
    const T& mass, const T& radius) {
  ThrowUnlessValueIsPositiveFinite(mass, "mass", __func__);
  ThrowUnlessValueIsPositiveFinite(radius, "radius", __func__);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::HollowSphere(radius);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidTetrahedronAboutPointWithDensity(
    const T& density, const Vector3<T>& p0, const Vector3<T>& p1,
    const Vector3<T>& p2, const Vector3<T>& p3) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);

  // This method calculates a tetrahedron B's spatial inertia M_BA about a
  // point A by forming 3 new position vectors, namely the position vectors
  // from B's vertex B0 to vertices B1, B2, B3 (B's other three vertices).
  const Vector3<T> p_B0B1 = p1 - p0;  // Position from vertex B0 to vertex B1.
  const Vector3<T> p_B0B2 = p2 - p0;  // Position from vertex B0 to vertex B2.
  const Vector3<T> p_B0B3 = p3 - p0;  // Position from vertex B0 to vertex B3.

  // Form B's spatial inertia about vertex B0 and then shifts to point A.
  SpatialInertia<T> M_BB0 =
      SpatialInertia<T>::SolidTetrahedronAboutVertexWithDensity(
          density, p_B0B1, p_B0B2, p_B0B3);
  const Vector3<T>& p_AB0 = p0;  // Alias for position from point A to B0.
  M_BB0.ShiftInPlace(-p_AB0);
  return M_BB0;  // Since M_BB0 was shifted, this actually returns M_BA.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidTetrahedronAboutVertexWithDensity(
    const T& density, const Vector3<T>& p1, const Vector3<T>& p2,
    const Vector3<T>& p3) {
  ThrowUnlessValueIsPositiveFinite(density, "density", __func__);
  using std::abs;
  const T volume = (1.0 / 6.0) * abs(p1.cross(p2).dot(p3));
  const T mass = density * volume;

  // Get position from tetrahedron B's vertex at Bo to Bcm (B's center of mass).
  const Vector3<T> p_BoBcm = 0.25 * (p1 + p2 + p3);
  const UnitInertia<T> G_BBo =
      UnitInertia<T>::SolidTetrahedronAboutVertex(p1, p2, p3);
  return SpatialInertia<T>(mass, p_BoBcm, G_BBo);
}

template <typename T>
boolean<T> SpatialInertia<T>::IsPhysicallyValid() const {
  // This spatial inertia is not physically valid if the mass is negative or
  // non-finite or the center of mass or unit inertia matrix have NaN elements.
  boolean<T> ret_value = is_nonnegative_finite(mass_);
  if (ret_value) {
    // Form a rotational inertia about the body's center of mass and then use
    // the well-documented tests in RotationalInertia to test validity.
    const SpatialInertia<T> M_SScm_E = ShiftToCenterOfMass();
    const UnitInertia<T>& G_SScm_E = M_SScm_E.get_unit_inertia();
    const RotationalInertia<T> I_SScm_E =
        G_SScm_E.MultiplyByScalarSkipValidityCheck(mass_);
    ret_value = I_SScm_E.CouldBePhysicallyValid();
  }
  return ret_value;
}

template <typename T>
void SpatialInertia<T>::ThrowNotPhysicallyValid() const {
  std::string error_message = fmt::format(
          "Spatial inertia fails SpatialInertia::IsPhysicallyValid().");
  const T& mass = get_mass();
  if (!is_positive_finite(mass)) {
      error_message += fmt::format(
          "\nmass = {} is not positive and finite.\n", mass);
  } else {
    error_message += fmt::format("{}", *this);
    WriteExtraCentralInertiaProperties(&error_message);
  }
  throw std::runtime_error(error_message);
}

template <typename T>
SpatialInertia<T>& SpatialInertia<T>::operator+=(
    const SpatialInertia<T>& M_BP_E) {
  const T total_mass = get_mass() + M_BP_E.get_mass();
  if (total_mass != 0) {
    p_PScm_E_ = (CalcComMoment() + M_BP_E.CalcComMoment()) / total_mass;
    G_SP_E_.SetFromRotationalInertia(
        CalcRotationalInertia() + M_BP_E.CalcRotationalInertia(), total_mass);
  } else {
    // Compose the spatial inertias of two massless bodies in the limit when
    // the two bodies have the same mass. In this limit, p_PScm_E_ and G_SP_E_
    // are the arithmetic mean of the constituent COMs and unit inertias.
    p_PScm_E_ = 0.5 * (get_com() + M_BP_E.get_com());
    G_SP_E_.SetFromRotationalInertia(
        get_unit_inertia() + M_BP_E.get_unit_inertia(), 2.0);
  }
  mass_ = total_mass;
  return *this;
}

template <typename T>
SpatialInertia<T>& SpatialInertia<T>::ShiftFromCenterOfMassInPlace(
    const Vector3<T>& p_ScmP_E) {
  DRAKE_ASSERT(p_PScm_E_ == Vector3<T>::Zero());
  G_SP_E_.ShiftFromCenterOfMassInPlace(p_ScmP_E);
  p_PScm_E_ = -p_ScmP_E;
  return *this;  // On entry, `this` is M_SScm_E. On return, `this` is M_SP_E.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::ShiftFromCenterOfMass(
    const Vector3<T>& p_ScmP_E) const {
  return SpatialInertia(*this).ShiftFromCenterOfMassInPlace(p_ScmP_E);
}

template <typename T>
SpatialInertia<T>& SpatialInertia<T>::ShiftToCenterOfMassInPlace() {
  G_SP_E_.ShiftToCenterOfMassInPlace(p_PScm_E_);
  p_PScm_E_ = Vector3<T>::Zero();
  return *this;  // On entry, `this` is M_SP_E. On return, `this` is M_SScm_E.
}

template <typename T>
SpatialInertia<T>& SpatialInertia<T>::ShiftInPlace(const Vector3<T>& p_PQ_E) {
  const Vector3<T> p_QScm_E = p_PScm_E_ - p_PQ_E;
  // The following two lines apply the parallel axis theorem (in place) so that:
  //  G_SQ = G_SP + px_QScm² - px_PScm²
  G_SP_E_.ShiftFromCenterOfMassInPlace(p_QScm_E);
  G_SP_E_.ShiftToCenterOfMassInPlace(p_PScm_E_);
  p_PScm_E_ = p_QScm_E;
  // Note: It would be a implementation bug if a shift starts with a valid
  // spatial inertia and the shift produces an invalid spatial inertia.
  // Hence, no need to use DRAKE_ASSERT_VOID(CheckInvariants()).
  return *this;
}

template <typename T>
SpatialForce<T> SpatialInertia<T>::operator*(
    const SpatialAcceleration<T>& A_WB_E) const {
  const Vector3<T>& alpha_WB_E = A_WB_E.rotational();
  const Vector3<T>& a_WBo_E = A_WB_E.translational();
  const Vector3<T>& mp_BoBcm_E = CalcComMoment();  // = m * p_BoBcm
  // Return (see class's documentation):
  // ⌈ tau_Bo_E ⌉   ⌈    I_Bo_E     | m * p_BoBcm× ⌉   ⌈ alpha_WB_E ⌉
  // |          | = |               |              | * |            |
  // ⌊  f_Bo_E  ⌋   ⌊ -m * p_BoBcm× |   m * Id     ⌋   ⌊  a_WBo_E   ⌋
  return SpatialForce<T>(
      /* rotational */
      CalcRotationalInertia() * alpha_WB_E + mp_BoBcm_E.cross(a_WBo_E),
      /* translational: notice the order of the cross product is the reversed
       * of the documentation above and thus no minus sign is needed. */
      alpha_WB_E.cross(mp_BoBcm_E) + get_mass() * a_WBo_E);
}

template <typename T>
SpatialMomentum<T> SpatialInertia<T>::operator*(
    const SpatialVelocity<T>& V_WBp_E) const {
  const Vector3<T>& w_WB_E = V_WBp_E.rotational();
  const Vector3<T>& v_WP_E = V_WBp_E.translational();
  const Vector3<T>& mp_BoBcm_E = CalcComMoment();  // = m * p_BoBcm
  // Return (see class's documentation):
  // ⌈ h_WB  ⌉   ⌈     I_Bp      | m * p_BoBcm× ⌉   ⌈ w_WB ⌉
  // |       | = |               |              | * |      |
  // ⌊ l_WBp ⌋   ⌊ -m * p_BoBcm× |   m * Id     ⌋   ⌊ v_WP ⌋
  return SpatialMomentum<T>(
      // Rotational
      CalcRotationalInertia() * w_WB_E + mp_BoBcm_E.cross(v_WP_E),
      // Translational: notice the order of the cross product is the reversed
      // of the documentation above and thus no minus sign is needed.
      w_WB_E.cross(mp_BoBcm_E) + get_mass() * v_WP_E);
}

template <typename T>
std::pair<Vector3<double>, drake::math::RigidTransform<double>>
SpatialInertia<T>::CalcPrincipalHalfLengthsAndPoseForEquivalentShape(
    double inertia_shape_factor) const {
  // Form M_SScm_E by shifting `this` spatial inertia from about-point P to Scm.
  const SpatialInertia<T> M_SScm_E = ShiftToCenterOfMass();

  // Form the principal semi-diameters (half-lengths) and rotation matrix R_EA
  // that contains the associated principal axes directions Ax, Ay, Az.
  const UnitInertia<T>& G_SScm_E = M_SScm_E.get_unit_inertia();
  const auto [abc, R_EA] =
      G_SScm_E.CalcPrincipalHalfLengthsAndAxesForEquivalentShape(
          inertia_shape_factor);

  // Since R_EA is of type double and X_EA must also be of type double,
  // create a position vector from P to Scm that is of type double.
  const Vector3<T>& p_PScm_E = get_com();
  const double xcm = ExtractDoubleOrThrow(p_PScm_E(0));
  const double ycm = ExtractDoubleOrThrow(p_PScm_E(1));
  const double zcm = ExtractDoubleOrThrow(p_PScm_E(2));
  const Vector3<double> p_PAo_E(xcm, ycm, zcm);  // Note: Point Ao is at Scm.
  const drake::math::RigidTransform<double> X_EA(R_EA, p_PAo_E);
  return std::pair(abc, X_EA);
}

// The derivation of the formula below starts with calculations documented in
// UnitInertia<T>::CalcPrincipalHalfLengthsAndAxesForEquivalentShape() of
// lmax, lmed, lmin (½-lengths of sides of the minimum bounding box) that depend
// on Gmax, Gmed, Gmin (the maximum, medium, minimum central principal moments
// of the unit inertia calculated via shifting `this` to the center of mass).
// 1. lmax² = 0.5 / inertia_shape_factor * (Gmed + Gmax - Gmin)
//    lmed² = 0.5 / inertia_shape_factor * (Gmin + Gmax - Gmed)
//    lmin² = 0.5 / inertia_shape_factor * (Gmin + Gmed - Gmax)
// 2. lmax² + lmed² + lmin² = 0.5 / inertia_shape_factor * (Gmin + Gmed + Gmax)
// 3. Gmin + Gmed + Gmax = Trace(unit_inertia) is invariant meaning it does not
//    matter if unit_inertia has zero or non-zero products of inertia, i.e.,
//    Gxx + Gyy + Gzz = Gmin + Gmed + Gmax is invariant to the unit inertia's
//    expressed-in frame. One proof of this invarience is that the trace of a
//    symmetric matrix is invariant under an orthogonal transformation, e.g.,
//    for the rotation matrix R_AB and using the trace cyclic property
//    Trace(R_AB * unit_inertia * R_BA) = Trace(unit_inertia * R_BA * R_AB)
//                                      = Trace(unit_inertia * identity_matrix)
//                                      = Trace(unit_inertia)
// 4. space_diagonal² = (2 lmax)² + (2 lmed)² + (2 lmin)²
//                    = 4 (lmax² + lmed² + lmin²)
//                    = 4 * 0.5 / inertia_shape_factor * (Gxx + Gyy + Gzz)
//                    = 4 * 0.5 / inertia_shape_factor * Trace(unit_inertia)
//                    = 2 / inertia_shape_factor * Trace(unit_inertia)
// 5. The largest inertia shape_factor is 1.0, which corresponds to a massless
//    box with particles at its vertices. Hence the minimum space diagonal is
//    space_diagonal² = 2 * Trace(unit_inertia) or
//    space_diagonal = √(2 * Trace(unit_inertia))
// This space_diagonal formula is useful when template type <T> is symbolic or
// Autodiff. Also, this formula is more efficient than calculating the length of
// the space-diagonal via the minimum bounding box (which uses an inherently
// numerical eigenvalue process).
template <typename T>
T SpatialInertia<T>::CalcMinimumPhysicalLength() const {
  const SpatialInertia<T> M_SScm_E = ShiftToCenterOfMass();
  const UnitInertia<T>& G_SScm_E = M_SScm_E.get_unit_inertia();
  using std::sqrt;
  return sqrt(2.0 * G_SScm_E.Trace());
}

template <typename T>
void SpatialInertia<T>::WriteExtraCentralInertiaProperties(
    std::string* message) const {
  DRAKE_DEMAND(message != nullptr);
  // Form M_BBcm by shifting `this` from about-point P to Scm. Use unit inertia
  // G_BBcm to form I_BBcm (rotational inertia about Bcm) without validity
  // checks like IsPhysicallyValid() so this function works for error messages.
  const SpatialInertia<T> M_BBcm = ShiftToCenterOfMass();
  const UnitInertia<T>& G_BBcm = M_BBcm.get_unit_inertia();
  const RotationalInertia<T> I_BBcm =
      G_BBcm.MultiplyByScalarSkipValidityCheck(get_mass());

  // If `this` about-point P is not Bcm, write B's rotational inertia about Bcm.
  const Vector3<T>& p_PBcm = get_com();
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

  // TODO(jwnimmer-tri) Rewrite this to use fmt to our advantage.
  if constexpr (scalar_predicate<T>::is_bool) {
    out << "\n"
        << fmt::format(" mass = {}\n", mass)
        << fmt::format(" Center of mass = [{}  {}  {}]\n", x, y, z);
  } else {
    // Print symbolic results.
    out << " mass = " << mass << "\n"
        << fmt::format(" Center of mass = {}\n", fmt_eigen(p_PBcm.transpose()));
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
