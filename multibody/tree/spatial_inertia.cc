#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include "drake/common/fmt_eigen.h"

namespace drake {
namespace multibody {

namespace {

template <typename T>
const boolean<T> is_positive_finite(const T& value) {
  using std::isfinite;
  return isfinite(value) && value > 0;
}
}  // anonymous namespace

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
  if (!is_positive_finite(mass)) {
    const std::string error_message = fmt::format(
        "{}(): The mass of a particle is not positive and finite: {}.",
        __func__, mass);
    throw std::logic_error(error_message);
  }

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
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid box's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(lx) ||
      !is_positive_finite(ly) ||
      !is_positive_finite(lz)) {
    const std::string error_message = fmt::format(
        "{}(): One or more dimensions of a solid box is not positive and "
        "finite: ({}, {}, {}).", __func__, lx, ly, lz);
    throw std::logic_error(error_message);
  }

  const T volume = lx * ly * lz;
  const T mass = density * volume;
  return SpatialInertia<T>::SolidBoxWithMass(mass, lx, ly, lz);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidBoxWithMass(
    const T& mass, const T& lx, const T& ly, const T& lz) {
  if (!is_positive_finite(mass)) {
    const std::string error_message = fmt::format(
        "{}(): A solid box's mass is not positive and finite: {}.",
        __func__, mass);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(lx) ||
      !is_positive_finite(ly) ||
      !is_positive_finite(lz)) {
    const std::string error_message = fmt::format(
        "{}(): One or more dimensions of a solid box is not positive and "
        "finite: ({}, {}, {}).", __func__, lx, ly, lz);
    throw std::logic_error(error_message);
  }
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidBox(lx, ly, lz);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCubeWithDensity(
    const T& density, const T& length) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid cube's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): The length of a solid cube is not positive and finite: {}.",
        __func__, length);
    throw std::logic_error(error_message);
  }

  const T volume = length * length * length;
  const T mass = density * volume;
  return SolidCubeWithMass(mass, length);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCubeWithMass(
    const T& mass, const T& length) {
  if (!is_positive_finite(mass)) {
    const std::string error_message = fmt::format(
        "{}(): A solid cube's mass is not positive and finite: {}.",
        __func__, mass);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): The length of a solid cube is not positive and finite: {}.",
        __func__, length);
    throw std::logic_error(error_message);
  }

  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidCube(length);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCapsuleWithDensity(
    const T& density, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid capsule's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(radius) || !is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): A solid capsule's radius = {} or length = {} is not "
        "positive and finite.", __func__, radius, length);
    throw std::logic_error(error_message);
  }

  // Volume = π r² L + 4/3 π r³
  const T pi_r_squared = M_PI * radius * radius;
  const T volume = pi_r_squared * length + (4.0 / 3.0) * pi_r_squared * radius;
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B =
      UnitInertia<T>::SolidCapsule(radius, length, unit_vector);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithDensity(
    const T& density, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid cylinder's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(radius) || !is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): A solid cylinder's radius = {} or length = {} is not "
        "positive and finite.", __func__, radius, length);
    throw std::logic_error(error_message);
  }

  // Ensure ‖unit_vector‖ is within ≈ 5.5 bits of 1.0.
  // Note: 1E-14 ≈ 2^5.5 * std::numeric_limits<double>::epsilon();
  using std::abs;
  constexpr double kTolerance = 1E-14;
  if (abs(unit_vector.norm() - 1) > kTolerance) {
    throw std::logic_error(
        fmt::format("{}(): The unit_vector argument {} is not a unit vector.",
                    __func__, fmt_eigen(unit_vector.transpose())));
  }

  const T volume = M_PI * radius * radius * length;  // π r² l
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();

  // Note: Although a check is made that ‖unit_vector‖ ≈ 1, even if imperfect,
  // UnitInertia::SolidCylinder() allows for a near zero-vector due to code in
  // UnitInertia::AxiallySymmetric() which normalizes unit_vector before use.
  // TODO(Mitiguy) remove normalization in UnitInertia::AxiallySymmetric().
  const UnitInertia<T> G_BBo_B =
      UnitInertia<T>::SolidCylinder(radius, length, unit_vector);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithDensityAboutEnd(
    const T& density, const T& radius, const T& length,
    const Vector3<T>& unit_vector) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid cylinder's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(radius) || !is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): A solid cylinder's radius = {} or length = {} is not "
        "positive and finite.", __func__, radius, length);
    throw std::logic_error(error_message);
  }
  SpatialInertia<T> M_BBcm_B =
      SpatialInertia<T>::SolidCylinderWithDensity(
          density, radius, length, unit_vector);
  const Vector3<T> p_BcmBp_B = -0.5 * length * unit_vector;
  M_BBcm_B.ShiftInPlace(p_BcmBp_B);
  return M_BBcm_B;  // Due to shift, this actually returns M_BBp_B.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::ThinRodWithMass(
    const T& mass, const T& length, const Vector3<T>& unit_vector) {
  if (!is_positive_finite(mass) || !is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): A thin rod's mass = {} or length = {} is not positive and "
        "finite.", __func__, mass, length);
    throw std::logic_error(error_message);
  }

  // Ensure ‖unit_vector‖ is within ≈ 5.5 bits of 1.0.
  // Note: 1E-14 ≈ 2^5.5 * std::numeric_limits<double>::epsilon();
  using std::abs;
  constexpr double kTolerance = 1E-14;
  const T unit_vector_norm = unit_vector.norm();
  if (abs(unit_vector_norm - 1) > kTolerance) {
    throw std::logic_error(
        fmt::format("{}(): The unit_vector argument {} is not a unit vector.",
                    __func__, fmt_eigen(unit_vector.transpose())));
  }

  // Although a check is made that ‖unit_vector‖ ≈ 1, we normalize regardless.
  // Note: UnitInertia::ThinRod() calls UnitInertia::AxiallySymmetric() which
  // also normalizes unit_vector before use.
  // TODO(Mitiguy) remove normalization in UnitInertia::AxiallySymmetric().
  const Vector3<T> unit_vector_normalized = unit_vector / unit_vector_norm;
  const UnitInertia<T> G_BBcm_B =
      UnitInertia<T>::ThinRod(length, unit_vector_normalized);
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBcm_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::ThinRodWithMassAboutEnd(
    const T& mass, const T& length, const Vector3<T>& unit_vector) {
  if (!is_positive_finite(mass) || !is_positive_finite(length)) {
    const std::string error_message = fmt::format(
        "{}(): A thin rod's mass = {} or length = {} is not positive and "
        "finite.", __func__, mass, length);
    throw std::logic_error(error_message);
  }

  SpatialInertia<T> M_BBcm_B =
      SpatialInertia<T>::ThinRodWithMass(mass, length, unit_vector);
  const Vector3<T> p_BcmBp_B = -0.5 * length * unit_vector;
  M_BBcm_B.ShiftInPlace(p_BcmBp_B);
  return M_BBcm_B;  // Due to shift, this actually returns M_BBp_B.
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidEllipsoidWithDensity(
    const T& density, const T& a, const T& b, const T& c) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
       "{}(): A solid ellipsoid's density is not positive and finite: {}.",
       __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(a) ||
      !is_positive_finite(b) ||
      !is_positive_finite(c)) {
    const std::string error_message = fmt::format(
        "{}(): A solid ellipsoid's semi-axis a = {} or b = {} or c = {} "
        "is not positive and finite.", __func__, a, b, c);
    throw std::logic_error(error_message);
  }
  const T volume = (4.0 / 3.0) * M_PI * a * b * c;  // 4/3 π a b c
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidEllipsoid(a, b, c);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidSphereWithDensity(
    const T& density, const T& radius) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid sphere's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(radius)) {
    const std::string error_message = fmt::format(
        "{}(): A solid sphere's radius = {} is not positive and finite.",
        __func__, radius);
    throw std::logic_error(error_message);
  }
  const T volume = (4.0 / 3.0) * M_PI * radius * radius * radius;  // 4/3 π r³
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidSphere(radius);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::HollowSphereWithDensity(
    const T& area_density, const T& radius) {
  if (!is_positive_finite(area_density)) {
    const std::string error_message = fmt::format(
        "{}(): A hollow sphere's area density is not positive and finite: {}.",
        __func__, area_density);
    throw std::logic_error(error_message);
  }
  if (!is_positive_finite(radius)) {
    const std::string error_message = fmt::format(
        "{}(): A hollow sphere's radius = {} is not positive and finite.",
        __func__, radius);
    throw std::logic_error(error_message);
  }
  const T area = 4.0 * M_PI * radius * radius;  // 4 π r²
  const T mass = area_density * area;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::HollowSphere(radius);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidTetrahedronAboutPointWithDensity(
    const T& density, const Vector3<T>& p0, const Vector3<T>& p1,
    const Vector3<T>& p2, const Vector3<T>& p3) {
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid tetrahedron's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
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
  if (!is_positive_finite(density)) {
    const std::string error_message = fmt::format(
        "{}(): A solid tetrahedron's density is not positive and finite: {}.",
        __func__, density);
    throw std::logic_error(error_message);
  }
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
