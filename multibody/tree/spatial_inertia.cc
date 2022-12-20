#include "drake/multibody/tree/spatial_inertia.h"

#include <string>

#include <fmt/format.h>

namespace drake {
namespace multibody {

template <typename T>
SpatialInertia<T> SpatialInertia<T>::MakeUnitary() {
  const T mass = 1;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();  // Position from Bo to Bcm.
  const UnitInertia<T> G_BBo_B(/* Ixx = */ 1, /* Iyy = */ 1, /* Izz = */ 1);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidBoxWithDensity(const T& density,
    const T& lx, const T& ly, const T& lz) {
  // Ensure lx, ly, lz are positive.
  if (lx <= 0 || ly <= 0 || lz <= 0) {
    std::string error_message = fmt::format("{}(): A solid box's "
      "length, width, or height is negative or zero.", __func__);
    throw std::logic_error(error_message);
  }
  const T volume = lx * ly * lz;
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidBox(lx, ly, lz);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCapsuleWithDensity(const T& density,
    const T& r, const T& l, const Vector3<T>& unit_vector) {
  // Ensure r and l are positive.
  if (r <= 0 || l <= 0) {
    std::string error_message = fmt::format("{}(): A solid capsule's "
      "radius or length is negative or zero.", __func__);
    throw std::logic_error(error_message);
  }
  // Note: Although a check is made that ‖unit_vector‖ ≈ 1, even if imperfect,
  // UnitInertia::SolidCapsule() normalizes unit_vector before its use.
  using std::abs;
  constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();
  if (abs(unit_vector.norm() - 1) > kTolerance) {
    // ‖unit_vector‖ is not within 6 bits of 1.0 (2^6 = 64).
    std::string error_message = fmt::format("{}(): The unit_vector argument "
      "is not a unit vector. Consider normalizing it.", __func__);
    throw std::logic_error(error_message);
  }
  const T pi_r_squared = M_PI * r * r;
  const T volume = 4.0 / 3.0 * pi_r_squared * r +  pi_r_squared * l;
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B =
      UnitInertia<T>::SolidCapsule(r, l, unit_vector);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidCylinderWithDensity(const T& density,
    const T& r, const T& l, const Vector3<T>& unit_vector) {
  // Ensure r and l are positive.
  if (r <= 0 || l <= 0) {
    std::string error_message = fmt::format("{}(): A solid cylinder's "
      "radius or length is negative or zero.", __func__);
    throw std::logic_error(error_message);
  }
  // Note: Although a check is made that ‖unit_vector‖ ≈ 1, even if imperfect,
  // UnitInertia::SolidCylinder() normalizes unit_vector before its use.
  using std::abs;
  constexpr double kTolerance = 64 * std::numeric_limits<double>::epsilon();
  if (abs(unit_vector.norm() - 1) > kTolerance) {
    // ‖unit_vector‖ is not within 6 bits of 1.0 (2^6 = 64).
    std::string error_message = fmt::format("{}(): The unit_vector argument "
      "is not a unit vector. Consider normalizing it.", __func__);
    throw std::logic_error(error_message);
  }
  const T volume = M_PI * r * r * l;
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B =
      UnitInertia<T>::SolidCylinder(r, l, unit_vector);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidEllipsoidWithDensity(const T& density,
    const T& a, const T& b, const T& c) {
  // Ensure a, b, c are positive.
  if (a <= 0 || b <= 0 || c <= 0) {
    std::string error_message = fmt::format("{}(): A solid ellipsoid's "
      "semi-diameter is negative or zero.", __func__);
    throw std::logic_error(error_message);
  }
  const T volume = 4.0 * M_PI / 3.0 * a * b * c;
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidEllipsoid(a, b, c);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidSphereWithDensity(const T& density,
    const T& r) {
  // Ensure r is positive.
  if (r <= 0) {
    std::string error_message = fmt::format("{}(): A solid sphere's "
      "radius is negative or zero.", __func__);
    throw std::logic_error(error_message);
  }
  const T volume = 4.0 * M_PI / 3.0 * r * r * r;
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = Vector3<T>::Zero();
  const UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidSphere(r);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
SpatialInertia<T> SpatialInertia<T>::SolidTetrahedronAboutVertexWithDensity(
    const T& density,
    const Vector3<T>& p, const Vector3<T>& q, const Vector3<T>& r) {
  const T volume = 1.0 / 6.0 * p.cross(q).dot(r);

  // Ensure volume is sufficiently far from zero. For a given ‖𝐩‖, ‖𝐪‖, ‖𝐫‖,
  // the maximum volume = ‖𝐩‖ ‖𝐪‖ ‖𝐫‖ / 6 occurs when 𝐩, 𝐪, 𝐫 are orthogonal.
  constexpr double kTolerance = 0.25 * std::numeric_limits<double>::epsilon();
  if (volume * volume <= kTolerance * p.dot(p) * q.dot(q) * r.dot(r)) {
    std::string error_message = fmt::format("{}(): A solid tetrahedron's "
      "volume is zero or near zero.", __func__);
    throw std::logic_error(error_message);
  }
  // Note: Tetrahedon volume, mass, center of mass, and inertia formulas are
  // from the mass/inertia appendix in
  // [Mitiguy, 2017]: "Advanced Dynamics and Motion Simulation,
  //                   For professional engineers and scientists,"
  //                   Available at www.MotionGenesis.com
  const T mass = density * volume;
  const Vector3<T> p_BoBcm_B = 0.25 * (p + q + r);
  UnitInertia<T> G_BBo_B = UnitInertia<T>::SolidTetrahedronAboutVertex(p, q, r);
  return SpatialInertia<T>(mass, p_BoBcm_B, G_BBo_B);
}

template <typename T>
void SpatialInertia<T>::ThrowNotPhysicallyValid() const {
  std::string error_message = fmt::format(
          "Spatial inertia fails SpatialInertia::IsPhysicallyValid().");
  const T& mass = get_mass();
  if (mass < T(0)) {
      error_message += fmt::format("\nmass = {} is negative.\n", mass);
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
