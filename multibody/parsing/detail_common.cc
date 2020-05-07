#include "drake/multibody/parsing/detail_common.h"

namespace drake {
namespace multibody {
namespace internal {

geometry::ProximityProperties ParseProximityProperties(
    const std::function<std::optional<double>(const char*)>& read_double,
    bool is_rigid, bool is_soft) {
  // Both being true is disallowed -- so assert is_rigid NAND is_soft.
  DRAKE_DEMAND(!(is_rigid && is_soft));
  geometry::ProximityProperties properties;
  std::optional<double> rez_hint = read_double("drake:mesh_resolution_hint");
  if (rez_hint) {
    if (is_rigid) {
      geometry::AddRigidHydroelasticProperties(*rez_hint, &properties);
    } else if (is_soft) {
      geometry::AddSoftHydroelasticProperties(*rez_hint, &properties);
    } else {
      properties.AddProperty(geometry::internal::kHydroGroup,
                             geometry::internal::kRezHint, *rez_hint);
    }
  } else {
    if (is_rigid) {
      geometry::AddRigidHydroelasticProperties(&properties);
    } else if (is_soft) {
      geometry::AddSoftHydroelasticProperties(&properties);
    }
  }

  std::optional<double> elastic_modulus = read_double("drake:elastic_modulus");

  std::optional<double> dissipation =
      read_double("drake:hunt_crossley_dissipation");

  std::optional<double> mu_dynamic = read_double("drake:mu_dynamic");
  std::optional<double> mu_static = read_double("drake:mu_static");
  std::optional<CoulombFriction<double>> friction;
  // Note: we rely on the constructor of CoulombFriction to detect negative
  // values and bad relationship between static and dynamic coefficients.
  if (mu_dynamic && mu_static) {
    friction = CoulombFriction<double>(*mu_static, *mu_dynamic);
  } else if (mu_dynamic) {
    friction = CoulombFriction<double>(*mu_dynamic, *mu_dynamic);
  } else if (mu_static) {
    friction = CoulombFriction<double>(*mu_static, *mu_static);
  }

  geometry::AddContactMaterial(elastic_modulus, dissipation, friction,
                               &properties);

  return properties;
}

const LinearBushingRollPitchYaw<double>& ParseLinearBushingRollPitchYaw(
    const std::function<Eigen::Vector3d(const char*)>& read_vector,
    const std::function<const Frame<double>&(const char*)>& read_frame,
    MultibodyPlant<double>* plant) {
  const Frame<double>& frame_A = read_frame("drake:bushing_frameA");
  const Frame<double>& frame_C = read_frame("drake:bushing_frameC");

  const Eigen::Vector3d bushing_torque_stiffness =
      read_vector("drake:bushing_torque_stiffness");
  const Eigen::Vector3d bushing_torque_damping =
      read_vector("drake:bushing_torque_damping");
  const Eigen::Vector3d bushing_force_stiffness =
      read_vector("drake:bushing_force_stiffness");
  const Eigen::Vector3d bushing_force_damping =
      read_vector("drake:bushing_force_damping");

  return plant->AddForceElement<LinearBushingRollPitchYaw>(
      frame_A, frame_C, bushing_torque_stiffness, bushing_torque_damping,
      bushing_force_stiffness, bushing_force_damping);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
