#include "drake/multibody/tree/rigid_body.h"

#include <memory>

#include "drake/multibody/tree/model_instance.h"

namespace drake {
namespace multibody {

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double>& M)
    : Body<T>(body_name, default_model_instance()),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        ModelInstanceIndex model_instance,
                        const SpatialInertia<double>& M)
    : Body<T>(body_name, model_instance),
      default_spatial_inertia_(M) {}

template <typename T>
void RigidBody<T>::SetCenterOfMassInBodyFrameNoModifyInertia(
    systems::Context<T>* context,
    const Vector3<T>& center_of_mass_position) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  const T& x = center_of_mass_position(0);
  const T& y = center_of_mass_position(1);
  const T& z = center_of_mass_position(2);
  systems::BasicVector<T>& spatial_inertia_parameter =
      context->get_mutable_numeric_parameter(spatial_inertia_parameter_index_);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_com_x, x);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_com_y, y);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_com_z, z);
}

template <typename T>
void RigidBody<T>::SetUnitInertiaAboutBodyOrigin(
    systems::Context<T>* context,
    const UnitInertia<T>& G_BBo_B) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  const T& Gxx = G_BBo_B(0, 0);
  const T& Gyy = G_BBo_B(1, 1);
  const T& Gzz = G_BBo_B(2, 2);
  const T& Gxy = G_BBo_B(0, 1);
  const T& Gxz = G_BBo_B(0, 2);
  const T& Gyz = G_BBo_B(1, 2);
  systems::BasicVector<T>& spatial_inertia_parameter =
      context->get_mutable_numeric_parameter(spatial_inertia_parameter_index_);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_Gxx, Gxx);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_Gyy, Gyy);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_Gzz, Gzz);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_Gxy, Gxy);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_Gxz, Gxz);
  spatial_inertia_parameter.SetAtIndex(
      internal::parameter_conversion::SpatialInertiaIndex::k_Gyz, Gyz);
}

template <typename T>
void RigidBody<T>::SetCenterOfMassInBodyFrameAndPreserveCentralInertia(
    systems::Context<T>* context,
    const Vector3<T>& center_of_mass_position) const {
  DRAKE_THROW_UNLESS(context != nullptr);

  // Get B's initial spatial inertia about Bo (before Bcm changes location).
  // Get pi_BoBcm_B position from Bo to Bcm before Bcm changes location.
  // Get Gi_BBo_B (B's initial unit inertia about Bo, before Bcm changes).
  const SpatialInertia<T> Mi_BBo_B =
      CalcSpatialInertiaInBodyFrame(*context);
  const Vector3<T>& pi_BoBcm_B = Mi_BBo_B.get_com();
  const UnitInertia<T>& Gi_BBo_B = Mi_BBo_B.get_unit_inertia();

  // Calculate Gf_BBo_B (B's final unit inertia about Bo, after Bcm changes).
  const Vector3<T>& pf_BoBcm_B = center_of_mass_position;  // Alias for clarity.
  const RotationalInertia<T> I_BBo_B = Gi_BBo_B.ShiftToThenAwayFromCenterOfMass(
      /* mass = */ 1, pi_BoBcm_B, pf_BoBcm_B);
  const UnitInertia<T> Gf_BBo_B = UnitInertia<T>(I_BBo_B);
  // Note: One way to conceptualize this calculation is that B's origin Bo moves
  // from its initial location Boi to an intermediate location Bof and it only
  // returns to its initial location Boi with the final call below to:
  // SetCenterOfMassInBodyFrameNoModifyInertia(context, pf_BoBcm_B);
  // Hint: Drawing a picture can help speed making sense of this.

  // Modify the context. Update B's unit inertia about Bo.
  // Modify the context. Update B's center of mass position from Bo.
  SetUnitInertiaAboutBodyOrigin(context, Gf_BBo_B);
  SetCenterOfMassInBodyFrameNoModifyInertia(context, pf_BoBcm_B);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody)
