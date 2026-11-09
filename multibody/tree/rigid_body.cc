#include "drake/multibody/tree/rigid_body.h"

#include <memory>

#include <fmt/format.h>

#include "drake/multibody/tree/model_instance.h"

namespace drake {
namespace multibody {

// RigidBodyFrame (a.k.a. LinkFrame) function definitions.

template <typename T>
RigidBodyFrame<T>::~RigidBodyFrame() = default;

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> RigidBodyFrame<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const RigidBody<ToScalar>& body_clone =
      tree_clone.get_link(this->body().index());
  // RigidBodyFrame's constructor cannot be called from std::make_unique since
  // it is private and therefore we use "new".
  return std::unique_ptr<RigidBodyFrame<ToScalar>>(
      new RigidBodyFrame<ToScalar>(body_clone));
}

template <typename T>
std::unique_ptr<Frame<double>> RigidBodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> RigidBodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<symbolic::Expression>> RigidBodyFrame<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<T>> RigidBodyFrame<T>::DoShallowClone() const {
  // RigidBodyFrame's constructor cannot be called from std::make_unique since
  // it is private and therefore we use "new".
  return std::unique_ptr<RigidBodyFrame<T>>(
      new RigidBodyFrame<T>(this->body()));
}

// RigidBody function definitions.

template <typename T>
RigidBody<T>::~RigidBody() = default;

template <typename T>
ScopedName RigidBody<T>::scoped_name() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  return ScopedName(
      this->get_parent_tree().GetModelInstanceName(this->model_instance()),
      name_);
}

template <typename T>
const Link<T>& RigidBody<T>::primary_link() const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const internal::MultibodyTree<T>& tree = this->get_parent_tree();
  return tree.get_link(
      tree.graph().link_by_index(this->index()).primary_link());
}

template <typename T>
void RigidBody<T>::Lock(systems::Context<T>* context) const {
  ThrowIfNotFinalized(__func__);
  if (!is_floating_base_body()) {
    throw std::logic_error(fmt::format(
        "Attempted to call Lock() on non-floating-base rigid body {}", name()));
  }
  mobilizer().Lock(context);
}

template <typename T>
void RigidBody<T>::Unlock(systems::Context<T>* context) const {
  ThrowIfNotFinalized(__func__);
  if (!is_floating_base_body()) {
    throw std::logic_error(fmt::format(
        "Attempted to call Unlock() on non-floating-base rigid body {}",
        name()));
  }
  mobilizer().Unlock(context);
}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        const SpatialInertia<double>& M)
    : MultibodyElement<T>(default_model_instance()),
      name_(internal::DeprecateWhenEmptyName(body_name, "RigidBody")),
      link_frame_(*this),
      default_spatial_inertia_(M) {}

template <typename T>
RigidBody<T>::RigidBody(const std::string& body_name,
                        ModelInstanceIndex model_instance,
                        const SpatialInertia<double>& M)
    : MultibodyElement<T>(model_instance),
      name_(internal::DeprecateWhenEmptyName(body_name, "RigidBody")),
      link_frame_(*this),
      default_spatial_inertia_(M) {}

template <typename T>
void RigidBody<T>::DoSetTopology() {
  DRAKE_DEMAND(mobilizer_ == nullptr);
  const internal::MultibodyTree<T>& tree = this->get_parent_tree();
  const internal::SpanningForest& forest = tree.forest();
  const internal::LinkJointGraph::Link& link = forest.link_by_index(index());
  mobilizer_ = &tree.get_mobilizer(link.mobod_index());

  // Is this RigidBody the active link on its Mobod?
  const bool is_active_link =
      link.ordinal() == forest.mobods(link.mobod_index()).active_link_ordinal();
  is_floating_base_body_ =
      is_active_link && mobilizer_->is_floating_base_mobilizer();
}

template <typename T>
void RigidBody<T>::DoDeclareParameters(
    internal::MultibodyTreeSystem<T>* tree_system) {
  // Ephemeral shadow links (created to break kinematic loops) deliberately have
  // no spatial-inertia parameter: their mass properties are not independently
  // settable but instead mirror their primary link's parameterized inertia
  // (split evenly in CalcFrameBodyPoses()). Leaving the parameter index invalid
  // makes any attempt to read or write their parameter fail loudly.
  if (this->is_ephemeral()) return;

  // Sets model values to dummy values to indicate that the model values are
  // not used. This class stores the the default values of the parameters.
  // 10 numeric values are used to store mass, center of mass, moments and
  // products of inertia packed into one basic vector.
  spatial_inertia_parameter_index_ =
      this->DeclareNumericParameter(tree_system, systems::BasicVector<T>(10));
}

template <typename T>
void RigidBody<T>::DoSetDefaultParameters(
    systems::Parameters<T>* parameters) const {
  // Ephemeral shadow links have no spatial-inertia parameter (see
  // DoDeclareParameters()), so there is nothing to default.
  if (this->is_ephemeral()) return;

  // Set the default spatial inertia.
  systems::BasicVector<T>& spatial_inertia_parameter =
      parameters->get_mutable_numeric_parameter(
          spatial_inertia_parameter_index_);
  spatial_inertia_parameter.SetFrom(
      internal::parameter_conversion::ToBasicVector<T>(
          default_spatial_inertia_.template cast<T>()));
}

template <typename T>
void RigidBody<T>::ThrowIfNotFinalized(const char* source_method) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  if (!this->get_parent_tree().is_finalized()) {
    throw std::runtime_error(fmt::format(
        "From '{}'. The model to which this rigid body belongs must be "
        "finalized. See MultibodyPlant::Finalize().",
        source_method));
  }
}

template <typename T>
void RigidBody<T>::ThrowEphemeralInertiaError(const char* source_method) const {
  throw std::logic_error(fmt::format(
      "{}(): RigidBody '{}' is a shadow link, which Finalize() created to "
      "break a kinematic loop. A shadow link has no mass properties of its "
      "own; use its primary_link(), '{}', instead.",
      source_method, this->name(), primary_link().name()));
}

template <typename T>
void RigidBody<T>::SetCenterOfMassInBodyFrameNoModifyInertia(
    systems::Context<T>* context,
    const Vector3<T>& center_of_mass_position) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  ThrowIfEphemeralInertia(__func__);
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
    systems::Context<T>* context, const UnitInertia<T>& G_BBo_B) const {
  DRAKE_THROW_UNLESS(context != nullptr);
  ThrowIfEphemeralInertia(__func__);
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
  ThrowIfEphemeralInertia(__func__);

  // Get B's initial spatial inertia about Bo (before Bcm changes location).
  // Get pi_BoBcm_B position from Bo to Bcm before Bcm changes location.
  // Get Gi_BBo_B (B's initial unit inertia about Bo, before Bcm changes).
  const SpatialInertia<T> Mi_BBo_B = CalcSpatialInertiaInBodyFrame(*context);
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

template <typename T>
void RigidBody<T>::AddInForce(const systems::Context<T>& context,
                              const Vector3<T>& p_BP_E,
                              const SpatialForce<T>& F_Bp_E,
                              const Frame<T>& frame_E,
                              MultibodyForces<T>* forces) const {
  DRAKE_THROW_UNLESS(forces != nullptr);
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  DRAKE_THROW_UNLESS(
      forces->CheckHasRightSizeForModel(this->get_parent_tree()));
  const math::RotationMatrix<T> R_WE =
      frame_E.CalcRotationMatrixInWorld(context);
  const Vector3<T> p_PB_W = -(R_WE * p_BP_E);
  const SpatialForce<T> F_Bo_W = (R_WE * F_Bp_E).Shift(p_PB_W);
  AddInForceInWorld(context, F_Bo_W, forces);
}

template <typename T>
Vector3<T> RigidBody<T>::CalcCenterOfMassTranslationalVelocityInWorld(
    const systems::Context<T>& context) const {
  ThrowIfEphemeralInertia(__func__);
  return this->get_parent_tree()
      .CalcLinkCenterOfMassTranslationalVelocityInWorld(context, *this);
}

template <typename T>
Vector3<T> RigidBody<T>::CalcCenterOfMassTranslationalAccelerationInWorld(
    const systems::Context<T>& context) const {
  ThrowIfEphemeralInertia(__func__);
  return this->get_parent_tree()
      .CalcLinkCenterOfMassTranslationalAccelerationInWorld(context, *this);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::RigidBodyFrame);

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RigidBody);
