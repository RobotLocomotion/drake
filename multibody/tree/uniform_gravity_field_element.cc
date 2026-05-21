#include "drake/multibody/tree/uniform_gravity_field_element.h"

#include <utility>
#include <vector>

#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"

// TODO(sherm1) Everything here needs to be reworked in terms of mobilized
//  bodies rather than individual links. Also, use the available caches
//  rather than recalculating them.
namespace drake {
namespace multibody {

template <typename T>
UniformGravityFieldElement<T>::UniformGravityFieldElement()
    : UniformGravityFieldElement<T>(
          Vector3<double>(0.0, 0.0, -kDefaultStrength)) {}

template <typename T>
UniformGravityFieldElement<T>::UniformGravityFieldElement(Vector3<double> g_W)
    : ForceElement<T>(world_model_instance()), g_W_(g_W) {}

template <typename T>
UniformGravityFieldElement<T>::~UniformGravityFieldElement() = default;

template <typename T>
void UniformGravityFieldElement<T>::set_enabled(
    ModelInstanceIndex model_instance, bool is_enabled) {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  if (this->get_parent_tree().is_finalized()) {
    throw std::logic_error("Gravity can only be enabled pre-finalize.");
  }
  if (model_instance >= this->get_parent_tree().num_model_instances()) {
    throw std::logic_error("Model instance index is invalid.");
  }
  if (is_enabled) {
    disabled_model_instances_.erase(model_instance);
  } else {
    disabled_model_instances_.insert(model_instance);
  }
}

template <typename T>
UniformGravityFieldElement<T>::UniformGravityFieldElement(
    Vector3<double> g_W, std::set<ModelInstanceIndex> disabled_model_instances)
    : ForceElement<T>(world_model_instance()),
      g_W_(g_W),
      disabled_model_instances_(std::move(disabled_model_instances)) {}

template <typename T>
VectorX<T> UniformGravityFieldElement<T>::CalcGravityGeneralizedForces(
    const systems::Context<T>& context) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const internal::PositionKinematicsCache<T>& pc =
      model.EvalPositionKinematics(context);
  const internal::FrameBodyPoseCache<T>& fbpc =
      model.EvalFrameBodyPoses(context);

  const Vector3<double>& g_W = gravity_vector();
  const int num_mobods = model.num_mobods();

  // Accumulate the gravity spatial force on each mobod, applied at the
  // mobod origin Bo, expressed in the World frame W. We don't need to
  // initialize this memory because we'll start by writing the active link's
  // contribution.
  std::vector<SpatialForce<T>> F_Bo_W_array(num_mobods,
                                            SpatialForce<T>::Zero());

  // Loop over all mobilized bodies, skipping World and anchored mobods.
  for (const auto& mobod : model.forest().mobods()) {
    if (mobod.is_anchored()) continue;  // e.g., World
    const internal::MobodIndex mobod_index = mobod.index();
    const math::RotationMatrix<T>& R_WB = pc.get_R_WB(mobod_index);

    for (const auto& link_ordinal : mobod.follower_link_ordinals()) {
      const auto& topo_link = model.graph().links(link_ordinal);

      // Skip this link if gravity is disabled for its model instance.
      if (!is_enabled(topo_link.model_instance())) continue;

      const T& mass_of_L = fbpc.get_M_LLo_L(link_ordinal).get_mass();
      const Vector3<T>& p_BoLcm_B = fbpc.get_p_BoLcm_B(link_ordinal);

      // NOTE: p_BoLcm_W could be cached in the PositionKinematicsCache.
      const Vector3<T> p_BoLcm_W = R_WB * p_BoLcm_B;  // 15 flops

      // Gravity force at the link's center of mass, shifted to Bo.
      const Vector3<T> f_Lcm_W = mass_of_L * g_W;
      F_Bo_W_array[mobod_index] +=
          SpatialForce<T>(p_BoLcm_W.cross(f_Lcm_W), f_Lcm_W);
    }
  }

  // tau_g = ∑ J_WBᵀ(q) ⋅ F_grav_Bo_W, computed in O(n) without forming J.
  // The operator destructively accumulates into F_Bo_W_array during
  // its tip-to-base sweep.
  VectorX<T> tau_g(model.num_velocities());
  model.CalcSystemJacobianTransposeTimesF(context, &F_Bo_W_array, &tau_g);
  return tau_g;
}

template <typename T>
void UniformGravityFieldElement<T>::DoCalcAndAddForceContribution(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>&,
    MultibodyForces<T>* forces) const {
  // Alias to the array of applied body forces:
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();

  // Add the force of gravity contribution for each body in the model.
  // Skip the world.
  DRAKE_ASSERT(this->has_parent_tree());
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const int num_links = model.num_links();
  // Skip the "world" link.
  for (LinkIndex link_index(1); link_index < num_links; ++link_index) {
    const RigidBody<T>& link = model.get_link(link_index);

    // Skip this link if gravity is disabled.
    if (!is_enabled(link.model_instance())) continue;

    internal::MobodIndex mobod_index = link.mobod_index();

    // TODO(amcastro-tri): Replace this CalcFoo() calls by GetFoo() calls once
    //  caching is in place.
    const T mass = link.get_mass(context);
    const Vector3<T> p_BoBcm_B = link.CalcCenterOfMassInBodyFrame(context);
    const math::RotationMatrix<T> R_WB = pc.get_R_WB(mobod_index);
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const Vector3<T> f_Bcm_W = mass * gravity_vector();
    const SpatialForce<T> F_Bo_W(p_BoBcm_W.cross(f_Bcm_W), f_Bcm_W);
    F_Bo_W_array[mobod_index] += F_Bo_W;
  }
}

template <typename T>
T UniformGravityFieldElement<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& pc) const {
  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const int num_links = model.num_links();
  T TotalPotentialEnergy = 0.0;
  // Skip the "world" link.
  for (LinkIndex link_index(1); link_index < num_links; ++link_index) {
    const RigidBody<T>& link = model.get_link(link_index);

    // Skip this link if gravity is disabled.
    if (!is_enabled(link.model_instance())) continue;

    // TODO(amcastro-tri): Replace this CalcFoo() calls by GetFoo() calls once
    // caching is in place.
    const T mass = link.get_mass(context);
    const Vector3<T> p_BoBcm_B = link.CalcCenterOfMassInBodyFrame(context);
    const math::RigidTransform<T>& X_WB = pc.get_X_WB(link.mobod_index());
    const math::RotationMatrix<T> R_WB = X_WB.rotation();
    const Vector3<T> p_WBo = X_WB.translation();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W and/or p_WBcm.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;
    const Vector3<T> p_WBcm = p_WBo + p_BoBcm_W;

    TotalPotentialEnergy -= (mass * p_WBcm.dot(gravity_vector()));
  }
  return TotalPotentialEnergy;
}

template <typename T>
T UniformGravityFieldElement<T>::CalcConservativePower(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& pc,
    const internal::VelocityKinematicsCache<T>& vc) const {
  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const int num_links = model.num_links();
  T TotalConservativePower = 0.0;
  // Skip the "world" link.
  for (LinkIndex link_index(1); link_index < num_links; ++link_index) {
    const RigidBody<T>& link = model.get_link(link_index);

    // Skip this link if gravity is disabled.
    if (!is_enabled(link.model_instance())) continue;

    // TODO(amcastro-tri): Replace this CalcFoo() calls by GetFoo() calls once
    //  caching is in place.
    const T mass = link.get_mass(context);
    const Vector3<T> p_BoBcm_B = link.CalcCenterOfMassInBodyFrame(context);
    const math::RigidTransform<T>& X_WB = pc.get_X_WB(link.mobod_index());
    const math::RotationMatrix<T> R_WB = X_WB.rotation();
    // TODO(amcastro-tri): Consider caching p_BoBcm_W.
    const Vector3<T> p_BoBcm_W = R_WB * p_BoBcm_B;

    const SpatialVelocity<T>& V_WB = vc.get_V_WB(link.mobod_index());
    const SpatialVelocity<T> V_WBcm = V_WB.Shift(p_BoBcm_W);
    const Vector3<T>& v_WBcm = V_WBcm.translational();

    // The conservative power is defined to be positive when the potential
    // energy decreases.
    TotalConservativePower += (mass * v_WBcm.dot(gravity_vector()));
  }
  return TotalConservativePower;
}

template <typename T>
T UniformGravityFieldElement<T>::CalcNonConservativePower(
    const systems::Context<T>&, const internal::PositionKinematicsCache<T>&,
    const internal::VelocityKinematicsCache<T>&) const {
  // A uniform gravity field is conservative. Therefore return zero power.
  return 0.0;
}

template <typename T>
std::unique_ptr<ForceElement<double>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>&) const {
  return std::make_unique<UniformGravityFieldElement<double>>(
      gravity_vector(), disabled_model_instances_);
}

template <typename T>
std::unique_ptr<ForceElement<AutoDiffXd>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>&) const {
  return std::make_unique<UniformGravityFieldElement<AutoDiffXd>>(
      gravity_vector(), disabled_model_instances_);
}

template <typename T>
std::unique_ptr<ForceElement<symbolic::Expression>>
UniformGravityFieldElement<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>&) const {
  return std::make_unique<UniformGravityFieldElement<symbolic::Expression>>(
      gravity_vector(), disabled_model_instances_);
}

template <typename T>
std::unique_ptr<ForceElement<T>> UniformGravityFieldElement<T>::DoShallowClone()
    const {
  return std::make_unique<UniformGravityFieldElement<T>>(
      gravity_vector(), disabled_model_instances_);
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::UniformGravityFieldElement);
