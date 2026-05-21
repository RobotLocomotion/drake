#include "drake/multibody/tree/uniform_gravity_field_element.h"

#include <utility>
#include <vector>

#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"

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

// This is a private helper method to collect all the gravity forces, shifted
// to each mobilized body's body frame.
template <typename T>
void UniformGravityFieldElement<T>::AccumulateGravitySpatialForces(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& pc,
    std::vector<SpatialForce<T>>* F_Bo_W_array) const {
  DRAKE_ASSERT(F_Bo_W_array != nullptr);
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const internal::FrameBodyPoseCache<T>& fbpc =
      model.EvalFrameBodyPoses(context);

  const Vector3<double>& g_W = gravity_vector();

  // Loop over all mobilized bodies, skipping World.
  for (const auto& mobod : model.forest().mobods()) {
    if (mobod.is_world()) continue;
    const internal::MobodIndex mobod_index = mobod.index();
    const math::RotationMatrix<T>& R_WB = pc.get_R_WB(mobod_index);

    // Loop over all the links that follow this mobod (can be more than
    // one link if we're combining welded-together links into composites).

    // TODO(sherm1) It seems very unlikely that there would be individual links
    //  for which gravity has been disabled -- the useful cases (if any!) would
    //  likely turn off gravity for the whole mobod. Consider making it a
    //  modeling error to mix gravity on/off within a composite. In that case
    //  we could simply use the precalculated mobod spatial inertia M_BBo_B
    //  for mass and center of mass and not have to loop through links at all.
    for (const auto& link_ordinal : mobod.follower_link_ordinals()) {
      const auto& topo_link = model.graph().links(link_ordinal);

      // Skip this link if gravity is disabled for its model instance.
      if (!is_enabled(topo_link.model_instance())) continue;

      // Compute p_BoLcm_W, the position vector from the mobod origin Bo to
      // the link's center of mass, expressed in the World frame W. (Could
      // be cached in the PositionKinematicsCache.)
      const Vector3<T>& p_BoLcm_B = fbpc.get_p_BoLcm_B(link_ordinal);
      const Vector3<T> p_BoLcm_W = R_WB * p_BoLcm_B;  // 15 flops

      // Compute f_Lcm_W, the gravity force at the link's center of mass,
      // expressed in World. Then shift to the mobod origin Bo and accumulate.
      const T& mass_of_L = fbpc.get_M_LLo_L(link_ordinal).get_mass();
      const Vector3<T> f_Lcm_W = mass_of_L * g_W;
      (*F_Bo_W_array)[mobod_index] +=
          SpatialForce<T>(p_BoLcm_W.cross(f_Lcm_W), f_Lcm_W);
    }
  }
}

template <typename T>
VectorX<T> UniformGravityFieldElement<T>::CalcGravityGeneralizedForces(
    const systems::Context<T>& context) const {
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const internal::PositionKinematicsCache<T>& pc =
      model.EvalPositionKinematics(context);

  const int num_mobods = model.num_mobods();

  // Accumulate the gravity spatial force on each mobod, applied at the
  // mobod origin Bo, expressed in the World frame W. Zero-initialize so
  // we can skip some links and accumulate others.
  std::vector<SpatialForce<T>> F_Bo_W_array(num_mobods,
                                            SpatialForce<T>::Zero());
  AccumulateGravitySpatialForces(context, pc, &F_Bo_W_array);

  // tau_g = ∑ Jv_V_WBo_Wᵀ ⋅ F_grav_Bo_W, computed in O(n) without
  // forming the entire system Jacobian matrix. The function
  // CalcSystemJacobianTransposeTimesF() overwrites (accumulates)
  // into F_Bo_W_array during its tip-to-base sweep.
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
  DRAKE_ASSERT(this->has_parent_tree());
  std::vector<SpatialForce<T>>& F_Bo_W_array = forces->mutable_body_forces();
  AccumulateGravitySpatialForces(context, pc, &F_Bo_W_array);
}

template <typename T>
T UniformGravityFieldElement<T>::CalcPotentialEnergy(
    const systems::Context<T>& context,
    const internal::PositionKinematicsCache<T>& pc) const {
  // Add the potential energy due to gravity for each body in the model.
  // Skip the world.
  DRAKE_THROW_UNLESS(this->has_parent_tree());
  const internal::MultibodyTree<T>& model = this->get_parent_tree();
  const internal::FrameBodyPoseCache<T>& fbpc =
      model.EvalFrameBodyPoses(context);
  const int num_links = model.num_links();
  T TotalPotentialEnergy = 0.0;
  // Skip the "world" link.
  for (LinkOrdinal link_ordinal(1); link_ordinal < num_links; ++link_ordinal) {
    const auto& topo_link = model.graph().links(link_ordinal);

    // Skip this link if gravity is disabled for its model instance.
    if (!is_enabled(topo_link.model_instance())) continue;

    // Compute p_WLcm_W, the position of the link's center of mass in the World
    // frame, expressed in World. (Could be cached in the
    // PositionKinematicsCache.)
    const SpatialInertia<T>& M_LLo_L = fbpc.get_M_LLo_L(link_ordinal);
    const Vector3<T>& p_LoLcm_L = M_LLo_L.get_com();
    const math::RigidTransform<T>& X_WL = pc.get_X_WL(link_ordinal);
    const Vector3<T>& p_WLcm_W = X_WL * p_LoLcm_L;

    const T& mass = M_LLo_L.get_mass();
    TotalPotentialEnergy -= (mass * p_WLcm_W.dot(gravity_vector()));
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
  const internal::FrameBodyPoseCache<T>& fbpc =
      model.EvalFrameBodyPoses(context);
  const int num_links = model.num_links();
  T TotalConservativePower = 0.0;
  // Skip the "world" link.
  for (LinkOrdinal link_ordinal(1); link_ordinal < num_links; ++link_ordinal) {
    const auto& topo_link = model.graph().links(link_ordinal);

    // Skip this link if gravity is disabled for its model instance.
    if (!is_enabled(topo_link.model_instance())) continue;

    // Compute p_LoLcm_W, the vector from link origin to its center of mass,
    // expressed in World. (Could be cached in the PositionKinematicsCache.)
    const SpatialInertia<T>& M_LLo_L = fbpc.get_M_LLo_L(link_ordinal);
    const Vector3<T>& p_LoLcm_L = M_LLo_L.get_com();
    const math::RotationMatrix<T>& R_WL = pc.get_X_WL(link_ordinal).rotation();
    const Vector3<T>& p_LoLcm_W = R_WL * p_LoLcm_L;

    // Now compute v_WLcm_W, the linear velocity in the World frame of the
    // center of mass, expressed World. (Could be cached in the
    // VelocityKinematicsCache.)
    const SpatialVelocity<T>& V_WL = vc.get_V_WL(link_ordinal);
    const SpatialVelocity<T> V_WLcm_W = V_WL.Shift(p_LoLcm_W);
    const Vector3<T>& v_WLcm_W = V_WLcm_W.translational();

    // The conservative power is defined to be positive when the potential
    // energy decreases.
    const T& mass = M_LLo_L.get_mass();
    TotalConservativePower += (mass * v_WLcm_W.dot(gravity_vector()));
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
