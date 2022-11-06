#include "drake/systems/controllers/joint_stiffness_controller.h"

#include <utility>
#include <vector>

namespace drake {
namespace systems {
namespace controllers {

using multibody::MultibodyForces;
using multibody::MultibodyPlant;

using Eigen::VectorXd;

template <typename T>
JointStiffnessController<T>::JointStiffnessController(
    std::unique_ptr<multibody::MultibodyPlant<T>> owned_plant,
    const MultibodyPlant<T>* plant, const Eigen::Ref<const Eigen::VectorXd>& kp,
    const Eigen::Ref<const Eigen::VectorXd>& kd)
    : LeafSystem<T>(SystemTypeTag<JointStiffnessController>{}),
      owned_plant_(std::move(owned_plant)),
      plant_(owned_plant_ ? owned_plant_.get() : plant),
      kp_(kp),
      kd_(kd) {
  // Exactly one of owned_plant_ or plant should have been nullptr.
  DRAKE_DEMAND(owned_plant_ == nullptr || plant == nullptr);
  DRAKE_DEMAND(plant_ != nullptr);
  DRAKE_DEMAND(plant_->is_finalized());

  const int num_states = plant_->num_multibody_states();
  const int num_q = plant_->num_positions();
  DRAKE_DEMAND(num_q == plant_->num_velocities());
  DRAKE_DEMAND(num_q == plant_->num_actuated_dofs());

  DRAKE_DEMAND(kp.size() == num_q);
  DRAKE_DEMAND(kd.size() == num_q);

  input_port_index_estimated_state_ =
      this->DeclareInputPort("estimated_state", kVectorValued, num_states)
          .get_index();

  input_port_index_desired_state_ =
      this->DeclareInputPort("desired_state", kVectorValued, num_states)
          .get_index();

  // Forces need to be recalculated when any input has changed but have no
  // other dependencies. Specifying this here is also essential so that that
  // GetDirectFeedthrough won't attempt to cast to Symbolic for evaluation
  // (which would fail if the plant is not owned).
  output_port_index_force_ =
      this->DeclareVectorOutputPort(
              "generalized_force", num_q,
              &JointStiffnessController<T>::CalcOutputForce,
              {this->all_input_ports_ticket()})
          .get_index();

  auto plant_context = plant_->CreateDefaultContext();

  // Declare cache entry for the multibody plant context.
  plant_context_cache_index_ =
      this->DeclareCacheEntry(
              "plant_context_cache",
              *plant_context,
              &JointStiffnessController<T>::SetMultibodyContext,
              {this->input_port_ticket(
                  get_input_port_estimated_state().get_index())})
          .cache_index();

  // Declare external forces cache entry
  applied_forces_cache_index_ =
      this->DeclareCacheEntry(
              "applied_forces_cache", MultibodyForces<T>(*plant_),
              &JointStiffnessController<T>::CalcMultibodyForces,
              {this->cache_entry_ticket(plant_context_cache_index_)})
          .cache_index();
}

template <typename T>
JointStiffnessController<T>::JointStiffnessController(
    const MultibodyPlant<T>& plant, const Eigen::Ref<const Eigen::VectorXd>& kp,
    const Eigen::Ref<const Eigen::VectorXd>& kd)
    : JointStiffnessController(nullptr, &plant, kp, kd) {}

template <typename T>
JointStiffnessController<T>::JointStiffnessController(
    std::unique_ptr<multibody::MultibodyPlant<T>> plant,
    const Eigen::Ref<const Eigen::VectorXd>& kp,
    const Eigen::Ref<const Eigen::VectorXd>& kd)
    : JointStiffnessController(std::move(plant), nullptr, kp, kd) {}

template <typename T>
template <typename U>
JointStiffnessController<T>::JointStiffnessController(
    const JointStiffnessController<U>& other)
    : JointStiffnessController(
          systems::System<U>::template ToScalarType<T>(*other.plant_),
          other.kp_, other.kd_) {}

template <typename T>
JointStiffnessController<T>::~JointStiffnessController() = default;

template <typename T>
void JointStiffnessController<T>::SetMultibodyContext(
    const Context<T>& context,
    Context<T>* plant_context) const {
  const VectorX<T>& x = get_input_port_estimated_state().Eval(context);
  plant_->SetPositionsAndVelocities(plant_context, x);
}

template <typename T>
void JointStiffnessController<T>::CalcMultibodyForces(
    const Context<T>& context, MultibodyForces<T>* cache_value) const {
  const auto& plant_context =
      this->get_cache_entry(plant_context_cache_index_)
          .template Eval<Context<T>>(context);
  plant_->CalcForceElementsContribution(plant_context, cache_value);
}

template <typename T>
void JointStiffnessController<T>::CalcOutputForce(
    const Context<T>& context, BasicVector<T>* output) const {
  const int num_q = plant_->num_positions();

  const auto& plant_context =
      this->get_cache_entry(plant_context_cache_index_)
          .template Eval<Context<T>>(context);

  // These include gravity.
  const auto& applied_forces =
      this->get_cache_entry(applied_forces_cache_index_)
          .template Eval<MultibodyForces<T>>(context);

  // Compute inverse dynamics with zero generalized accelerations.
  // ID(q, v, v̇)  = M(q)v̇ + C(q, v)v - tau_app
  // So with v̇ = 0 we get:
  // ID(q, v, 0) = C(q,v)v - tau_app(q).
  VectorX<T> tau = plant_->CalcInverseDynamics(
      plant_context, VectorX<T>::Zero(num_q), /* vdot = 0 */
      applied_forces);

  // Subtract off C(q,v)v
  // Note: we do not simply set v=0 because we want to be able to cancel the
  // contribution from damping forces.
  VectorX<T> Cv(num_q);
  plant_->CalcBiasTerm(plant_context, &Cv);
  tau -= Cv;

  // Add in the stiffness terms.
  const VectorX<T>& x = get_input_port_estimated_state().Eval(context);
  const VectorX<T>& x_d = get_input_port_desired_state().Eval(context);

  tau += (kp_.array() * (x_d.head(num_q) - x.head(num_q)).array() +
          kd_.array() * (x_d.tail(num_q) - x.tail(num_q)).array())
             .matrix();

  output->get_mutable_value() = tau;
}

}  // namespace controllers
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::controllers::JointStiffnessController)
