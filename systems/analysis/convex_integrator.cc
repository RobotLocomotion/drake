#include "drake/systems/analysis/convex_integrator.h"

#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap.h"
#include "drake/multibody/contact_solvers/pooled_sap/pooled_sap_builder.h"

namespace drake {
namespace systems {

using multibody::contact_solvers::pooled_sap::PooledSapBuilder;
using multibody::contact_solvers::pooled_sap::PooledSapModel;

template <typename T>
ConvexIntegrator<T>::ConvexIntegrator(const System<T>& system,
                                      Context<T>* context)
    : IntegratorBase<T>(system, context) {}

template <typename T>
ConvexIntegrator<T>::ConvexIntegrator(const System<T>& system,
                                      MultibodyPlant<T>* plant,
                                      Context<T>* context)
    : IntegratorBase<T>(system, context) {
  set_plant(plant);
}

template <typename T>
void ConvexIntegrator<T>::DoInitialize() {
  DRAKE_THROW_UNLESS(plant_ != nullptr);

  // TODO(vincekurtz): check that the plant is part of this->get_system().

  // For now, the convex integrator only supports systems where the only
  // second-order state (q, v) is from the MultibodyPlant.
  const int nq = this->get_context().get_continuous_state().num_q();
  const int nv = this->get_context().get_continuous_state().num_v();
  DRAKE_THROW_UNLESS(nq == plant_->num_positions());
  DRAKE_THROW_UNLESS(nv == plant_->num_velocities());
}

template <typename T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  Context<T>& context = *this->get_mutable_context();

  // Set up the pooled sap model (eventually allocation of builder and model
  // will move to DoInitialize).
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);
  PooledSapBuilder<T> builder(plant());
  PooledSapModel<T> model;
  builder.UpdateModel(plant_context, h, &model);

  context.SetTime(context.get_time() + h);
  return true;
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::systems::ConvexIntegrator);
