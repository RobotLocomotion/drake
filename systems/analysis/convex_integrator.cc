#include "drake/systems/analysis/convex_integrator.h"

namespace drake {
namespace systems {

using drake::multibody::MultibodyForces;

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  Context<T>& context = *this->get_mutable_context();

  // TODO: use plant context only:
  // Context<T>& context = plant().GetMyMutableContextFromRoot(*this->get_mutable_context());


  VectorBase<T>& x = context.get_mutable_continuous_state_vector();

  // TODO(vincekurtz): update the state
  // For now just some placeholder nonesense so we can see some dynamics
  const T x0 = x.GetAtIndex(0);
  x.SetAtIndex(0, x0 + h);

  // TODO(vincekurtz): avoid allocation
  VectorX<T> v_star(plant().num_velocities());

  CalcFreeMotionVelocities(context, &v_star);

  return true;  // step was successful
}

template <class T>
void ConvexIntegrator<T>::CalcFreeMotionVelocities(const Context<T>& context,
                                                   VectorX<T>* v_star) const {
  const Context<T>& plant_context = plant().GetMyContextFromRoot(context);

  // TODO(vincekurtz) avoid allocations with a workspace or similar
  MultibodyForces<T> f_ext(plant());
  VectorX<T> a = VectorX<T>::Zero(plant().num_velocities());
  VectorX<T> k(plant().num_velocities());
  MatrixX<T> M(plant().num_velocities(), plant().num_velocities());

  plant().CalcForceElementsContribution(plant_context, &f_ext);
  k = plant().CalcInverseDynamics(plant_context, a, f_ext);
  plant().CalcMassMatrix(plant_context, &M);

  Eigen::VectorBlock<const VectorX<T>> v0 = plant().GetVelocities(plant_context);
  double h = 0.01; // TODO: pass h and plant_context to this method
  *v_star = v0 + h * M.ldlt().solve(k);


}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
