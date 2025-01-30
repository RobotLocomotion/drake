#include "drake/systems/analysis/convex_integrator.h"

namespace drake {
namespace systems {

using drake::multibody::MultibodyForces;

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  // Get the plant context. Note that we assume the only continuous state is the
  // plant's, and there are no controllers connected to it.
  Context<T>& context =
      plant().GetMyMutableContextFromRoot(this->get_mutable_context());

  // TODO(vincekurtz): avoid allocations
  VectorX<T> v_star(plant().num_velocities());
  VectorX<T> q = plant().GetPositions(context);

  CalcFreeMotionVelocities(context, h, &v_star);
  q += h * v_star;  // TODO(vincekurtz): handle quaternions

  plant().SetPositions(&context, q);
  plant().SetVelocities(&context, v_star);

  return true;  // step was successful
}

template <class T>
void ConvexIntegrator<T>::CalcFreeMotionVelocities(const Context<T>& context,
                                                   const T& h,
                                                   VectorX<T>* v_star) const {
  // TODO(vincekurtz) avoid allocations with a workspace or similar
  MultibodyForces<T> f_ext(plant());
  VectorX<T> a = VectorX<T>::Zero(plant().num_velocities());
  VectorX<T> k(plant().num_velocities());
  MatrixX<T> M(plant().num_velocities(), plant().num_velocities());

  plant().CalcForceElementsContribution(context, &f_ext);
  k = plant().CalcInverseDynamics(context, a, f_ext);
  plant().CalcMassMatrix(context, &M);

  Eigen::VectorBlock<const VectorX<T>> v0 = plant().GetVelocities(context);
  *v_star = v0 + h * M.ldlt().solve(-k);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
