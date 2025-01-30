#include "drake/systems/analysis/convex_integrator.h"

namespace drake {
namespace systems {

template <class T>
void ConvexIntegrator<T>::DoInitialize() {
  const int nq = plant().num_positions();
  const int nv = plant().num_velocities();

  // TODO(vincekurtz): in the future we might want some fancy caching instead of
  // the workspace, but for now we'll just try to allocate most things here.
  workspace_.q.resize(nq);
  workspace_.v_star.resize(nv);

  workspace_.M.resize(nv, nv);
  workspace_.k.resize(nv);
  workspace_.a.resize(nv);
  workspace_.f_ext = std::make_unique<MultibodyForces<T>>(plant());
}

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  // Get the plant context. Note that we assume the only continuous state is the
  // plant's, and there are no controllers connected to it.
  Context<T>& context =
      plant().GetMyMutableContextFromRoot(this->get_mutable_context());
  VectorX<T>& v_star = workspace_.v_star;
  VectorX<T>& q = workspace_.q;

  // Set up the SAP problem
  CalcFreeMotionVelocities(context, h, &v_star);

  // Set q_{t+h} = q_t + h N(q_t) v_{t+h}
  plant().MapVelocityToQDot(context, h * v_star, &q);
  q += plant().GetPositions(context);

  plant().SetPositions(&context, q);
  plant().SetVelocities(&context, v_star);

  return true;  // step was successful
}

template <class T>
void ConvexIntegrator<T>::CalcFreeMotionVelocities(const Context<T>& context,
                                                   const T& h,
                                                   VectorX<T>* v_star) {
  VectorX<T>& k = workspace_.k;
  VectorX<T>& a = workspace_.a;
  MatrixX<T>& M = workspace_.M;
  MultibodyForces<T>& f_ext = *workspace_.f_ext;
  Eigen::VectorBlock<const VectorX<T>> v0 = plant().GetVelocities(context);

  a.setZero();
  plant().CalcForceElementsContribution(context, &f_ext);
  k = plant().CalcInverseDynamics(context, a, f_ext);
  plant().CalcMassMatrix(context, &M);

  *v_star = v0 + h * M.ldlt().solve(-k);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
