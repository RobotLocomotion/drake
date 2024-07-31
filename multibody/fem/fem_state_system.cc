#include "drake/multibody/fem/fem_state_system.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
FemStateSystem<T>::FemStateSystem(const VectorX<T>& model_q,
                                  const VectorX<T>& model_v,
                                  const VectorX<T>& model_a) {
  num_dofs_ = model_q.size();
  DRAKE_THROW_UNLESS(model_q.size() == model_v.size());
  DRAKE_THROW_UNLESS(model_q.size() == model_a.size());
  DRAKE_THROW_UNLESS(model_q.size() % 3 == 0);
  q_index_ = this->DeclareDiscreteState(model_q);
  q0_index_ = this->DeclareDiscreteState(model_q);
  v_index_ = this->DeclareDiscreteState(model_v);
  a_index_ = this->DeclareDiscreteState(model_a);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::FemStateSystem);
