#include "drake/multibody/contact_solvers/system_dynamics_data.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SystemDynamicsData<T>::SystemDynamicsData(const LinearOperator<T>* Ainv,
                                          const VectorX<T>* v_star)
    : Ainv_(Ainv), v_star_(v_star) {
  DRAKE_DEMAND(Ainv != nullptr);
  DRAKE_DEMAND(v_star != nullptr);
  DRAKE_DEMAND(Ainv->rows() == Ainv->cols());
  nv_ = Ainv->rows();
  DRAKE_DEMAND(v_star->size() == num_velocities());
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SystemDynamicsData)
