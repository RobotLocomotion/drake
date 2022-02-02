#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {

template <typename T>
void FemStateImplImplImplImplBase<T>::SetPositions(
    const Eigen::Ref<const VectorX<T>>& q) {
  DRAKE_THROW_UNLESS(q.size() == q_.size());
  InvalidateAllCacheEntries();
  q_ = q;
}

template <typename T>
void FemStateImplImplImplImplBase<T>::SetVelocities(
    const Eigen::Ref<const VectorX<T>>& v) {
  DRAKE_THROW_UNLESS(v.size() == v_.size());
  InvalidateAllCacheEntries();
  v_ = v;
}

template <typename T>
void FemStateImplImplImplImplBase<T>::SetAccelerations(
    const Eigen::Ref<const VectorX<T>>& a) {
  DRAKE_THROW_UNLESS(a.size() == a_.size());
  InvalidateAllCacheEntries();
  a_ = a;
}

}  // namespace fem
}  // namespace multibody
}  // namespace drake
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemStateImplImplImplImplBase);
