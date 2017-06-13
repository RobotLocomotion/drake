#include "drake/multibody/multibody_tree/revolute_mobilizer.h"

#include <stdexcept>

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {

template <typename T>
const T& RevoluteMobilizer<T>::get_angle(
    const systems::Context<T>& context) const {
  const MultibodyTreeContext<T>& mbt_context =
      this->GetMultibodyTreeContextOrThrow(context);
  auto q = this->get_positions(mbt_context);
  DRAKE_ASSERT(q.size() == nq);
  return q.coeffRef(0);
}

template <typename T>
const RevoluteMobilizer<T>& RevoluteMobilizer<T>::set_angle(
    systems::Context<T>* context, const T& angle) const {
  MultibodyTreeContext<T>& mbt_context =
      this->GetMutableMultibodyTreeContextOrThrow(context);
  auto q = this->get_mutable_positions(&mbt_context);
  DRAKE_ASSERT(q.size() == nq);
  q[0] = angle;
  return *this;
}

template <typename T>
Isometry3<T> RevoluteMobilizer<T>::CalcAcrossMobilizerTransform(
    const MultibodyTreeContext<T>& context) const {
  auto q = this->get_positions(context);
  Isometry3<T> X_FM = Isometry3<T>::Identity();
  X_FM.linear() = Eigen::AngleAxis<T>(q[0], axis_F_).toRotationMatrix();
  return X_FM;
}

// Explicitly instantiates on the most common scalar types.
template class RevoluteMobilizer<double>;
template class RevoluteMobilizer<AutoDiffXd>;

}  // namespace multibody
}  // namespace drake
