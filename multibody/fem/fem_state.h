#pragma once

#include <memory>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace fem {

/** Stores the FEM states. The states include the generalized positions,
 velocities, and accelerations associated with each node.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemState {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FemState);

  virtual ~FemState() = default;

  /** @name State getters. @{ */
  const VectorX<T>& GetPositions() const { return q_; }

  const VectorX<T>& GetVelocities() const { return v_; }

  const VectorX<T>& GetAccelerations() const { return a_; }
  /** @} */

  /** @name State setters.
   The size of the values provided must match the current size of the states.
   Throw an exception otherwise.
   @{ */
  void SetPositions(const Eigen::Ref<const VectorX<T>>& q);

  void SetVelocities(const Eigen::Ref<const VectorX<T>>& v);

  void SetAccelerations(const Eigen::Ref<const VectorX<T>>& a);
  /** @} */

  /* Returns the number of generalized positions in the state. */
  int num_dofs() const { return q_.size(); }

  /** Constructs an %FemState with prescribed generalized positions,
   velocities, and accelerations.
   @param[in] q  The prescribed generalized positions.
   @param[in] v  The prescribed generalized velocities.
   @param[in] a  The prescribed generalized accelerations.
   @pre q.size() == v.size().
   @pre q.size() == a.size(). */
  FemState(const Eigen::Ref<const VectorX<T>>& q,
           const Eigen::Ref<const VectorX<T>>& v,
           const Eigen::Ref<const VectorX<T>>& a)
      : q_(q), v_(v), a_(a) {
    DRAKE_DEMAND(q_.size() == v_.size());
    DRAKE_DEMAND(q_.size() == a_.size());
  }

 private:
  /* Generalized positions. */
  VectorX<T> q_{};
  /* Generalized velocities. */
  VectorX<T> v_{};
  /* Generalized accelerations. */
  VectorX<T> a_{};
};

}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::FemState);
