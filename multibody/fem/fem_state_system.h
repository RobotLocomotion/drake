#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A leaf system that manages state and element data in a FEM model.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemStateSystem : public systems::LeafSystem<T> {
 public:
  /* Constructs a new FemStateSystem with the given model states. No element
   data is declared.
   @pre model_q, model_v, model_a all have the same size. */
  FemStateSystem(const VectorX<T>& model_q, const VectorX<T>& model_v,
                  const VectorX<T>& model_a);

  /* Promotes DeclareCacheEntry so that FemStateSystem can declare cache
  entries publicly. */
  using systems::SystemBase::DeclareCacheEntry;

  /* Returns the discrete state index. */
  systems::DiscreteStateIndex fem_position_index() const { return q_index_; }
  systems::DiscreteStateIndex fem_velocity_index() const { return v_index_; }
  systems::DiscreteStateIndex fem_acceleration_index() const {
    return a_index_;
  }

 private:
  systems::DiscreteStateIndex q_index_;
  systems::DiscreteStateIndex v_index_;
  systems::DiscreteStateIndex a_index_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
