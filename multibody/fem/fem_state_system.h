#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* A leaf system that manages state and element data in a FEM model.
 @tparam_default_scalar */
template <typename T>
class FemStateSystem : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FemStateSystem);

  /* Constructs a new FemStateSystem with the given model states. No element
   data is declared.
   @pre model_q, model_v, model_a all have the same size.
   @pre model_q's size is a multiple of 3. */
  FemStateSystem(const VectorX<T>& model_q, const VectorX<T>& model_v,
                 const VectorX<T>& model_a);

  /* Promotes DeclareCacheEntry so that FemStateSystem can declare cache
  entries publicly. */
  using systems::SystemBase::DeclareCacheEntry;

  /* Returns the discrete state index. */
  systems::DiscreteStateIndex fem_position_index() const { return q_index_; }
  systems::DiscreteStateIndex fem_previous_step_position_index() const {
    return q0_index_;
  }
  systems::DiscreteStateIndex fem_velocity_index() const { return v_index_; }
  systems::DiscreteStateIndex fem_acceleration_index() const {
    return a_index_;
  }

  /* Returns the number of degrees of freedom in the system. */
  int num_dofs() const { return num_dofs_; }

 private:
  systems::DiscreteStateIndex q_index_;
  systems::DiscreteStateIndex q0_index_;
  systems::DiscreteStateIndex v_index_;
  systems::DiscreteStateIndex a_index_;
  int num_dofs_{0};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
