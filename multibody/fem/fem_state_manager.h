#pragma once

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/** A leaf system that can publicly declare discrete states and cache entries.
 Used to manage FemState.
 @tparam_nonsymbolic_scalar */
template <typename T>
class FemStateManager : public systems::LeafSystem<T> {
 public:
  using systems::LeafSystem<T>::DeclareDiscreteState;
  using systems::SystemBase::DeclareCacheEntry;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
