#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace test {

// A LeafSystem that makes its protected methods public for unit testing.
template <typename T>
class PublicLeafSystem : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PublicLeafSystem)
  PublicLeafSystem() = default;
  ~PublicLeafSystem() override = default;

  using LeafSystem<T>::DeclareAbstractInputPort;
  using LeafSystem<T>::DeclareAbstractOutputPort;
  using LeafSystem<T>::DeclareCacheEntry;
  using LeafSystem<T>::DeclareContinuousState;
  using LeafSystem<T>::DeclareDiscreteState;
  using LeafSystem<T>::DeclareNumericParameter;
  using LeafSystem<T>::DeclarePerStepEvent;
  using LeafSystem<T>::DeclareVectorInputPort;
  using LeafSystem<T>::DeclareVectorOutputPort;
};

}  // namespace test
}  // namespace systems
}  // namespace drake
