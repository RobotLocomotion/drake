#pragma once

#include <functional>
#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/** Implements an output port whose value is managed by a cache entry in the
same LeafSystem as the port. This is intended for internal use in implementing
the DeclareOutputPort() variants in LeafSystem.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following kinds of T's are provided:
- double
- AutoDiffXd
- symbolic::Expression

They are already available to link against in the containing library.
No other values for T are currently supported. */
// TODO(sherm1) Output ports that simply expose existing Context objects
// should not require a cache entry. Add provision for that to avoid the
// unnecessary copying currently done by Eval() for those.
template <typename T>
class LeafOutputPort final : public OutputPort<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafOutputPort)

  ~LeafOutputPort() override = default;

  // TODO(sherm1) These callbacks should not be specific to this class. Move
  // elsewhere, e.g. framework_common.h so they can be shared with cache entry.

  /** Signature of a function suitable for allocating an object that can hold
  a value of a particular output port. The result is returned as an
  AbstractValue even if this is a vector-valued port. */
  using AllocCallback = std::function<std::unique_ptr<AbstractValue>()>;

  /** Signature of a function suitable for calculating a value of a particular
  output port, given a place to put the value. */
  using CalcCallback =
  std::function<void(const Context<T>&, AbstractValue*)>;

  /** Signature of a function suitable for calculating a value of a particular
  vector-valued output port, given a place to put the value. */
  using CalcVectorCallback =
  std::function<void(const Context<T>&, BasicVector<T>*)>;

  /** Constructs a cached output port. The `system` parameter must be the same
  object as the `system_base` parameter. */
  LeafOutputPort(const System<T>* system, SystemBase* system_base,
                 OutputPortIndex index, DependencyTicket ticket,
                 PortDataType data_type, int size,
                 const CacheEntry* cache_entry)
      : OutputPort<T>(system, system_base, index, ticket, data_type, size),
        cache_entry_(cache_entry) {
    DRAKE_DEMAND(cache_entry != nullptr);
  }

  /** Returns the cache entry associated with this output port. */
  const CacheEntry& cache_entry() const {
    DRAKE_ASSERT(cache_entry_ != nullptr);
    return *cache_entry_;
  }

 private:
  // Invokes the cache entry's allocation function.
  std::unique_ptr<AbstractValue> DoAllocate() const final {
    return cache_entry().Allocate();
  }

  // Invokes the cache entry's calculator function.
  void DoCalc(const Context<T>& context, AbstractValue* value) const final {
    cache_entry().Calc(context, value);
  }

  // Invokes the cache entry's Eval() function.
  const AbstractValue& DoEval(const Context<T>& context) const final {
    return cache_entry().EvalAbstract(context);
  }

  // Returns the cache entry's ticket and no subsystem.
  internal::OutputPortPrerequisite DoGetPrerequisite() const final {
    return {nullopt, cache_entry().ticket()};
  };

  const CacheEntry* const cache_entry_;
};

}  // namespace systems
}  // namespace drake
