#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/cache_entry.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/value_producer.h"

namespace drake {
namespace systems {

// TODO(sherm1) Output ports that simply expose existing Context objects
// should not require a cache entry. Add provision for that to avoid the
// unnecessary copying currently done by Eval() for those.
/** (Advanced.) Implements an output port whose value is managed by a cache
entry in the same LeafSystem as the port. This is intended for internal use in
implementing the DeclareOutputPort() variants in LeafSystem.

@tparam_default_scalar
*/
template <typename T>
class LeafOutputPort final : public OutputPort<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafOutputPort)

  ~LeafOutputPort() final = default;

  // TODO(jwnimmer-tri) It's likely that nixing these output-specific callback
  // function types in favor of more ValueProducer sugar would lead to clearer
  // code and the ability to further reduce the computational overhead of the
  // systems framework. For the moment, though, they are heaviliy used and
  // deprecating them without a specific plan for follow-up improvements is
  // a lot of churn for little gain. When we next revisit this question, we
  // should look into replacing their use of Context<T> with ContextBase, and
  // use of BasicVector<T> with drake::VectorX<T>.

  /** Signature of a function suitable for allocating an object that can hold
  a value of a particular output port. The result is returned as an
  AbstractValue even if this is a vector-valued port. */
  using AllocCallback = ValueProducer::AllocateCallback;

  /** Signature of a function suitable for calculating a value of a particular
  output port, given a place to put the value. */
  using CalcCallback =
  std::function<void(const Context<T>&, AbstractValue*)>;

  /** Signature of a function suitable for calculating a value of a particular
  vector-valued output port, given a place to put the value. */
  using CalcVectorCallback =
  std::function<void(const Context<T>&, BasicVector<T>*)>;

  /** Returns the cache entry associated with this output port. */
  const CacheEntry& cache_entry() const {
    DRAKE_ASSERT(cache_entry_ != nullptr);
    return *cache_entry_;
  }

  /** (Debugging) Specifies that caching should be disabled for this output
  port when a Context is first allocated. This is useful if you have observed
  different behavior with caching on or off and would like to determine if
  the problem is caused by this port.
  @see CacheEntry::disable_caching_by_default() */
  void disable_caching_by_default() {
    cache_entry_->disable_caching_by_default();
  }

 private:
  friend class internal::FrameworkFactory;

  // Constructs a cached output port. The `system` parameter must be the same
  // object as the `system_interface` parameter.
  LeafOutputPort(const System<T>* system,
                 internal::SystemMessageInterface* system_interface,
                 internal::SystemId system_id,
                 std::string name, OutputPortIndex index,
                 DependencyTicket ticket, PortDataType data_type, int size,
                 CacheEntry* cache_entry)
      : OutputPort<T>(system, system_interface, system_id, std::move(name),
                      index, ticket, data_type, size),
        cache_entry_(cache_entry) {
    DRAKE_DEMAND(cache_entry != nullptr);
  }

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
    return {std::nullopt, cache_entry().ticket()};
  };

  // Check that an AbstractValue provided to Calc() is suitable for this port
  // by comparing it with the port's cache entry value type.
  void ThrowIfInvalidPortValueType(
      const Context<T>& context,
      const AbstractValue& proposed_value) const final;

  CacheEntry* const cache_entry_;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafOutputPort)
