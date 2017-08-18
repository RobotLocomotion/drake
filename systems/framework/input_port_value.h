#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class ContextBase;

//==============================================================================
//                     FREESTANDING INPUT PORT VALUE
//==============================================================================
/** A %FreestandingInputPortValue encapsulates a vector or abstract value for
use as an internal value source for one of a System's input ports. The semantics
are identical to a Parameter. We assign a DependencyTracker to this object
and subscribe the InputPort to it when that port is fixed. Any modification to
the value here issues a notification to its dependent, and increments a serial
number kept here. */
class FreestandingInputPortValue {
 public:
  /** @name  Does not allow move or assignment; copy is private. */
  /** @{ */
  FreestandingInputPortValue(FreestandingInputPortValue&&) =
      delete;
  FreestandingInputPortValue& operator=(const FreestandingInputPortValue&) =
      delete;
  FreestandingInputPortValue& operator=(FreestandingInputPortValue&&) =
      delete;
  /** @} */

  /** Constructs an abstract-valued %FreestandingInputPortValue from a value
  of unknown type. Takes ownership of the given value and sets the serial
  number to 1. */
  explicit FreestandingInputPortValue(std::unique_ptr<AbstractValue> value)
      : value_(std::move(value)), serial_number_{1} {}

  /** Constructs a vector-valued %FreestandingInputPortValue using a given
  vector object. Takes ownership of the vector and sets serial number to 1.
  @tparam T The type of the vector data. Must be a valid Eigen scalar.
  @tparam V The type of `vec` itself. Must implement BasicVector<T>. */
  template <template <typename T> class V, typename T>
  explicit FreestandingInputPortValue(std::unique_ptr<V<T>> vec)
      : FreestandingInputPortValue(
            std::make_unique<Value<BasicVector<T>>>(std::move(vec))) {
    static_assert(std::is_base_of<BasicVector<T>, V<T>>::value,
                  "Expected vector type derived from BasicVector.");
  }

  /** Constructs an abstract-valued %FreestandingInputPortValue from a value
  of currently-known type. This will become a type-erased AbstractValue
  here but a knowledgeable caller can recover the original typed object
  using `dynamic_cast`. Takes ownership of `value` and sets serial number to 1.
  @tparam T The type of the data. Must be copyable or cloneable. */
  template <typename T>
  explicit FreestandingInputPortValue(std::unique_ptr<Value<T>> value)
      : FreestandingInputPortValue(
            std::unique_ptr<AbstractValue>(value.release())) {}

  ~FreestandingInputPortValue() = default;

  /** Returns a reference to the contained abstract value; throws if there
  isn't one. */
  const AbstractValue& get_value() const {
    DRAKE_DEMAND(value_ != nullptr);
    return *value_;
  }

  /** Returns a reference to the contained `BasicVector<T>` or throw an
  exception if this doesn't contain an object of that type. */
  template <typename T>
  const BasicVector<T>& get_vector_value() const {
    return get_value().GetValue<BasicVector<T>>();
  }

  /** Returns a pointer to the data inside this %FreestandingInputPortValue, and
  notifies the dependent input port that the value has changed.

  To ensure invalidation notifications are delivered, callers should call this
  method every time they wish to update the stored value. In particular, callers
  MUST NOT write through the returned pointer if there is any possibility this
  %FreestandingInputPortValue has been accessed since the last time this method
  was called. */
  // TODO(sherm1) Replace these with safer Set() methods.
  AbstractValue* GetMutableData();

  /** Returns a pointer to the data inside this %FreestandingInputPortValue, and
  notifies the dependent input port that the value has changed, invalidating
  downstream computations. Throws std::bad_cast if the data is not vector data.

  To ensure invalidation notifications are delivered, callers should call this
  method every time they wish to update the stored value. In particular, callers
  MUST NOT write through the returned pointer if there is any possibility this
  %FreestandingInputPortValue has been accessed since the last time this method
  was called.

  @tparam T Element type of the input port's vector value. Must be a valid
            Eigen scalar. */
  template <typename T>
  BasicVector<T>* GetMutableVectorData() {
    return &GetMutableData()->GetMutableValueOrThrow<BasicVector<T>>();
  }

  /** Returns the serial number of the contained value. This counts up every
  time the contained value changes, or when mutable access is granted. */
  int64_t serial_number() const { return serial_number_; }

  /** Returns the ticket used to find the associated DependencyTracker. */
  DependencyTicket ticket() const { return ticket_; }

  /** Returns index of the input port for which this is the value. */
  InputPortIndex input_port_index() const {
    DRAKE_ASSERT(index_.is_valid());
    return index_;
  }

  /** Returns a const reference to the context that owns this object. */
  const ContextBase& get_owning_context() const {
    DRAKE_ASSERT(owner_ != nullptr);
    return *owner_;
  }

  /** Returns a mutable reference to the context that owns this object. */
  ContextBase& get_mutable_owning_context() {
    DRAKE_ASSERT(owner_ != nullptr);
    return *owner_;
  }

  /** Informs this %FreestandingInputPortValue of its assigned DependencyTracker
  so it knows who to notify when its value changes. */
  void set_ticket(DependencyTicket ticket) { ticket_ = ticket; }

  /** Informs this %FreestandingInputPortValue of the context that owns
  it and the input port within that context for which it is the value. */
  void set_owning_context(ContextBase* context, InputPortIndex index) {
    DRAKE_DEMAND(owner_ == nullptr && !index_.is_valid());
    DRAKE_DEMAND(context != nullptr && index.is_valid());
    owner_ = context;
    index_ = index;
  }

  /** (Internal use only) Makes a copy of this %FreestandingInputPortValue
  to be used in a new context. The copy must have the same index as the source;
  we pass it here just to validate that. The value, ticket, and serial number
  are preserved. */
  std::unique_ptr<FreestandingInputPortValue> CloneForNewContext(
      ContextBase* new_owner, InputPortIndex new_index) const {
    DRAKE_DEMAND(new_owner != nullptr && new_index.is_valid());
    DRAKE_DEMAND(new_owner != owner_ && new_index == index_);
    auto clone = std::unique_ptr<FreestandingInputPortValue>(
        new FreestandingInputPortValue(*this));
    clone->set_owning_context(new_owner, new_index);
    return clone;
  }

 private:
  // Copy constructor is only used for cloning and is not a complete copy --
  // we leave owner_ and index_ unassigned.
  FreestandingInputPortValue(const FreestandingInputPortValue& source)
      : value_(source.value_->Clone()),
        serial_number_(source.serial_number()),
        ticket_(source.ticket()) {}

  // Needed for invalidation.
  ContextBase* owner_{nullptr};
  InputPortIndex index_;

  // The value and its serial number.
  std::unique_ptr<AbstractValue> value_;
  int64_t serial_number_{-1};  // Currently just for debugging use.

  // Index of the dependency tracker for this freestanding value. The input
  // port should have registered with this tracker.
  DependencyTicket ticket_;
};

}  // namespace systems
}  // namespace drake
