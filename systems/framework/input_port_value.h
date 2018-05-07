#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/reset_on_copy.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class ContextBase;

/** A %FixedInputPortValue encapsulates a vector or abstract value for
use as an internal value source for one of a System's input ports. The semantics
are identical to a Parameter. We assign a DependencyTracker to this object
and subscribe the InputPort to it when that port is fixed. Any modification to
the value here issues a notification to its dependent, and increments a serial
number kept here. */
class FixedInputPortValue {
 public:
  /** @name  Does not allow move or assignment; copy is private. */
  /** @{ */
  FixedInputPortValue(FixedInputPortValue&&) = delete;
  FixedInputPortValue& operator=(const FixedInputPortValue&) = delete;
  FixedInputPortValue& operator=(FixedInputPortValue&&) = delete;
  /** @} */

  /** Constructs an abstract-valued %FixedInputPortValue from a value
  of arbitrary type. Takes ownership of the given value and sets the serial
  number to 1. */
  explicit FixedInputPortValue(std::unique_ptr<AbstractValue> value)
      : value_(std::move(value)), serial_number_{1} {}

  ~FixedInputPortValue() = default;

  /** Returns a reference to the contained abstract value; throws if there
  isn't one. */
  const AbstractValue& get_value() const {
    DRAKE_DEMAND(value_ != nullptr);
    return *value_;
  }

  /** Returns a reference to the contained `BasicVector<T>` or throws an
  exception if this doesn't contain an object of that type. */
  template <typename T>
  const BasicVector<T>& get_vector_value() const {
    return get_value().GetValue<BasicVector<T>>();
  }

  /** Returns a pointer to the data inside this %FixedInputPortValue, and
  notifies the dependent input port that the value has changed.

  To ensure invalidation notifications are delivered, callers should call this
  method every time they wish to update the stored value. In particular, callers
  MUST NOT write through the returned pointer if there is any possibility this
  %FixedInputPortValue has been accessed since the last time this method
  was called. */
  // TODO(sherm1) Replace these with safer Set() methods.
  AbstractValue* GetMutableData();

  /** Returns a pointer to the data inside this %FixedInputPortValue, and
  notifies the dependent input port that the value has changed, invalidating
  downstream computations. Throws std::bad_cast if the data is not vector data.

  To ensure invalidation notifications are delivered, callers should call this
  method every time they wish to update the stored value. In particular, callers
  MUST NOT write through the returned pointer if there is any possibility this
  %FixedInputPortValue has been accessed since the last time this method
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

  /** Returns the index of the input port for which this is the value. */
  InputPortIndex input_port_index() const {
    DRAKE_ASSERT(index_.is_valid());
    return index_;
  }

  /** Returns a const reference to the context that owns this object. */
  const ContextBase& get_owning_context() const {
    DRAKE_ASSERT(owning_subcontext_ != nullptr);
    return *owning_subcontext_;
  }

  /** Returns a mutable reference to the context that owns this object. */
  ContextBase& get_mutable_owning_context() {
    DRAKE_ASSERT(owning_subcontext_ != nullptr);
    return *owning_subcontext_;
  }

  /** (Internal use only) */
  // Informs this FixedInputPortValue of its assigned DependencyTracker
  // so it knows who to notify when its value changes.
  void set_ticket(DependencyTicket ticket) { ticket_ = ticket; }

  /** (Internal use only) */
  // Informs this %FixedInputPortValue of the  the input port within
  // its owning subcontext for which it is the value. Aborts if
  // this has already been done or given bad args.
  void set_input_port_index(InputPortIndex index) {
    DRAKE_DEMAND(index.is_valid() && !index_.is_valid());
    index_ = index;
  }

  /** (Internal use only) */
  // Informs this %FixedInputPortValue of the subcontext that owns it.
  // Aborts if this has already been done or given bad args.
  void set_owning_subcontext(ContextBase* owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr && owning_subcontext_ == nullptr);
    owning_subcontext_ = owning_subcontext;
  }

  /** (Internal use only) */
  // Makes a copy of this %FixedInputPortValue to be used in a new
  // context. The copy must have the same index as the source; we pass it here
  // just to validate that. The value, ticket, and serial number are preserved.
  std::unique_ptr<FixedInputPortValue> CloneForNewContext(
      ContextBase* new_owner, InputPortIndex new_index) const {
    DRAKE_DEMAND(new_owner != nullptr && new_index.is_valid());
    DRAKE_DEMAND(new_owner != owning_subcontext_ && new_index == index_);
    auto clone = std::unique_ptr<FixedInputPortValue>(
        new FixedInputPortValue(*this));
    clone->set_owning_subcontext(new_owner);
    return clone;
  }

  /** (Internal use only) */
  // Copy constructor is only used for cloning and is not a complete copy --
  // owning_subcontext_ is left unassigned.
  FixedInputPortValue(const FixedInputPortValue& source) =
      default;

 private:
  // Needed for invalidation.
  reset_on_copy<ContextBase*> owning_subcontext_;
  InputPortIndex index_;

  // The value and its serial number.
  copyable_unique_ptr<AbstractValue> value_;
  int64_t serial_number_{-1};  // Currently just for debugging use.

  // Index of the dependency tracker for this fixed value. The input
  // port should have registered with this tracker.
  DependencyTicket ticket_;
};


/** (Deprecated name) Please use FixedInputPortValue instead. */
// TODO(sherm1) Deprecate this for real.
using FreestandingInputPortValue = FixedInputPortValue;

}  // namespace systems
}  // namespace drake
