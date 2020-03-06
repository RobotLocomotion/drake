#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/reset_on_copy.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

class ContextBase;

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
// This provides ContextBase limited "friend" access to FixedInputPortValue.
class ContextBaseFixedInputAttorney;
}  // namespace internal
#endif

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

  // Construction is private and only accessible to ContextBase via the
  // attorney.

  ~FixedInputPortValue() = default;

  /** Returns a reference to the contained abstract value. */
  const AbstractValue& get_value() const {
    DRAKE_DEMAND(value_ != nullptr);  // Should always be a value.
    return *value_;
  }

  /** Returns a reference to the contained `BasicVector<T>` or throws an
  exception if this doesn't contain an object of that type. */
  template <typename T>
  const BasicVector<T>& get_vector_value() const {
    return get_value().get_value<BasicVector<T>>();
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
  downstream computations.
  @throws std::bad_cast if the data is not vector data.

  To ensure invalidation notifications are delivered, callers should call this
  method every time they wish to update the stored value. In particular, callers
  MUST NOT write through the returned pointer if there is any possibility this
  %FixedInputPortValue has been accessed since the last time this method
  was called.

  @tparam T Scalar type of the input port's vector value. Must match the type
            associated with this port. */
  template <typename T>
  BasicVector<T>* GetMutableVectorData() {
    return &GetMutableData()->get_mutable_value<BasicVector<T>>();
  }

  /** Returns the serial number of the contained value. This counts up every
  time the contained value changes, or when mutable access is granted. */
  int64_t serial_number() const { return serial_number_; }

  /** Returns the ticket used to find the associated DependencyTracker. */
  DependencyTicket ticket() const {
    DRAKE_ASSERT(ticket_.is_valid());
    return ticket_;
  }

  /** Returns a const reference to the context that owns this object. */
  const ContextBase& get_owning_context() const {
    DRAKE_ASSERT(owning_subcontext_ != nullptr);
    return *owning_subcontext_;
  }

 private:
  friend class internal::ContextBaseFixedInputAttorney;

  // Allow this adapter access to our private copy constructor. This is intended
  // only for use by ContextBase.
  friend class copyable_unique_ptr<FixedInputPortValue>;

  // Constructs an abstract-valued FixedInputPortValue from a value
  // of arbitrary type. Takes ownership of the given value and sets the serial
  // number to 1. The value must not be null.
  explicit FixedInputPortValue(std::unique_ptr<AbstractValue> value)
      : value_(std::move(value)), serial_number_{1} {
    DRAKE_DEMAND(value_ != nullptr);
  }

  // Copy constructor is only used for cloning and is not a complete copy --
  // owning_subcontext_ is left unassigned.
  FixedInputPortValue(const FixedInputPortValue& source) = default;

  // Informs this FixedInputPortValue of its assigned DependencyTracker
  // so it knows who to notify when its value changes.
  void set_ticket(DependencyTicket ticket) {
    DRAKE_DEMAND(ticket.is_valid() && !ticket_.is_valid());
    ticket_ = ticket;
  }

  // Informs this %FixedInputPortValue of the subcontext that owns it.
  // Aborts if this has already been done or given bad args.
  void set_owning_subcontext(ContextBase* owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr && owning_subcontext_ == nullptr);
    owning_subcontext_ = owning_subcontext;
  }

  // Needed for invalidation.
  reset_on_copy<ContextBase*> owning_subcontext_;

  // The value and its serial number.
  copyable_unique_ptr<AbstractValue> value_;

  // The serial number is useful for debugging and counting changes but has
  // no role in cache invalidation. Note that after a Context is cloned, both
  // the value and serial number are copied -- the clone serial number does
  // not reset. That avoids accidental serial number matches if someone has
  // recorded the number somewhere. If the serial number matches, the value
  // is either unchanged since you last saw it, or an identical copy.
  int64_t serial_number_{-1};

  // Index of the dependency tracker for this fixed value. The input port
  // should have registered with this tracker.
  DependencyTicket ticket_;
};

#ifndef DRAKE_DOXYGEN_CXX
namespace internal {

class ContextBaseFixedInputAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContextBaseFixedInputAttorney);
  ContextBaseFixedInputAttorney() = delete;

 private:
  friend class drake::systems::ContextBase;

  // "Output" argument is first here since it is serving as a `this` pointer.
  static void set_owning_subcontext(FixedInputPortValue* fixed,
                                    ContextBase* owning_subcontext) {
    DRAKE_DEMAND(fixed != nullptr);
    fixed->set_owning_subcontext(owning_subcontext);
  }

  static void set_ticket(FixedInputPortValue* fixed,
                         DependencyTicket ticket) {
    DRAKE_DEMAND(fixed != nullptr);
    fixed->set_ticket(ticket);
  }

  // This serves as the only accessible constructor for FixedInputPortValues.
  // It must be followed immediately by inserting into a Context with the
  // assigned ticket and the owning subcontext set using the above methods.
  static std::unique_ptr<FixedInputPortValue> CreateFixedInputPortValue(
      std::unique_ptr<AbstractValue> value) {
    // Can't use make_unique here since constructor is private.
    return std::unique_ptr<FixedInputPortValue>(
        new FixedInputPortValue(std::move(value)));
  }
};

}  // namespace internal
#endif

}  // namespace systems
}  // namespace drake
