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
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

class ContextBase;

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {
// This provides ContextBase limited "friend" access to FixedInputPortValue.
class ContextBaseFixedInputAttorney;
}  // namespace detail
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

 private:
  friend class detail::ContextBaseFixedInputAttorney;

  // Allow this adapter access to our private copy constructor. This is intended
  // only for use by ContextBase.
  friend class copyable_unique_ptr<FixedInputPortValue>;

  // Copy constructor is only used for cloning and is not a complete copy --
  // owning_subcontext_ is left unassigned.
  FixedInputPortValue(const FixedInputPortValue& source) =
      default;

  // Informs this FixedInputPortValue of the  the input port within
  // its owning subcontext for which it is the value. Aborts if
  // this has already been done or given bad args.
  void set_input_port_index(InputPortIndex index) {
    DRAKE_DEMAND(index.is_valid() && !index_.is_valid());
    index_ = index;
  }

  // Informs this FixedInputPortValue of the subcontext that owns it.
  // Aborts if this has already been done or given bad args.
  void set_owning_subcontext(ContextBase* owning_subcontext) {
    DRAKE_DEMAND(owning_subcontext != nullptr && owning_subcontext_ == nullptr);
    owning_subcontext_ = owning_subcontext;
  }

  // Needed for invalidation.
  reset_on_copy<ContextBase*> owning_subcontext_;
  InputPortIndex index_;

  // The value and its serial number.
  copyable_unique_ptr<AbstractValue> value_;
  int64_t serial_number_{-1};  // Currently just for debugging use.
};

// TODO(sherm1) Get rid of this after 8/7/2018 (three months).
DRAKE_DEPRECATED("Please use FixedInputPortValue instead.")
typedef FixedInputPortValue FreestandingInputPortValue;

#ifndef DRAKE_DOXYGEN_CXX
namespace detail {

class ContextBaseFixedInputAttorney {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContextBaseFixedInputAttorney);
  ContextBaseFixedInputAttorney() = delete;

 private:
  friend class drake::systems::ContextBase;

  static void set_input_port_index(InputPortIndex index,
                                   FixedInputPortValue* fixed) {
    DRAKE_DEMAND(fixed != nullptr);
    fixed->set_input_port_index(index);
  }

  static void set_owning_subcontext(ContextBase* owning_subcontext,
                                    FixedInputPortValue* fixed) {
    DRAKE_DEMAND(owning_subcontext != nullptr && fixed != nullptr);
    fixed->set_owning_subcontext(owning_subcontext);
  }
};

}  // namespace detail
#endif

}  // namespace systems
}  // namespace drake
