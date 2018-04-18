#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/output_port_value.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// %InputPortValue identifies the value source for a single System input port.
/// Objects of this type are contained in the System's Context for use when the
/// value of that input port is needed. Users should not subclass
/// %InputPortValue; these will always be of one of two predefined derived
/// types:
/// - DependentInputPortValue, meaning the value source for this input port is
///   an output port of another System, or
/// - FreestandingInputPortValue, meaning the value source is an independent
///   object contained internally.
class InputPortValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortValue)

  virtual ~InputPortValue();

  /// Returns true if this InputPortValue is not in control of its own data.
  virtual bool requires_evaluation() const = 0;

  /// Returns the data on this port, which must be connected to a value source.
  const AbstractValue* get_abstract_data() const {
    DRAKE_DEMAND(get_output_port_value() != nullptr);
    return get_output_port_value()->get_abstract_data();
  }

  /// Returns the vector data on this port, which must be connected to a value
  /// source. Throws std::bad_cast if the port is not vector-valued.
  ///
  /// @tparam T The type of the input port. Must be a valid Eigen scalar.
  template <typename T>
  const BasicVector<T>* get_vector_data() const {
    DRAKE_DEMAND(get_output_port_value() != nullptr);
    return get_output_port_value()->get_vector_data<T>();
  }

 protected:
  InputPortValue() {}

  virtual const OutputPortValue* get_output_port_value() const = 0;
};

/// A %DependentInputPortValue wraps a pointer to an OutputPortValue associated
/// with one System for use as an input to another System. Many
/// %DependentInputPortValue objects may wrap a single OutputPortValue.
class DependentInputPortValue : public InputPortValue {
 public:
  // DependentInputPortValue objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DependentInputPortValue)

  /// Creates an input port value source connected to the given
  /// @p output_port_value, which must not be nullptr. The OutputPortValue must
  /// outlive this %DependentInputPortValue object.
  explicit DependentInputPortValue(OutputPortValue* output_port_value);

  /// A %DependentInputPortValue must be evaluated in a Context, because it does
  /// not control its own data.
  bool requires_evaluation() const override { return true; }

 protected:
  const OutputPortValue* get_output_port_value() const override {
    return output_port_value_;
  }

 private:
  OutputPortValue* output_port_value_{};
};

/// A %FreestandingInputPortValue encapsulates a vector or abstract value for
/// use as an internal value source for one of a System's input ports.
class FreestandingInputPortValue : public InputPortValue {
 public:
  // FreestandingInputPortValue objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreestandingInputPortValue)

  /// Constructs an abstract-valued %FreestandingInputPortValue from a value
  /// of arbitrary type. Takes ownership of @p data.
  explicit FreestandingInputPortValue(std::unique_ptr<AbstractValue> data);

  /// A %FreestandingInputPortValue does not require evaluation, because it
  /// controls its own data.
  bool requires_evaluation() const override { return false; }

  /// Returns a pointer to the data inside this %FreestandingInputPortValue, and
  /// updates the version so that Contexts depending on this know to invalidate
  /// their caches.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write through the returned pointer if there
  /// is any possibility this %FreestandingInputPortValue has been accessed
  /// since the last time this method was called.
  AbstractValue* GetMutableData() {
    return output_port_value_.GetMutableData();
  }

  /// Returns a pointer to the data inside this %FreestandingInputPortValue, and
  /// updates the version so that Contexts depending on this know to invalidate
  /// their caches. Throws std::bad_cast if the data is not vector data.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write through the returned pointer if there
  /// is any possibility this %FreestandingInputPortValue has been accessed
  /// since the last time this method was called.
  ///
  /// @tparam T Element type of the input port's vector value. Must be a valid
  ///           Eigen scalar.
  template <typename T>
  BasicVector<T>* GetMutableVectorData() {
    return output_port_value_.GetMutableVectorData<T>();
  }

 protected:
  const OutputPortValue* get_output_port_value() const override {
    return &output_port_value_;
  }

 private:
  OutputPortValue output_port_value_;
};

}  // namespace systems
}  // namespace drake
