#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/input_port_evaluator_interface.h"
#include "drake/systems/framework/output_port_listener_interface.h"
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
class InputPortValue : public detail::OutputPortListenerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPortValue)

  ~InputPortValue() override;

  /// Returns a positive number that increases monotonically, and changes
  /// whenever the data on this port changes, according to the source of
  /// that data.
  virtual int64_t get_version() const = 0;

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

  /// Registers @p callback to be called whenever the value source on which this
  /// %InputPortValue depends. The callback should invalidate data that depends
  /// on the value of this port, but should not do any substantive computation.
  void set_invalidation_callback(std::function<void()> callback) {
    invalidation_callback_ = callback;
  }

  /// Receives notification that the value source on which this %InputPortValue
  /// depends has changed, and calls the invalidation callback.
  void Invalidate() override;

 protected:
  InputPortValue() {}

  virtual const OutputPortValue* get_output_port_value() const = 0;

 private:
  std::function<void()> invalidation_callback_ = nullptr;
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

  /// Disconnects from the output port.
  ~DependentInputPortValue() override;

  /// Sets the associated OutputPortValue to nullptr.
  void Disconnect() override;

  /// Returns the value version of the connected output port.
  int64_t get_version() const override {
    return output_port_value_->get_version();
  }

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

  /// Constructs a vector-valued %FreestandingInputPortValue.
  /// Takes ownership of @p vec.
  ///
  /// @tparam T The type of the vector data. Must be a valid Eigen scalar.
  /// @tparam V The type of @p vec itself. Must implement BasicVector<T>.
  template <template <typename T> class V, typename T>
  explicit FreestandingInputPortValue(std::unique_ptr<V<T>> vec)
      : output_port_value_(std::move(vec)) {
    output_port_value_.add_dependent(this);
  }

  /// Constructs an abstract-valued %FreestandingInputPortValue from a value
  /// of unknown type. Takes ownership of @p data.
  explicit FreestandingInputPortValue(std::unique_ptr<AbstractValue> data);

  /// Constructs an abstract-valued %FreestandingInputPortValue from a value
  /// of currently-known type. This will become a type-erased AbstractValue
  /// here but a knowledgeable caller can recover the original typed object
  /// using `dynamic_cast`. Takes ownership of @p data.
  ///
  /// @tparam T The type of the data.
  template <typename T>
  explicit FreestandingInputPortValue(std::unique_ptr<Value<T>> data)
      : FreestandingInputPortValue(
            std::unique_ptr<AbstractValue>(data.release())) {}

  ~FreestandingInputPortValue() override;

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  int64_t get_version() const override {
    return output_port_value_.get_version();
  }

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

  /// Does nothing. A %FreestandingInputPortValue wraps its own OutputPortValue,
  /// so there is no need to handle destruction of an output port.
  void Disconnect() override {}

 protected:
  const OutputPortValue* get_output_port_value() const override {
    return &output_port_value_;
  }

 private:
  OutputPortValue output_port_value_;
};

}  // namespace systems
}  // namespace drake
