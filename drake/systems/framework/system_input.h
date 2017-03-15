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
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The InputPort describes a single input to a System. Users should not
/// subclass InputPort: all InputPorts are either DependentInputPorts or
/// FreestandingInputPorts.
class InputPort : public detail::OutputPortListenerInterface {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(InputPort)

  ~InputPort() override;

  /// Returns a positive number that increases monotonically, and changes
  /// whenever the data on this port changes, according to the source of
  /// that data.
  virtual int64_t get_version() const = 0;

  /// Returns true if this InputPort is not in control of its own data.
  virtual bool requires_evaluation() const = 0;

  /// Returns the data on this port, or nullptr if this port is not connected.
  const AbstractValue* get_abstract_data() const {
    DRAKE_DEMAND(get_output_port_value() != nullptr);
    return get_output_port_value()->get_abstract_data();
  }

  /// Returns the vector data on this port, or nullptr if this port is not
  /// connected. Throws std::bad_cast if the port is not vector-valued.
  ///
  /// @tparam T The type of the input port. Must be a valid Eigen scalar.
  template <typename T>
  const BasicVector<T>* get_vector_data() const {
    DRAKE_DEMAND(get_output_port_value() != nullptr);
    return get_output_port_value()->get_vector_data<T>();
  }

  /// Registers @p callback to be called whenever the value of get_version
  /// changes. The callback should invalidate data that depends on the value
  /// of this port, but should not do any substantive computation.
  void set_invalidation_callback(std::function<void()> callback) {
    invalidation_callback_ = callback;
  }

  /// Receives notification that the output port on which this InputPort
  /// depends has changed, and calls the invalidation_callback_.
  void Invalidate() override;

 protected:
  InputPort() {}

  virtual const OutputPortValue* get_output_port_value() const = 0;

 private:
  std::function<void()> invalidation_callback_ = nullptr;
};

/// A %DependentInputPort wraps a pointer to an OutputPortValue associated
/// with one System for use as an input to another System. Many
/// %DependentInputPort objects may wrap a single OutputPortValue.
class DependentInputPort : public InputPort {
 public:
  // DependentInputPort objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DependentInputPort)

  /// Creates an input port value source connected to the given
  /// @p output_port_value, which must not be nullptr. The OutputPortValue must
  /// outlive this %DependentInputPort object.
  explicit DependentInputPort(OutputPortValue* output_port_value);

  /// Disconnects from the output port.
  ~DependentInputPort() override;

  /// Sets the associated OutputPortValue to nullptr.
  void Disconnect() override;

  /// Returns the value version of the connected output port.
  int64_t get_version() const override {
    return output_port_value_->get_version();
  }

  /// A %DependentInputPort must be evaluated in a Context, because it does not
  /// control its own data.
  bool requires_evaluation() const override { return true; }

 protected:
  const OutputPortValue* get_output_port_value() const override {
    return output_port_value_;
  }

 private:
  OutputPortValue* output_port_value_{};
};

/// The FreestandingInputPort encapsulates a vector of data for use as the
/// value of a System's input port.
class FreestandingInputPort : public InputPort {
 public:
  // FreestandingInputPort objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FreestandingInputPort)

  /// Constructs a vector-valued %FreestandingInputPort.
  /// Takes ownership of @p vec.
  ///
  /// @tparam T The type of the vector data. Must be a valid Eigen scalar.
  /// @tparam V The type of @p vec itself. Must implement BasicVector<T>.
  template <template <typename T> class V, typename T>
  explicit FreestandingInputPort(std::unique_ptr<V<T>> vec)
      : output_port_value_(std::move(vec)) {
    output_port_value_.add_dependent(this);
  }

  /// Constructs an abstract-valued %FreestandingInputPort.
  /// Takes ownership of @p data.
  explicit FreestandingInputPort(std::unique_ptr<AbstractValue> data);

  /// Constructs an abstract-valued %FreestandingInputPort.
  /// Takes ownership of @p data.
  ///
  /// @tparam T The type of the data.
  template <typename T>
  explicit FreestandingInputPort(std::unique_ptr<Value<T>> data)
      : FreestandingInputPort(std::unique_ptr<AbstractValue>(data.release())) {}

  ~FreestandingInputPort() override;

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  int64_t get_version() const override {
    return output_port_value_.get_version();
  }

  /// A %FreestandingInputPort does not require evaluation, because it controls
  /// its own data.
  bool requires_evaluation() const override { return false; }

  /// Returns a pointer to the data inside this %FreestandingInputPort, and
  /// updates the version so that Contexts depending on this know to invalidate
  /// their caches.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write on the returned pointer if there is any
  /// possibility this %FreestandingInputPort has been accessed since the last
  /// time this method was called.
  AbstractValue* GetMutableData() {
    return output_port_value_.GetMutableData();
  }

  /// Returns a pointer to the data inside this %FreestandingInputPort, and
  /// updates the version so that Contexts depending on this know to invalidate
  /// their caches. Throws std::bad_cast if the data is not vector data.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write on the returned pointer if there is any
  /// possibility this %FreestandingInputPort has been accessed since the last
  /// time this method was called.
  ///
  /// @tparam T The type of the input port. Must be a valid Eigen scalar.
  template <typename T>
  BasicVector<T>* GetMutableVectorData() {
    return output_port_value_.GetMutableVectorData<T>();
  }

  /// Does nothing. A %FreestandingInputPort wraps its own OutputPortValue, so
  /// there is no need to handle unexpected destruction of an output port.
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
