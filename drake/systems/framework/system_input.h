#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// The InputPort describes a single input to a System. Users should not
/// subclass InputPort: all InputPorts are either DependentInputPorts or
/// FreestandingInputPorts.
class DRAKESYSTEMFRAMEWORK_EXPORT InputPort
    : public OutputPortListenerInterface {
 public:
  ~InputPort() override;

  /// Returns a positive number that increases monotonically, and changes
  /// whenever the data on this port changes, according to the source of
  /// that data.
  virtual int64_t get_version() const = 0;

  /// Returns the data on this port, or nullptr if this port is not connected.
  const AbstractValue* get_abstract_data() const {
    return get_output_port()->get_abstract_data();
  }

  /// Returns the vector data on this port, or nullptr if this port is not
  /// connected. Throws std::bad_cast if the port is not vector-valued.
  ///
  /// @tparam T The type of the input port. Must be a valid Eigen scalar.
  template <typename T>
  const VectorBase<T>* get_vector_data() const {
    return get_output_port()->get_vector_data<T>();
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

  virtual const OutputPort* get_output_port() const = 0;

 private:
  std::function<void()> invalidation_callback_ = nullptr;
};

/// The DependentInputPort wraps a pointer to the OutputPort of a System for use
/// as an input to another System. Many DependentInputPorts may wrap a single
/// OutputPort.
class DRAKESYSTEMFRAMEWORK_EXPORT DependentInputPort : public InputPort {
 public:
  /// Creates an input port connected to the given @p output_port, which
  /// must not be nullptr. The output port must outlive this input port.
  explicit DependentInputPort(OutputPort* output_port)
      : output_port_(output_port) {
    output_port_->add_dependent(this);
  }

  /// Disconnects from the output port.
  ~DependentInputPort() override;

  /// Returns the value version of the connected output port.
  int64_t get_version() const override { return output_port_->get_version(); }

 protected:
  const OutputPort* get_output_port() const override { return output_port_; }

 private:
  // DependentInputPort objects are neither copyable nor moveable.
  DependentInputPort(const DependentInputPort& other) = delete;
  DependentInputPort& operator=(const DependentInputPort& other) = delete;
  DependentInputPort(DependentInputPort&& other) = delete;
  DependentInputPort& operator=(DependentInputPort&& other) = delete;

  OutputPort* output_port_;
};

/// The FreestandingInputPort encapsulates a vector of data for use as an input
/// to a System.
class DRAKESYSTEMFRAMEWORK_EXPORT FreestandingInputPort : public InputPort {
 public:
  /// Constructs a vector-valued FreestandingInputPort.
  /// Takes ownership of @p vec.
  ///
  /// @tparam T The type of the vector data. Must be a valid Eigen scalar.
  /// @tparam V The type of @p vec itself. Must implement VectorBase<T>.
  template <template <typename T> class V, typename T>
  explicit FreestandingInputPort(std::unique_ptr<V<T>> vec)
      : output_port_(std::move(vec)) {
    output_port_.add_dependent(this);
  }

  /// Constructs an abstract-valued FreestandingInputPort.
  /// Takes ownership of @p data.
  explicit FreestandingInputPort(std::unique_ptr<AbstractValue> data)
      : output_port_(std::move(data)) {
    output_port_.add_dependent(this);
  }

  /// Constructs an abstract-valued FreestandingInputPort.
  /// Takes ownership of @p data.
  ///
  /// @tparam T The type of the data.
  template <typename T>
  explicit FreestandingInputPort(std::unique_ptr<Value<T>> data)
      : output_port_(std::move(data)) {
    output_port_.add_dependent(this);
  }

  ~FreestandingInputPort() override;

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  int64_t get_version() const override { return output_port_.get_version(); }

  /// Returns a pointer to the data inside this InputPort, and updates the
  /// version so that Contexts depending on this InputPort know to invalidate
  /// their caches.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write on the returned pointer if there is any
  /// possibility this FreestandingInputPort has been accessed since the last
  /// time this method was called.
  AbstractValue* GetMutableData() { return output_port_.GetMutableData(); }

  /// Returns a pointer to the data inside this InputPort, and updates the
  /// version so that Contexts depending on this InputPort know to invalidate
  /// their caches. Throws std::bad_cast if the data is not vector data.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write on the returned pointer if there is any
  /// possibility this FreestandingInputPort has been accessed since the last
  /// time this method was called.
  ///
  /// @tparam T The type of the input port. Must be a valid Eigen scalar.
  template <typename T>
  VectorBase<T>* GetMutableVectorData() {
    return output_port_.GetMutableVectorData<T>();
  }

 protected:
  const OutputPort* get_output_port() const override { return &output_port_; }

 private:
  // FreestandingInputPort objects are neither copyable nor moveable.
  FreestandingInputPort(const FreestandingInputPort& other) = delete;
  FreestandingInputPort& operator=(const FreestandingInputPort& other) = delete;
  FreestandingInputPort(FreestandingInputPort&& other) = delete;
  FreestandingInputPort& operator=(FreestandingInputPort&& other) = delete;

  OutputPort output_port_;
};

}  // namespace systems
}  // namespace drake
