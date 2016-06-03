#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The InputPort describes a single input to a System, from another
/// System or from an external driver.
///
/// @tparam T The type of the input port. Must be a valid Eigen scalar.
template <typename T>
class InputPort : public OutputPortListenerInterface {
 public:
  virtual ~InputPort() {}

  /// Returns a positive number that increases monotonically, and changes
  /// whenever the data on this port changes, according to the source of
  /// that data.
  virtual int64_t get_version() const = 0;

  /// Returns the sampling interval of this port in seconds, or zero if
  /// this port is continuous.
  virtual double get_sample_time_sec() const = 0;

  /// Returns the vector data on this port, or nullptr if this port is not
  /// vector-valued or not connected. Implementations must ensure that
  /// get_vector_data is O(1) and initiates no substantive computation.
  virtual const VectorInterface<T>* get_vector_data() const = 0;

  /// Registers @p callback to be called whenever the value of get_version
  /// changes. The callback should invalidate data that depends on the value
  /// of this port, but should not do any substantive computation.
  void set_invalidation_callback(std::function<void()> callback) {
    invalidation_callback_ = callback;
  }

  /// Receives notification that the output port on which this InputPort
  /// depends has changed, and calls the invalidation_callback_.
  void Invalidate() override {
    if (invalidation_callback_ != nullptr) {
      invalidation_callback_();
    }
  }

 protected:
  InputPort() {}

 private:
  std::function<void()> invalidation_callback_ = nullptr;
};

/// The ConnectedInputPort wraps a pointer to the OutputPort of a System for use
/// as an input to another System. Many ConnectedInputPorts may wrap a single
/// OutputPort.
///
/// @tparam T The type of the input port. Must be a valid Eigen scalar.
template <typename T>
class ConnectedInputPort : public InputPort<T> {
 public:
  /// Creates an input port with the given @p sample_time_sec, connected
  /// to the given @p output_port, which must not be nullptr. The output
  /// port must outlive this input port.
  ConnectedInputPort(OutputPort<T>* output_port, double sample_time_sec)
      : output_port_(output_port), sample_time_sec_(sample_time_sec) {
    output_port_->add_dependent(this);
  }

  ~ConnectedInputPort() override {
    output_port_->remove_dependent(this);
  }

  /// Returns the value version of the connected output port.
  int64_t get_version() const override {
    return output_port_->get_version();
  }

  double get_sample_time_sec() const override {
    return sample_time_sec_;
  }

  const VectorInterface<T>* get_vector_data() const override {
    return output_port_->get_vector_data();
  }

 private:
  // ConnectedInputPort objects are neither copyable nor moveable.
  ConnectedInputPort(const ConnectedInputPort& other) = delete;
  ConnectedInputPort& operator=(const ConnectedInputPort& other) = delete;
  ConnectedInputPort(ConnectedInputPort&& other) = delete;
  ConnectedInputPort& operator=(ConnectedInputPort&& other) = delete;

  OutputPort<T>* output_port_;
  double sample_time_sec_;
};

/// The FreestandingInputPort encapsulates a vector of data for use as an input
/// to a System.
///
/// @tparam T The type of the input port. Must be a valid Eigen scalar.
template <typename T>
class FreestandingInputPort : public InputPort<T> {
 public:
  /// Constructs a continuous FreestandingInputPort.
  /// Takes ownership of @p vector_data.
  explicit FreestandingInputPort(
      std::unique_ptr<VectorInterface<T>> vector_data)
      : FreestandingInputPort(std::move(vector_data), 0.0 /* continuous */) {}

  /// Constructs a FreestandingInputPort with the given sample rate
  /// @p sample_time_sec, which should be zero for continuous ports.
  /// Takes ownership of @p vector_data.
  FreestandingInputPort(std::unique_ptr<VectorInterface<T>> vector_data,
                        double sample_time_sec)
      : output_port_(std::move(vector_data)),
        sample_time_sec_(sample_time_sec) {
    output_port_.add_dependent(this);
  }

  virtual ~FreestandingInputPort() {
    output_port_.remove_dependent(this);
  }

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  int64_t get_version() const override {
    return output_port_.get_version();
  }

  double get_sample_time_sec() const override {
    return sample_time_sec_;
  }

  const VectorInterface<T>* get_vector_data() const override {
    return output_port_.get_vector_data();
  }

  /// Returns a pointer to the data inside this InputPort, and updates the
  /// version so that Contexts depending on this InputPort know to invalidate
  /// their caches.
  ///
  /// To ensure invalidation notifications are delivered, callers should
  /// call this method every time they wish to update the stored value.  In
  /// particular, callers MUST NOT write on the returned pointer if there is any
  /// possibility this FreestandingInputPort has been accessed since the last
  /// time this method was called.
  VectorInterface<T>* GetMutableVectorData() {
    return output_port_.GetMutableVectorData();
  }

 private:
  // FreestandingInputPort objects are neither copyable nor moveable.
  FreestandingInputPort(const FreestandingInputPort& other) = delete;
  FreestandingInputPort& operator=(const FreestandingInputPort& other) = delete;
  FreestandingInputPort(FreestandingInputPort&& other) = delete;
  FreestandingInputPort& operator=(FreestandingInputPort&& other) = delete;

  OutputPort<T> output_port_;
  double sample_time_sec_ = 0.0;
};

}  // namespace systems
}  // namespace drake
