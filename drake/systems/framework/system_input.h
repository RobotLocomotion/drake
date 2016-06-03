#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "drake/systems/framework/system_output.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// The InputPortInterface describes a single input to a System, from another
/// System or from an external driver.
///
/// @tparam T The type of the input port. Must be a valid Eigen scalar.
template <typename T>
class InputPortInterface {
 public:
  virtual ~InputPortInterface() {}

  /// Returns the current version of the port to which this input is connected.
  virtual std::ptrdiff_t get_version() = 0;

  /// Returns the rate at which this port is sampled, in seconds, or zero if
  /// this port is continuous.
  virtual double get_sample_time_sec() = 0;

  /// Returns the vector data on this port, or nullptr if this port is not
  /// vector-valued or not connected.
  virtual const VectorInterface<T>* get_vector_data() = 0;

 protected:
  InputPortInterface() {}
};

/// The ConnectedInputPort encapsulates the OutputPort of a System for use as
/// an input to another System.
///
/// @tparam T The type of the input port. Must be a valid Eigen scalar.
template <typename T>
class ConnectedInputPort : public InputPortInterface<T> {
 public:
  /// Creates an input port with the given @p sample_time_sec, connected
  /// to the given @p output_port, which must not be nullptr.
  ConnectedInputPort(const OutputPort<T>* output_port, double sample_time_sec)
      : output_port_(output_port), sample_time_sec_(sample_time_sec) {}

  ~ConnectedInputPort() override {}

  /// Returns the version of the connected output port.
  std::ptrdiff_t get_version() override { return output_port_->get_version(); }

  double get_sample_time_sec() override { return sample_time_sec_; }

  const VectorInterface<T>* get_vector_data() override {
    return output_port_->get_vector_data();
  }

 private:
  // ConnectedInputPort objects are neither copyable nor moveable.
  ConnectedInputPort(const ConnectedInputPort& other) = delete;
  ConnectedInputPort& operator=(const ConnectedInputPort& other) = delete;
  ConnectedInputPort(ConnectedInputPort&& other) = delete;
  ConnectedInputPort& operator=(ConnectedInputPort&& other) = delete;

  const OutputPort<T>* output_port_;
  double sample_time_sec_;
};

/// The FreestandingInputPort encapsulates a vector of data for use as an input
/// to a System.
///
/// @tparam T The type of the input port. Must be a valid Eigen scalar.
template <typename T>
class FreestandingInputPort : public InputPortInterface<T> {
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
        sample_time_sec_(sample_time_sec) {}

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  ptrdiff_t get_version() override { return output_port_.get_version(); }

  double get_sample_time_sec() override { return sample_time_sec_; }

  const VectorInterface<T>* get_vector_data() override {
    return output_port_.get_vector_data();
  }

  /// Returns a pointer to the data inside this InputPort, and updates the
  /// version so that Contexts depending on this InputPort know to invalidate
  /// their caches. Callers MUST NOT write on the returned pointer if there is
  /// any possibility this InputPort has been accessed since the last time
  /// GetMutableVectorData was called.
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

/// The Input is a container for all the Input ports to a particular System.
template <typename T>
struct SystemInput {
  std::vector<std::unique_ptr<InputPortInterface<T>>> ports;
};

}  // namespace systems
}  // namespace drake
