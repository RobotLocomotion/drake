#pragma once

#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// OutputPortListenerInterface is an interface that consumers of an output
/// port must satisfy to receive notifications when the value on that output
/// port's version number is incremented.
///
/// This interface is a Drake-internal detail. Users should not implement it.
/// TODO(david-german-tri): Consider moving to its own file.
class DRAKESYSTEMFRAMEWORK_EXPORT OutputPortListenerInterface {
 public:
  virtual ~OutputPortListenerInterface() {}

  /// Invalidates any data that depends on the OutputPort. Called whenever
  /// the OutputPort's version number is incremented.
  virtual void Invalidate() = 0;
};

/// The OutputPort represents a data output from a System. Other Systems
/// may depend on the OutputPort.
///
/// @tparam T The type of the output port. Must be a valid Eigen scalar.
template <typename T>
class OutputPort {
 public:
  /// Takes ownership of @p data.
  explicit OutputPort(std::unique_ptr<VectorInterface<T>> data)
      : vector_data_(std::move(data)) {}

  /// Returns the vector of data in this output port, or nullptr if this is
  /// an abstract-valued port.
  const VectorInterface<T>* get_vector_data() const {
    return vector_data_.get();
  }

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  int64_t get_version() const { return version_; }

  /// Registers @p dependent to receive invalidation notifications whenever this
  /// port's version number is incremented.
  void add_dependent(OutputPortListenerInterface* dependent) {
    dependents_.insert(dependent);
  }

  /// Unregisters @p dependent from invalidation notifications.
  void remove_dependent(OutputPortListenerInterface* dependent) {
    dependents_.erase(dependent);
  }

  /// Returns a pointer to the data inside this OutputPort, and updates the
  /// version so that Contexts depending on this OutputPort know to invalidate
  /// their caches. Callers MUST NOT write on the returned pointer if there is
  /// any possibility this OutputPort has been accessed since the last time
  /// GetMutableVectorData was called.
  VectorInterface<T>* GetMutableVectorData() {
    ++version_;
    for (OutputPortListenerInterface* dependent : dependents_) {
      dependent->Invalidate();
    }
    return vector_data_.get();
  }

  /// Returns a clone of this OutputPort containing a clone of the data, but
  /// without any dependents.
  std::unique_ptr<OutputPort<T>> Clone() const {
    return std::make_unique<OutputPort<T>>(vector_data_->CloneVector());
  }

 private:
  // OutputPort objects are neither copyable nor moveable.
  OutputPort(const OutputPort& other) = delete;
  OutputPort& operator=(const OutputPort& other) = delete;
  OutputPort(OutputPort&& other) = delete;
  OutputPort& operator=(OutputPort&& other) = delete;

  // The port data, if the port is vector-valued.
  // TODO(sherm1): Add abstract-valued ports.
  std::unique_ptr<VectorInterface<T>> vector_data_;

  // The list of consumers that should be notified when the value on this
  // output port changes.
  std::set<OutputPortListenerInterface*> dependents_;

  int64_t version_ = 0;
};

/// An abstract base class template for the output ports of a System.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class SystemOutput {
 public:
  virtual ~SystemOutput() {}

  virtual int get_num_ports() const = 0;
  virtual OutputPort<T>* get_mutable_port(int index) = 0;
  virtual const OutputPort<T>& get_port(int index) const = 0;

  /// Returns a type-preserving clone of this SystemOutput using the NVI idiom.
  std::unique_ptr<SystemOutput<T>> Clone() const {
    return std::unique_ptr<SystemOutput<T>>(DoClone());
  }

 protected:
  /// The NVI implementation of Clone().
  virtual SystemOutput<T>* DoClone() const = 0;
};

/// A container for all the output of a leaf System.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
struct LeafSystemOutput : public SystemOutput<T> {
  LeafSystemOutput() {}
  ~LeafSystemOutput() override {}

  int get_num_ports() const override { return static_cast<int>(ports_.size()); }

  OutputPort<T>* get_mutable_port(int index) override {
    return ports_[index].get();
  }

  const OutputPort<T>& get_port(int index) const override {
    return *ports_[index];
  }

  std::vector<std::unique_ptr<OutputPort<T>>>* get_mutable_ports() {
    return &ports_;
  }

 protected:
  /// Returns a clone that includes a deep copy of all the output ports.
  LeafSystemOutput<T>* DoClone() const override {
    LeafSystemOutput<T>* clone = new LeafSystemOutput<T>();
    for (const auto& port : ports_) {
      clone->ports_.push_back(port->Clone());
    }
    return clone;
  }

 private:
  std::vector<std::unique_ptr<OutputPort<T>>> ports_;
};

}  // namespace systems
}  // namespace drake
