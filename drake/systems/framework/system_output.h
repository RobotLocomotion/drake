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

template <typename T>
class OutputPort {
 public:
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
    version_++;
    for (OutputPortListenerInterface* dependent : dependents_) {
      dependent->Invalidate();
    }
    return vector_data_.get();
  }

 private:
  // OutputPort objects are neither copyable nor moveable.
  OutputPort(const OutputPort& other) = delete;
  OutputPort& operator=(const OutputPort& other) = delete;
  OutputPort(OutputPort&& other) = delete;
  OutputPort& operator=(OutputPort&& other) = delete;

  /// The port data, if the port is vector-valued.
  /// TODO(david-german-tri): Add abstract-valued ports.
  std::unique_ptr<VectorInterface<T>> vector_data_;

  /// The list of consumers that should be notified when the value on this
  /// output port changes.
  std::set<OutputPortListenerInterface*> dependents_;

  /// The rate at which this port produces output, in seconds.
  /// If zero, the port is continuous.
  double sample_time_sec_{};

  int64_t version_ = 0;
};

/// A container for all the output of a System.
/// @tparam T The mathematical type of the output.
template <typename T>
struct SystemOutput {
  std::vector<std::unique_ptr<OutputPort<T>>> ports;
};

}  // namespace systems
}  // namespace drake
