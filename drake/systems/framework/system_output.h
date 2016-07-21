#pragma once

#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// OutputPortListenerInterface is an interface that consumers of an output
/// port must satisfy to receive notifications when the value on that output
/// port changes.
///
/// This interface is a Drake-internal detail. Users should not implement it.
// TODO(david-german-tri): Consider moving to its own file.
class DRAKESYSTEMFRAMEWORK_EXPORT OutputPortListenerInterface {
 public:
  virtual ~OutputPortListenerInterface() {}

  /// Invalidates any data that depends on the OutputPort. Called whenever
  /// the OutputPort's version number is incremented.
  virtual void Invalidate() = 0;
};

/// An %OutputPort represents a data output from a System. Other Systems
/// may have InputPort objects that depend on the values presented at an
/// %OutputPort.
///
/// There are two kinds of concrete OutputPort:
/// - VectorOutputPort: Vector values templatized by scalar type, and
/// - AbstractOutputPort: Abstract values of arbitrary type.
///
/// VectorOutputPort values can participate in automatic differentiation, but 
/// AbstractOutputPort values cannot.
class OutputPort {
 public:
  virtual ~OutputPort() = default;

  /// Registers @p dependent to receive invalidation notifications whenever this
  /// output port's value changes.
  void add_dependent(OutputPortListenerInterface* dependent) {
    dependents_.insert(dependent);
  }

  /// Unregisters @p dependent from invalidation notifications.
  void remove_dependent(OutputPortListenerInterface* dependent) {
    dependents_.erase(dependent);
  }

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  long long get_version() const { return version_; }

  /// Notify any dependents that the value on this %OutputPort has changed,
  /// and update the value version here.
  void NoteValueModification() {
    ++version_;
    for (OutputPortListenerInterface* dependent : dependents_)
      dependent->Invalidate();
  }

 protected:
  OutputPort() {}

 private:
  // OutputPort objects are neither copyable nor moveable.
  OutputPort(const OutputPort& other) = delete;
  OutputPort& operator=(const OutputPort& other) = delete;
  OutputPort(OutputPort&& other) = delete;
  OutputPort& operator=(OutputPort&& other) = delete;

  // The rate at which this port produces output, in seconds.
  // If zero, the port is continuous.
  double sample_time_sec_{0.};

  // The list of consumers that should be notified when the value on this
  // output port changes.
  std::set<OutputPortListenerInterface*> dependents_;

  // A counter that is bumped every time this port's value changes.
  long long version_{0};
};

/// The OutputPort represents a data output from a System. Other Systems
/// may depend on the OutputPort.
///
/// @tparam T The type of the output port. Must be a valid Eigen scalar.
template <typename T>
class VectorOutputPort : public OutputPort {
 public:
  /// Takes ownership of @p data.
  explicit VectorOutputPort(std::unique_ptr<VectorInterface<T>> data)
      : OutputPort(), vector_data_(std::move(data)) {}

  /// Returns the vector of data in this output port, or nullptr if this is
  /// an abstract-valued port.
  const VectorInterface<T>* get_vector_data() const {
    return vector_data_.get();
  }

  /// Returns a pointer providing mutable access to the data inside this
  /// %VectorOutputPort, and updates the version so that Contexts depending on
  /// this OutputPort know to invalidate their caches. Callers MUST NOT write on
  /// the returned pointer if there is any possibility this OutputPort has been
  /// accessed since the last time GetMutableVectorData() was called.
  VectorInterface<T>* GetMutableVectorData() {
    NoteValueModification();
    return vector_data_.get();
  }

 private:
  // VectorOutputPort objects are neither copyable nor moveable.
  VectorOutputPort(const VectorOutputPort& other) = delete;
  VectorOutputPort& operator=(const VectorOutputPort& other) = delete;
  VectorOutputPort(VectorOutputPort&& other) = delete;
  VectorOutputPort& operator=(VectorOutputPort&& other) = delete;

  // The port data for this vector-valued port.
  std::unique_ptr<VectorInterface<T>> vector_data_;
};

/// A container for all the output ports of a System. These are a mix of 
/// vector-valued and abstract-valued ports.
///
/// @tparam T The type of the output data. Must be a valid Eigen scalar.
template <typename T>
class SystemOutput {
 public:
  /// Construct a %SystemOutput with room for `num_ports` OutputPort objects,
  /// initially empty.
  explicit SystemOutput(int num_ports) : ports_(num_ports) {}

  /// Sets or replaces the indicated OutputPort with the given `port` and takes
  /// over ownership. The existing port, if any, is deleted.
  /// @throws std::out_of_range `port_num` is out of range.
  void set_port(int port_num, std::unique_ptr<OutputPort> port) {
    RangeCheck(port_num, "reset_port");
    ports_[port_num] = std::move(port);
  }

  /// Returns the number of OutputPort objects which may be contained here.
  /// Some or all of the ports may be empty.
  int get_num_ports() const { return (int)ports_.size(); }

  /// Returns `true` if the indicated port slot is unoccupied. The `port_num`
  /// must be in range.
  /// @throws std::out_of_range `port_num` is out of range.
  bool is_empty_port(int port_num) const {
    RangeCheck(port_num, "empty_port");
    return !ports_[port_num];
  }

  /// Check whether the indicated port is of type VectorOutputPort<T>; otherwise
  /// it is an AbstractOutputPort whose type must be agreed upon between the
  /// producer and consumer. This method returns `false` if the given `port_num`
  /// is empty.
  /// @throws std::out_of_range `port_num` is out of range.
  bool is_vector_port(int port_num) const {
    RangeCheck(port_num, "is_vector_port");
    const OutputPort* port = ports_[port_num].get();
    if (port == nullptr) return false;
    return dynamic_cast<const VectorOutputPort<T>*>(port) != nullptr;
  }

  /// Returns a const reference to the indicated port, which must be in range
  /// and not empty.
  const OutputPort& get_port(int port_num) const {
    RangeCheck(port_num, "get_port");
    return *ports_[port_num];
  }

  /// Returns a mutable pointer to the indicated port.
  OutputPort* get_mutable_port(int port_num) {
    RangeCheck(port_num, "get_port");
    return ports_[port_num].get();
  }

  /// Returns a const reference to the indicated VectorOutputPort.
  /// @throws std::logic_error The specified port is not vector valued.
  const VectorOutputPort<T>& get_vector_port(int port_num) const {
    const OutputPort& port = get_port(port_num);
    auto vector_port = dynamic_cast<const VectorOutputPort<T>*>(&port);
    if (vector_port == nullptr) {
      throw std::logic_error("SystemOutput::get_vector_port(): Port " +
                             std::to_string(port_num) +
                             " is not a VectorOutputPort.");
    }
    return *vector_port;
  }

  /// Returns a mutable pointer to the indicated VectorOutputPort.
  /// @throws std::logic_error The specified port is not vector valued.
  VectorOutputPort<T>* get_mutable_vector_port(int port_num) {
    const VectorOutputPort<T>& port = get_vector_port(port_num);
    return const_cast<VectorOutputPort<T>*>(&port);
  }

 private:
  void RangeCheck(int port_num, const char* name) const {
    if (0 <= port_num && port_num < get_num_ports()) return;
    throw std::out_of_range(GetMethod(name) + "output port " +
                            std::to_string(port_num) +
                            " is out of range (there are " +
                            std::to_string(get_num_ports()) + " ports).");
  }
  void RangeNullCheck(int port_num, const char* name) const {
    RangeCheck(port_num, name);
    if (!ports_[port_num])
      throw std::logic_error(GetMethod(name) + "output port " +
                            std::to_string(port_num) + " is empty.");
  }
  static std::string GetMethod(const char* name) {
    using namespace std::string_literals;
    return "SystemOutput::"s + std::string(name) + "(): "s;
  }

  std::vector<std::unique_ptr<OutputPort>> ports_;
};

}  // namespace systems
}  // namespace drake
