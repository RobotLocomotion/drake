#pragma once

#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/output_port_listener_interface.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// The OutputPort represents a data output from a System. Other Systems
/// may depend on the OutputPort. When an OutputPort is deleted, it will
/// automatically notify the listeners that depend on it to disconnect,
/// meaning those ports will resolve to nullptr.
class DRAKE_EXPORT OutputPort {
 public:
  /// Constructs a vector-valued OutputPort.
  /// Takes ownership of @p vec.
  ///
  /// @tparam T The type of the vector data. Must be a valid Eigen scalar.
  /// @tparam V The type of @p vec itself. Must implement BasicVector<T>.
  template <template <typename T> class V, typename T>
  explicit OutputPort(std::unique_ptr<V<T>> vec)
      : data_(new VectorValue<T>(std::move(vec))) {}

  /// Constructs an abstract-valued OutputPort.
  /// Takes ownership of @p data.
  explicit OutputPort(std::unique_ptr<AbstractValue> data)
      : data_(std::move(data)) {}

  /// Constructs an abstract-valued OutputPort.
  /// Takes ownership of @p data.
  ///
  /// @tparam T The type of the data.
  template <typename T>
  explicit OutputPort(std::unique_ptr<Value<T>> data)
      : OutputPort(std::unique_ptr<AbstractValue>(data.release())) {}

  virtual ~OutputPort();

  /// Returns the abstract value in this port.
  const AbstractValue* get_abstract_data() const { return data_.get(); }

  /// Returns the vector of data in this output port. Throws std::bad_cast
  /// if this is not a vector-valued port.
  template <typename T>
  const BasicVector<T>* get_vector_data() const {
    return data_->GetValue<BasicVector<T>*>();
  }

  /// Returns a positive and monotonically increasing number that is guaranteed
  /// to change whenever GetMutableVectorData is called.
  int64_t get_version() const { return version_; }

  /// Registers @p dependent to receive invalidation notifications whenever this
  /// port's version number is incremented.
  void add_dependent(detail::OutputPortListenerInterface* dependent) {
    dependents_.insert(dependent);
  }

  /// Unregisters @p dependent from invalidation notifications.
  void remove_dependent(detail::OutputPortListenerInterface* dependent) {
    dependents_.erase(dependent);
  }

  /// Returns a pointer to the data inside this OutputPort, and updates the
  /// version so that Contexts depending on this OutputPort know to invalidate
  /// their caches. Callers MUST NOT write on the returned pointer if there is
  /// any possibility this OutputPort has been accessed since the last time
  /// GetMutableVectorData was called.
  AbstractValue* GetMutableData() {
    InvalidateAndIncrement();
    return data_.get();
  }

  /// Returns a pointer to the data inside this OutputPort, and updates the
  /// version so that Contexts depending on this OutputPort know to invalidate
  /// their caches. Callers MUST NOT write on the returned pointer if there is
  /// any possibility this OutputPort has been accessed since the last time
  /// GetMutableVectorData was called.
  ///
  /// Throws std::bad_cast if this is not a vector-valued port.
  template <typename T>
  BasicVector<T>* GetMutableVectorData() {
    InvalidateAndIncrement();
    return data_->GetValue<BasicVector<T>*>();
  }

  /// Returns a clone of this OutputPort containing a clone of the data, but
  /// without any dependents.
  std::unique_ptr<OutputPort> Clone() const;

 private:
  void InvalidateAndIncrement();

  // OutputPort objects are neither copyable nor moveable.
  OutputPort(const OutputPort& other) = delete;
  OutputPort& operator=(const OutputPort& other) = delete;
  OutputPort(OutputPort&& other) = delete;
  OutputPort& operator=(OutputPort&& other) = delete;

  // The port data.
  std::unique_ptr<AbstractValue> data_;

  // The list of consumers that should be notified when the value on this
  // output port changes.
  std::set<detail::OutputPortListenerInterface*> dependents_;

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
  virtual OutputPort* get_mutable_port(int index) = 0;
  virtual const OutputPort& get_port(int index) const = 0;

  /// Returns a type-preserving clone of this SystemOutput using the NVI idiom.
  std::unique_ptr<SystemOutput<T>> Clone() const {
    return std::unique_ptr<SystemOutput<T>>(DoClone());
  }

  /// Returns the abstract value in the port at @p index.
  const AbstractValue* get_data(int index) const {
    DRAKE_ASSERT(index >= 0 && index < get_num_ports());
    return get_port(index).get_abstract_data();
  }

  /// Returns the vector value in the port at @p index. Throws std::bad_cast if
  /// the port is not vector-valued.
  const BasicVector<T>* get_vector_data(int index) const {
    DRAKE_ASSERT(index >= 0 && index < get_num_ports());
    return get_port(index).template get_vector_data<T>();
  }

  /// Returns a pointer to the data inside the port at @p index, and updates the
  /// version so that Contexts depending on that OutputPort know to invalidate
  /// their caches. Callers MUST NOT write on the returned pointer if there is
  /// any possibility this OutputPort has been accessed since the last time
  /// GetMutableVectorData was called.
  AbstractValue* GetMutableData(int index) {
    DRAKE_ASSERT(index >= 0 && index < get_num_ports());
    return get_mutable_port(index)->GetMutableData();
  }

  /// Returns a pointer to the data inside the port at @p index, and updates the
  /// version so that Contexts depending on that OutputPort know to invalidate
  /// their caches. Callers MUST NOT write on the returned pointer if there is
  /// any possibility this OutputPort has been accessed since the last time
  /// GetMutableVectorData was called.
  ///
  /// Throws std::bad_cast if this is not a vector-valued port.
  BasicVector<T>* GetMutableVectorData(int index) {
    DRAKE_ASSERT(index >= 0 && index < get_num_ports());
    return get_mutable_port(index)->template GetMutableVectorData<T>();
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

  OutputPort* get_mutable_port(int index) override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return ports_[index].get();
  }

  const OutputPort& get_port(int index) const override {
    DRAKE_DEMAND(index >= 0 && index < get_num_ports());
    return *ports_[index];
  }

  std::vector<std::unique_ptr<OutputPort>>* get_mutable_ports() {
    return &ports_;
  }

  void add_port(std::unique_ptr<AbstractValue> value) {
    ports_.emplace_back(new OutputPort(std::move(value)));
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
  std::vector<std::unique_ptr<OutputPort>> ports_;
};

}  // namespace systems
}  // namespace drake
