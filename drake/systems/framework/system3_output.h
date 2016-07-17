#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/value.h"
#include "drake/systems/framework/value_listener3.h"
#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

class AbstractSystem3;

/** An %OutputPort3 represents a data output from a System. Other Systems
may have InputPort objects that depend on the values presented at an
%OutputPort3.

Every %OutputPort3 produces a value at run time, represented as an AbstractValue
object. A subclass `VectorOutputPort3<T>` requires that the actual type of the
value be a numerical vector whose elements have scalar type `T`.
VectorOutputPort3 values can participate in automatic differentiation, but
more general %OutputPort3 objects cannot.

An %OutputPort3 has a particular data type and sampling information that
together can be used to determine whether it satisfies the requirements of a
particular InputPort. The type is represented here by a "model" value in the
form of an AbstractValue object that can be copied to initialize the Context's
cache entry that holds the value for this %OutputPort3. **/
class OutputPort3 {
 public:
  virtual ~OutputPort3() = default;

  /** Create an OutputPort3 that has no model value. **/
  OutputPort3() {}

  /** Create an %OutputPort3 with the indicated model value, taking over
  ownership of the model value. **/
  explicit OutputPort3(std::unique_ptr<AbstractValue> model_value)
      : OutputPort3() {
    set_model_value(std::move(model_value));
  }

  /** Set the model value for this %OutputPort3. Replaces the existing one if
  there is one. Takes over ownership of the given value. **/
  void set_model_value(std::unique_ptr<AbstractValue> model_value) {
    model_value_ = std::move(model_value);
  }

  /** Returns a const reference to a value of the type that this %OutputPort3
  will present at run time. **/
  const AbstractValue& get_model_value() const { return *model_value_; }

  /** Returns a mutable pointer to a value of the type that this %OutputPort3
  will present at run time. **/
  AbstractValue* get_mutable_model_value() { return model_value_.get(); }

  /** If this %OutputPort3 is owned by a system, return that system and the
  output port number by which that system knows this port. If unowned, the
  returned pair is `(nullptr,-1)`. **/
  std::pair<const AbstractSystem3*, int> get_owner_system() const {
    return std::pair<const AbstractSystem3*, int>(system_, output_port_num_);
  }


 private:
  // OutputPort3 objects are neither copyable nor moveable.
  OutputPort3(const OutputPort3& other) = delete;
  OutputPort3& operator=(const OutputPort3& other) = delete;
  OutputPort3(OutputPort3&& other) = delete;
  OutputPort3& operator=(OutputPort3&& other) = delete;

  friend class AbstractSystem3;  // Allow call to set_owner().

  // Set backpointer; no ownership implied.
  void set_owner(AbstractSystem3* system, int output_port_num) {
    system_ = system;
    output_port_num_ = output_port_num;
  }

  // This AbstractSystem is the owner of this OutputPort3, where it has this
  // port
  // number.
  AbstractSystem3* system_{};
  int output_port_num_{-1};

  std::unique_ptr<AbstractValue> model_value_;

  // The rate at which this port produces output, in seconds.
  // If zero, the port is continuous.
  double sample_time_sec_{0.};
};

/** Extends OutputPort3 for cases where the OutputPort3 is known to be
vector-valued, with scalar elements of template type `T`. The model value is
a `VectorInterface<T>`.

The model value is actually stored in an AbstractValue so that it can be
retrieved as the model value of the base class OutputPort3. That allows all
output ports can be treated identically when appropriate. Note: the concrete
type is actually `Value<VectorObject<T>>`; the VectorObject wraps the
VectorInterface value.

@tparam T The type of the output port. Must be a valid Eigen scalar. **/
template <typename T>
class VectorOutputPort3 : public OutputPort3 {
 public:
  /** Takes over ownership of the supplied concrete VectorInterface object to
  serve as the model value for this VectorOutputPort3. **/
  explicit VectorOutputPort3(std::unique_ptr<VectorInterface<T>> model_value)
      : OutputPort3(std::make_unique<Value<VectorObject<T>>>(
            VectorObject<T>(std::move(model_value)))) {}

  /** If you supply just a length this port will use a BasicVector of that
  length as its model value. **/
  explicit VectorOutputPort3(int length)
      : VectorOutputPort3(std::make_unique<BasicVector<T>>(length)) {}

  /** Returns the model value of this port as a const reference to
  `VectorInterface<T>`. **/
  const VectorInterface<T>& get_model_vector() const {
    const auto& vector_object = get_model_value().GetValue<VectorObject<T>>();
    return vector_object.get_vector();
  }

  /** Returns the model value of this port as a mutable pointer to
  `VectorInterface<T>`. **/
  VectorInterface<T>* get_mutable_model_vector() {
    auto vector_object =
        get_mutable_model_value()->GetMutableValue<VectorObject<T>>();
    return vector_object->get_mutable_vector();
  }

 private:
  // VectorOutputPort3 objects are neither copyable nor moveable.
  VectorOutputPort3(const VectorOutputPort3& other) = delete;
  VectorOutputPort3& operator=(const VectorOutputPort3& other) = delete;
  VectorOutputPort3(VectorOutputPort3&& other) = delete;
  VectorOutputPort3& operator=(VectorOutputPort3&& other) = delete;
};

}  // namespace systems
}  // namespace drake
