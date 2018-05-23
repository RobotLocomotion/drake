#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

template <typename T>
class System;

template <typename T>
class Context;

/** Adds methods for allocation, calculation, and caching of an output
port's value. When created, an allocation and a calculation function must be
provided. The output type of the calculation callback must match the type
returned by the allocation function.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following kinds of T's are provided:
- double
- AutoDiffXd
- symbolic::Expression

They are already available to link against in the containing library.
No other values for T are currently supported. */
// TODO(sherm1) Implement caching for output ports.
template <typename T>
class LeafOutputPort : public OutputPort<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(LeafOutputPort)

  ~LeafOutputPort() override = default;

  /** Signature of a function suitable for allocating an object that can hold
  a value of a particular output port. The result is returned as an
  AbstractValue even if this is a vector-valued port. */
  using AllocCallback = std::function<std::unique_ptr<AbstractValue>()>;

  /** Signature of a function suitable for calculating a value of a particular
  output port, given a place to put the value. */
  using CalcCallback =
  std::function<void(const Context<T>&, AbstractValue*)>;

  /** Signature of a function suitable for calculating a value of a particular
  vector-valued output port, given a place to put the value. */
  using CalcVectorCallback =
  std::function<void(const Context<T>&, BasicVector<T>*)>;

  /** Signature of a function suitable for obtaining the cached value of a
  particular output port. */
  using EvalCallback = std::function<const AbstractValue&(const Context<T>&)>;

  // There is no EvalVectorCallback.

  /** Constructs an abstract-valued output port. The supplied allocator must
  return a suitable AbstractValue in which to hold the result. The supplied
  calculator function must write to an AbstractValue of the same underlying
  concrete type as is returned by the allocator. The allocator function is not
  invoked here during construction of the port so it may depend on data that
  becomes available only after completion of the containing System or
  Diagram. The `system` parameter must be the same object as the `system_base`
  parameter. */
  LeafOutputPort(const System<T>& system,
                 const internal::SystemMessageInterface& system_base,
                 OutputPortIndex index,
                 AllocCallback alloc_function,
                 CalcCallback calc_function)
      : OutputPort<T>(system, system_base, index, kAbstractValued,
                      0 /* size */) {
    set_allocation_function(alloc_function);
    set_calculation_function(calc_function);
  }

  /** Constructs a fixed-size vector-valued output port. The supplied allocator
  must return a `Value<BasicVector>` of the correct size in which to hold the
  result. The supplied calculator function must write to a BasicVector of the
  same underlying concrete type as is returned by the allocator. Requires the
  fixed size to be given explicitly here. The allocator function is not invoked
  here during construction of the port so it may depend on data that becomes
  available only after completion of the containing System or Diagram. The
  `system` parameter must be the same object as the `system_base` parameter. */
  // Note: there is no guarantee that the allocator can be invoked successfully
  // here since construction of the containing System is likely incomplete when
  // this method is invoked. Do not attempt to extract the size from
  // the allocator by calling it here.
  LeafOutputPort(const System<T>& system,
                 const internal::SystemMessageInterface& system_base,
                 OutputPortIndex index,
                 int fixed_size,
                 AllocCallback vector_alloc_function,
                 CalcVectorCallback vector_calc_function)
      : OutputPort<T>(system, system_base, index, kVectorValued, fixed_size) {
    set_allocation_function(vector_alloc_function);
    set_calculation_function(vector_calc_function);
  }

  /** (Advanced) Sets or replaces the evaluation function for this output
  port, using a function that returns an `AbstractValue`. By default, an
  evaluation function is automatically provided that calls the calculation
  function. */
  // TODO(sherm1) Not useful yet; implement with caching.
  void set_evaluation_function(EvalCallback eval_function) {
    eval_function_ = eval_function;
  }

 private:
  // Sets or replaces the allocation function for this output port, using
  // a function that returns an AbstractValue.
  void set_allocation_function(AllocCallback alloc_function) {
    alloc_function_ = alloc_function;
  }

  // Sets or replaces the calculation function for this output port, using
  // a function that writes into an `AbstractValue`.
  void set_calculation_function(CalcCallback calc_function) {
    calc_function_ = calc_function;
  }

  // Sets or replaces the calculation function for this vector-valued output
  // port, using a function that writes into a `BasicVector<T>`.
  void set_calculation_function(CalcVectorCallback vector_calc_function) {
    if (!vector_calc_function) {
      calc_function_ = nullptr;
    } else {
      // Wrap the vector-writing function with an AbstractValue-writing
      // function.
      calc_function_ = [this, vector_calc_function](const Context<T>& context,
                                                    AbstractValue* abstract) {
        // The abstract value must be a Value<BasicVector<T>>.
        auto value = dynamic_cast<Value<BasicVector<T>>*>(abstract);
        if (value == nullptr) {
          std::ostringstream oss;
          oss << "LeafOutputPort::Calc(): Expected a vector output type for "
              << this->GetPortIdString() << " but got a "
              << NiceTypeName::Get(*abstract) << " instead.";
          throw std::logic_error(oss.str());
        }
        vector_calc_function(context, &value->get_mutable_value());
      };
    }
  }

  // Invokes the supplied allocation function if there is one, otherwise
  // complains.
  std::unique_ptr<AbstractValue> DoAllocate() const final {
    std::unique_ptr<AbstractValue> result;

    if (alloc_function_) {
      result = alloc_function_();
    } else {
      throw std::logic_error(
          "LeafOutputPort::DoAllocate(): " + this->GetPortIdString() +
          " has no allocation function so cannot be allocated.");
    }
    if (result.get() == nullptr) {
      throw std::logic_error(
          "LeafOutputPort::DoAllocate(): allocator returned a nullptr for " +
          this->GetPortIdString());
    }
    return result;
  }

  // Invokes the supplied calculation function if present, otherwise complains.
  void DoCalc(const Context<T>& context, AbstractValue* value) const final {
    if (calc_function_) {
      calc_function_(context, value);
    } else {
      throw std::logic_error("LeafOutputPort::DoCalcWitnessValue(): " +
                             this->GetPortIdString() +
                             " had no calculation function available.");
    }
  }

  // Currently just invokes the supplied evaluation function if present,
  // otherwise complains.
  // TODO(sherm1) Generate this automatically using the cache & DoEvaluate().
  const AbstractValue& DoEval(const Context<T>& context) const final {
    if (eval_function_) return eval_function_(context);
    // TODO(sherm1) Provide proper default behavior for an output port with
    // its own cache entry.
    DRAKE_ABORT_MSG("LeafOutputPort::DoEval(): NOT IMPLEMENTED YET");
    static never_destroyed<Value<int>> dummy{0};
    return dummy.access();
  }

  AllocCallback alloc_function_;
  CalcCallback  calc_function_;
  EvalCallback  eval_function_;
};

// See diagram.h for DiagramOutputPort.

}  // namespace systems
}  // namespace drake
