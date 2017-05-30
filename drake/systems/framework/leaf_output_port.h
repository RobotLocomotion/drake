#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/system_common.h"
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
  using AllocCallback = std::function<std::unique_ptr<AbstractValue>(
      const Context<T>&)>;

  /** Signature of a function suitable for allocating a BasicVector of the
  right size and concrete type for a particular vector-valued output port. */
  using AllocVectorCallback = std::function<std::unique_ptr<BasicVector<T>>(
      const Context<T>&)>;

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

  /** Constructs an abstract-valued output port. The supplied allocator returns
  a suitable AbstractValue in which to hold the result. The supplied calculator
  function writes to an AbstractValue of the same underlying concrete type as is
  returned by the allocator. */
  LeafOutputPort(const System<T>& system, AllocCallback alloc_function,
                 CalcCallback calc_function)
      : OutputPort<T>(system, kAbstractValued, 0 /* size */) {
    set_allocation_function(alloc_function);
    set_calculation_function(calc_function);
  }

  /** Constructs a fixed-size vector-valued output port. The supplied allocator
  returns a BasicVector of the correct size in which to hold the result. The
  supplied calculator function writes to a BasicVector of the same underlying
  concrete type as is returned by the allocator. If a particular size is
  required, supply it here; otherwise use kAutoSize. (Note: currently a fixed
  size is required.) */
  LeafOutputPort(const System<T>& system,
                 AllocVectorCallback vector_alloc_function, int size,
                 CalcVectorCallback vector_calc_function)
      : OutputPort<T>(system, kVectorValued, size) {
    set_allocation_function(vector_alloc_function);
    set_calculation_function(vector_calc_function);
  }

  /** Sets or replaces the allocation function for this output port, using
  a function that returns an AbstractValue. */
  void set_allocation_function(AllocCallback alloc_function) {
    alloc_function_ = alloc_function;
  }

  /** Sets or replaces the allocation function for this vector-valued output
  port, using a function that returns an object derived from
  `BasicVector<T>`. */
  void set_allocation_function(AllocVectorCallback vector_alloc_function) {
    if (!vector_alloc_function) {
      alloc_function_ = nullptr;
    } else {
      // Wrap the vector-returning function with an AbstractValue-returning
      // allocator.
      alloc_function_ = [vector_alloc_function](const Context<T>& context) {
        std::unique_ptr<BasicVector<T>> vector = vector_alloc_function(context);
        return std::unique_ptr<AbstractValue>(
            new Value<BasicVector<T>>(std::move(vector)));
      };
    }
  }

  /** Sets or replaces the calculation function for this output port, using
  a function that writes into an `AbstractValue`. */
  void set_calculation_function(CalcCallback calc_function) {
    calc_function_ = calc_function;
  }

  /** Sets or replaces the calculation function for this vector-valued output
  port, using a function that writes into a `BasicVector<T>`. */
  void set_calculation_function(CalcVectorCallback vector_calc_function);

  /** (Advanced) Sets or replaces the evaluation function for this output
  port, using a function that returns an `AbstractValue`. By default, an
  evaluation function is automatically provided that calls the calculation
  function. */
  void set_evaluation_function(EvalCallback eval_function) {
    eval_function_ = eval_function;
  }

 private:
  // Invokes the supplied allocation function if there is one, otherwise clones
  // the model value if there is one, otherwise complains.
  std::unique_ptr<AbstractValue> DoAllocate(
      const Context<T>& context) const final;

  // Invokes the supplied calculation function if present.
  void DoCalc(const Context<T>& context, AbstractValue* value) const final;

  // If this output port has its own cache entry then this checks the validity
  // bit, invokes DoCalc() into the cache entry if necessary, and then returns
  // a reference to the cache entry's value. Otherwise it invokes a custom
  // evaluation function which must exist.
  const AbstractValue& DoEval(const Context<T>& context) const final;

  AllocCallback alloc_function_;
  CalcCallback  calc_function_;
  EvalCallback  eval_function_;
};

// See diagram.h for DiagramOutputPort.

}  // namespace systems
}  // namespace drake
