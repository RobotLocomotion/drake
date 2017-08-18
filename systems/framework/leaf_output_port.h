#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
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
  using AllocCallback = std::function<std::unique_ptr<AbstractValue>(
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

  /** Constructs an abstract-valued output port. The supplied allocator must
  return a suitable AbstractValue in which to hold the result. The supplied
  calculator function must write to an AbstractValue of the same underlying
  concrete type as is returned by the allocator. The allocator function is not
  invoked here during construction of the port so it may depend on data that
  becomes available only after completion of the containing System or
  Diagram. */
  LeafOutputPort(AllocCallback alloc_function, CalcCallback calc_function,
                 std::vector<DependencyTicket> calc_prerequisites,
                 System<T>* system);

  /** Constructs a fixed-size vector-valued output port. The supplied allocator
  must return a `Value<BasicVector>` of the correct size in which to hold the
  result. The supplied calculator function must write to a BasicVector of the
  same underlying concrete type as is returned by the allocator. Requires the
  fixed size to be given explicitly here. The allocator function is not invoked
  here during construction of the port so it may depend on data that becomes
  available only after completion of the containing System or Diagram. */
  // Note: there is no guarantee that the allocator can be invoked successfully
  // here since construction of the containing System is likely incomplete when
  // this method is invoked. Do not attempt to extract the size from
  // the allocator by calling it here.
  LeafOutputPort(int fixed_size, AllocCallback alloc_function,
                 CalcVectorCallback vector_calc_function,
                 std::vector<DependencyTicket> calc_prerequisites,
                 System<T>* system);

  /** (Advanced) Sets or replaces the evaluation function for this output
  port, using a function that returns an `AbstractValue`. By default, an
  evaluation function is automatically provided that calls the calculation
  function. */
  // TODO(sherm1) Not useful yet; implement with caching.
  void set_evaluation_function(EvalCallback eval_function) {
    eval_function_ = eval_function;
  }

  /** Returns the cache entry associated with this output port. */
  CacheIndex cache_index() const { return cache_index_; }

 private:
  // Creates an AbstractValue calculation function for this vector-valued output
  // port, using a function that writes into a `BasicVector<T>`.
  CalcCallback ConvertVectorCallback(CalcVectorCallback vector_calc_function);

  // Handles common constructor tasks.
  void CompleteConstruction(
  AllocCallback alloc_function, CalcCallback calc_function,
      std::vector<DependencyTicket> calc_prerequisites, System<T>* system);

  // Invokes the supplied allocation function if there is one, otherwise
  // complains.
  std::unique_ptr<AbstractValue> DoAllocate(
      const Context<T>& context) const final;

  // Invokes the supplied calculation function if present, otherwise complains.
  void DoCalc(const Context<T>& context, AbstractValue* value) const final;

  // Currently just invokes the supplied evaluation function if present,
  // otherwise forwards to this port's cache entry.
  const AbstractValue& DoEval(const Context<T>& context) const final;

  // Returns the cache entry's ticket and no subsystem.
  std::pair<optional<SubsystemIndex>, DependencyTicket> DoGetPrerequisite()
      const final;

  AllocCallback alloc_function_;
  CalcCallback  calc_function_;
  EvalCallback  eval_function_;

  CacheIndex cache_index_;
};

// See diagram.h for DiagramOutputPort.

}  // namespace systems
}  // namespace drake
