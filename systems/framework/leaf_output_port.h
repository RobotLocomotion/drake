#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/nice_type_name.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

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
template <typename T>
class LeafOutputPort final : public OutputPort<T> {
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
  Diagram. */
  LeafOutputPort(const System<T>* system, SystemBase* system_base,
                 OutputPortIndex index, DependencyTicket ticket,
                 AllocCallback alloc_function, CalcCallback calc_function,
                 std::vector<DependencyTicket> calc_prerequisites)
      : OutputPort<T>(system, system_base, index, ticket, kAbstractValued,
                      0 /* size */) {
    CompleteConstruction(std::move(alloc_function), std::move(calc_function),
                         std::move(calc_prerequisites));
  }

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
  LeafOutputPort(const System<T>* system, SystemBase* system_base,
                 OutputPortIndex index, DependencyTicket ticket, int fixed_size,
                 AllocCallback alloc_function,
                 CalcVectorCallback vector_calc_function,
                 std::vector<DependencyTicket> calc_prerequisites)
      : OutputPort<T>(system, system_base, index, ticket, kVectorValued,
                      fixed_size) {
    CompleteConstruction(std::move(alloc_function),
                         ConvertVectorCallback(std::move(vector_calc_function)),
                         std::move(calc_prerequisites));
  }

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
      std::vector<DependencyTicket> calc_prerequisites);

  // Invokes the supplied allocation function if there is one, otherwise
  // complains.
  std::unique_ptr<AbstractValue> DoAllocate() const final;

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

template <typename T>
auto LeafOutputPort<T>::ConvertVectorCallback(
    CalcVectorCallback vector_calc_function) -> CalcCallback {
  if (!vector_calc_function) return nullptr;

  // Wrap the vector-writing function with an AbstractValue-writing function.
  return [this, vector_calc_function](const Context<T>& context,
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

// Private method to be called from cached output port constructors, after the
// base class construction is complete. Records the functors and Calc()
// prerequisites, and allocates an appropriate cache entry.
template <typename T>
void LeafOutputPort<T>::CompleteConstruction(
    AllocCallback alloc_function, CalcCallback calc_function,
    std::vector<DependencyTicket> calc_prerequisites) {
  alloc_function_ = std::move(alloc_function);
  calc_function_ = std::move(calc_function);

  if (calc_prerequisites.empty())
    calc_prerequisites.push_back(this->get_system_base().all_sources_ticket());

  auto cache_alloc_function = [alloc = alloc_function_]() { return alloc(); };
  auto cache_calc_function = [calc = calc_function_](
      const ContextBase& context_base, AbstractValue* result) {
    const Context<T>& context = dynamic_cast<const Context<T>&>(context_base);
    return calc(context, result);
  };
  cache_index_ =
      this->get_mutable_system_base()
          .DeclareCacheEntry(
              "output port " + std::to_string(this->get_index()) + " cache",
              std::move(cache_alloc_function), std::move(cache_calc_function),
              std::move(calc_prerequisites))
          .cache_index();
}

template <typename T>
std::unique_ptr<AbstractValue> LeafOutputPort<T>::DoAllocate() const {
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

template <typename T>
void LeafOutputPort<T>::DoCalc(const Context<T>& context,
                               AbstractValue* value) const {
  if (calc_function_) {
    calc_function_(context, value);
  } else {
    throw std::logic_error("LeafOutputPort::DoCalcWitnessValue(): " +
        this->GetPortIdString() +
        " had no calculation function available.");
  }
}

template <typename T>
const AbstractValue& LeafOutputPort<T>::DoEval(
    const Context<T>& context) const {
  if (eval_function_)
    return eval_function_(context);
  const CacheEntry& entry =
      this->get_system_base().get_cache_entry(cache_index_);
  return entry.EvalAbstract(context);
}

template <typename T>
std::pair<optional<SubsystemIndex>, DependencyTicket>
LeafOutputPort<T>::DoGetPrerequisite() const {
  const CacheEntry& entry =
      this->get_system_base().get_cache_entry(cache_index_);
  return std::make_pair(nullopt, entry.ticket());
}

}  // namespace systems
}  // namespace drake
