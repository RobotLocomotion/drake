#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/common/autodiff.h"
#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/output_port_base.h"
#include "drake/systems/framework/system_base.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

// Break the System <=> OutputPort physical dependency cycle.  OutputPorts are
// decorated with a back-pointer to their owning System<T>, but that pointer is
// forward-declared here and never dereferenced within this file.
template <typename T>
class System;

/** An %OutputPort belongs to a System and represents the properties of one of
that System's output ports. %OutputPort objects are assigned OutputPortIndex
values in the order they are declared; these are unique within a single System.

An output port can be considered a "window" into a System that permits
controlled exposure of one of the values contained in that System's Context at
run time. Input ports of other subsystems may be connected to an output port to
construct system diagrams with carefully managed interdependencies.

The exposed value may be the result of an output computation, or it may
simply expose some other value contained in the Context, such as the values
of state variables. The Context handles caching of output port values and tracks
dependencies to ensure that the values are valid with respect to their
prerequisites. Leaf systems provide for the production of output port values, by
computation or forwarding from other values within the associated leaf context.
A diagram's output ports, on the other hand, are exported from output ports of
its contained subsystems.

An output port's value is always stored as an AbstractValue, but we also
provide special handling for output ports known to have numeric (vector) values.
Vector-valued ports may specify a particular vector length, or may leave that
to be determined at runtime.

%OutputPort objects support three important operations:
- Allocate() returns an object that can hold the port's value.
- Calc() unconditionally computes the port's value.
- Eval() updates a cached value if necessary.

@tparam T The vector element type, which must be a valid Eigen scalar.

Instantiated templates for the following kinds of T's are provided:
- double
- AutoDiffXd
- symbolic::Expression

They are already available to link against in the containing library.
No other values for T are currently supported. */
// TODO(sherm1) Implement caching for output ports and update the above
// documentation to explain in more detail.
template <typename T>
class OutputPort : public OutputPortBase {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputPort)

  ~OutputPort() override = default;

  /** Returns a reference to the up-to-date value of this output port contained
  in the given Context. This is the preferred way to obtain an output port's
  value since it will not be recalculated once up to date. If the value is not
  already up to date with respect to its prerequisites, this port's Calc()
  method is used first to update the value before the reference is returned. The
  Calc() method may be arbitrarily expensive, but Eval() is constant time and
  _very_ fast if the value is already up to date. */
  template <typename ValueType>
  const ValueType& Eval(const Context<T>& context) const {
    const AbstractValue& abstract_value = EvalAbstract(context);
    return ExtractValueOrThrow<ValueType>(__func__, abstract_value);
  }

  /** Allocates a concrete object suitable for holding the value to be exposed
  by this output port, and returns that as an AbstractValue. The returned object
  will never be null. If Drake assertions are enabled (typically only in Debug
  builds), validates for a vector-valued port that the returned AbstractValue
  is actually a BasicVector-derived type and that it has an acceptable size.
  @note If this is a vector-valued port, the underlying type is
  `Value<BasicVector<T>>`; downcast to `BasicVector<T>` before downcasting to
  the specific `BasicVector` subclass. */
  std::unique_ptr<AbstractValue> Allocate() const {
    std::unique_ptr<AbstractValue> value = DoAllocate();
    if (value == nullptr) {
      throw std::logic_error(fmt::format(
          "OutputPort::Allocate(): allocator returned a nullptr for {}.",
          GetPortIdString()));
    }
    DRAKE_ASSERT_VOID(CheckValidAllocation(*value));
    return value;
  }

  /** Unconditionally computes the value of this output port with respect to the
  given context, into an already-allocated AbstractValue object whose concrete
  type must be exactly the same as the type returned by this port's allocator.
  If Drake assertions are enabled (typically only in Debug builds), validates
  that the given `value` has exactly the same concrete type as is returned by
  the Allocate() method. */
  void Calc(const Context<T>& context, AbstractValue* value) const {
    DRAKE_DEMAND(value != nullptr);
    DRAKE_ASSERT_VOID(
        get_system_base().ThrowIfContextNotCompatible(context));
    DRAKE_ASSERT_VOID(CheckValidOutputType(*value));

    DoCalc(context, value);
  }

  /** Returns a reference to the value of this output port contained in the
  given Context. If that value is not up to date with respect to its
  prerequisites, the Calc() method above is used first to update the value
  before the reference is returned. */
  const AbstractValue& EvalAbstract(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(
        get_system_base().ThrowIfContextNotCompatible(context));
    return DoEval(context);
  }

  /** Returns a reference to the System that owns this output port. Note that
  for a diagram output port this will be the diagram, not the leaf system whose
  output port was forwarded. */
  const System<T>& get_system() const {
    return system_;
  }

 protected:
  /** Provides derived classes the ability to set the base class members at
  construction. See OutputPortBase::OutputPortBase() for the meaning of these
  parameters.
  @pre The `system` parameter must be the same object as the `system_base`
  parameter. */
  // The System and SystemBase are provided separately since we don't have
  // access to System's declaration here so can't cast but the caller can.
  OutputPort(const System<T>* system,
             SystemBase* system_base,
             OutputPortIndex index,
             DependencyTicket ticket, PortDataType data_type, int size)
      : OutputPortBase(system_base, index, ticket, data_type, size),
        system_{*system} {
    DRAKE_DEMAND(static_cast<const void*>(system) == system_base);
  }

  /** A concrete %OutputPort must provide a way to allocate a suitable object
  for holding the runtime value of this output port. The particulars may depend
  on values and types of objects in the given Context.
  @returns A unique_ptr to the new value-holding object as an AbstractValue. */
  virtual std::unique_ptr<AbstractValue> DoAllocate() const = 0;

  /** A concrete %OutputPort must implement this method to calculate the value
  this output port should have, given the supplied Context. The value may be
  determined by computation or by copying from a source value in the Context.
  @param context A Context that has already been validated as compatible with
                 the System whose output port this is.
  @param value   A pointer that has already be validated as non-null and
                 pointing to an object of the right type to hold a value of
                 this output port. */
  virtual void DoCalc(const Context<T>& context,
                      AbstractValue* value) const = 0;

  /** A concrete %OutputPort must provide access to the current value of this
  output port stored within the given Context. If the value is already up to
  date with respect to its prerequisites in `context`, no computation should be
  performed. Otherwise, the implementation should arrange for the value to be
  computed, typically but not necessarily by invoking DoCalc().
  @param context A Context that has already been validated as compatible with
                 the System whose output port this is. */
  virtual const AbstractValue& DoEval(const Context<T>& context) const = 0;

  /** This is useful for error messages and produces a human-readable
  identification of an offending output port. */
  std::string GetPortIdString() const {
    return fmt::format("OutputPort[{}] of System {} ({})",
                       this->get_index(),
                       this->get_system_base().GetSystemPathname(),
                       NiceTypeName::RemoveNamespaces(
                           this->get_system_base().GetSystemType()));
  }

 private:
  // If this is a vector-valued port, we can check that the returned abstract
  // value actually holds a BasicVector-derived object, and for fixed-size ports
  // that the object has the right size.
  void CheckValidAllocation(const AbstractValue&) const;

  // Check that an AbstractValue provided to Calc() is suitable for this port.
  // (Very expensive; use in Debug only.)
  void CheckValidOutputType(const AbstractValue&) const;

  // User said output port would have a particular concrete type but it doesn't.
  template <typename ValueType>
  [[noreturn]] void ThrowBadValueType(const char* func,
                                      const AbstractValue& abstract) const;

  // Pull a value of a given type from an abstract value or issue a nice
  // message if the type is not correct.
  template <typename ValueType>
  const ValueType& ExtractValueOrThrow(const char* func,
                                       const AbstractValue& abstract) const;

  const System<T>& system_;
};

template <typename T>
void OutputPort<T>::CheckValidAllocation(const AbstractValue& proposed) const {
  if (this->get_data_type() != kVectorValued)
    return;  // Nothing we can check for an abstract port.

  auto proposed_vec = dynamic_cast<const Value<BasicVector<T>>*>(&proposed);
  if (proposed_vec == nullptr) {
    throw std::logic_error(
        fmt::format("OutputPort::Allocate(): expected BasicVector output type "
                    "but got {} for {}.",
                    proposed.GetNiceTypeName(), GetPortIdString()));
  }

  if (this->size() == kAutoSize) return;  // Any size is acceptable.

  const int proposed_size = proposed_vec->get_value().size();
  if (proposed_size != this->size()) {
    throw std::logic_error(
        fmt::format("OutputPort::Allocate(): expected vector output type of "
                    "size {} but got a vector of size {} for {}.",
                    this->size(), proposed_size, GetPortIdString()));
  }
}

// See CacheEntry::CheckValidAbstractValue; treat both methods similarly.
template <typename T>
void OutputPort<T>::CheckValidOutputType(const AbstractValue& proposed) const {
  // TODO(sherm1) Consider whether we can depend on there already being an
  // object of this type in the output port's CacheEntryValue so we wouldn't
  // have to allocate one here. If so could also store a precomputed
  // type_index there for further savings. Would need to pass in a Context.
  auto good = DoAllocate();  // Expensive!
  if (proposed.type_info() != good->type_info()) {
    throw std::logic_error(
        fmt::format("OutputPort::Calc(): expected output type {} "
                    "but got {} for {}.",
                    good->GetNiceTypeName(), proposed.GetNiceTypeName(),
                    GetPortIdString()));
  }
}

template <typename T>
template <typename ValueType>
void OutputPort<T>::ThrowBadValueType(const char* func_name,
                                      const AbstractValue& abstract) const {
  throw std::logic_error(
      fmt::format("OutputPort::{}(): wrong value type {} "
                  "specified but actual type was {} for {}.",
                  func_name, NiceTypeName::Get<ValueType>(),
                  abstract.GetNiceTypeName(), GetPortIdString()));
}

template <typename T>
template <typename ValueType>
const ValueType& OutputPort<T>::ExtractValueOrThrow(
    const char* func, const AbstractValue& abstract) const {
  const ValueType* value = abstract.MaybeGetValue<ValueType>();
  if (!value)
    ThrowBadValueType<ValueType>(func, abstract);
  return *value;
}

}  // namespace systems
}  // namespace drake
