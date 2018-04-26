#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/system.h"  // Ugh
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

template <typename T>
class System;

template <typename T>
class Context;

using OutputPortIndex = TypeSafeIndex<class OutputPortTag>;

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
class OutputPort {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(OutputPort)

  virtual ~OutputPort() = default;

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
      throw std::logic_error("Allocate(): allocator returned a nullptr for " +
          GetPortIdString());
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
    DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));
    DRAKE_ASSERT_VOID(CheckValidOutputType(*value));

    DoCalc(context, value);
  }

  /** Returns a reference to the value of this output port contained in the
  given Context. If that value is not up to date with respect to its
  prerequisites, the Calc() method above is used first to update the value
  before the reference is returned. (Not implemented yet.) */
  // TODO(sherm1) Implement properly.
  const AbstractValue& Eval(const Context<T>& context) const {
    DRAKE_ASSERT_VOID(get_system().CheckValidContext(context));
    return DoEval(context);
  }

  /** Returns a reference to the System that owns this output port. Note that
  for a diagram output port this will be the diagram, not the leaf system whose
  output port was forwarded. */
  const System<T>& get_system() const {
    return system_;
  }

  /** Returns the index of this output port within the owning System. */
  OutputPortIndex get_index() const {
    return index_;
  }

  /** Gets the port data type specified at port construction. */
  PortDataType get_data_type() const { return data_type_; }

  /** Returns the fixed size expected for a vector-valued output port. Not
  meaningful for abstract output ports. */
  int size() const { return size_; }

 protected:
  /** Provides derived classes the ability to set the base class members at
  construction.
  @param system
    The System that will own this new output port. This port will
    be assigned the next available output port index in this system.
  @param data_type
    Whether the port described is vector or abstract valued.
  @param size
    If the port described is vector-valued, the number of elements expected,
    otherwise ignored. */
  OutputPort(const System<T>& system, PortDataType data_type, int size)
      : system_(system),
        index_(system.get_num_output_ports()),
        data_type_(data_type),
        size_(size) {
    if (size_ == kAutoSize) {
      DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
    }
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

  /** This is useful for error messages and produces `"output port <#> of
  GetSystemIdString()"` with whatever System identification string is produced
  by that method. */
  std::string GetPortIdString() const {
    std::ostringstream oss;
    oss << "output port " << this->get_index() << " of "
        << this->get_system().GetSystemIdString();
    return oss.str();
  }

 private:
  // Check whether the allocator returned a value that is consistent with
  // this port's specification.
  // If this is a vector-valued port, we can check that the returned abstract
  // value actually holds a BasicVector-derived object, and for fixed-size ports
  // that the object has the right size.
  void CheckValidAllocation(const AbstractValue& proposed) const {
    if (this->get_data_type() != kVectorValued)
      return;  // Nothing we can check for an abstract port.

    auto proposed_vec = dynamic_cast<const Value<BasicVector<T>>*>(&proposed);
    if (proposed_vec == nullptr) {
      std::ostringstream oss;
      oss << "Allocate(): expected BasicVector output type but got "
          << NiceTypeName::Get(proposed) << " for " << GetPortIdString();
      throw std::logic_error(oss.str());
    }

    if (this->size() == kAutoSize)
      return;  // Any size is acceptable.

    const int proposed_size = proposed_vec->get_value().size();
    if (proposed_size != this->size()) {
      std::ostringstream oss;
      oss << "Allocate(): expected vector output type of size " << this->size()
          << " but got a vector of size " << proposed_size
          << " for " << GetPortIdString();
      throw std::logic_error(oss.str());
    }
  }

  // Check that an AbstractValue provided to Calc() is suitable for this port.
  // (Very expensive; use in Debug only.)
  // See CacheEntry::CheckValidAbstractValue; treat both methods similarly.
  void CheckValidOutputType(const AbstractValue&) const {
    // TODO(sherm1) Consider whether we can depend on there already being an
    //              object of this type in the output port's CacheEntryValue so we
    //              wouldn't have to allocate one here. If so could also store
    //              a precomputed type_index there for further savings. Would
    //              need to pass in a Context.
    auto good = DoAllocate();  // Expensive!
    // Attempt to interpret these as BasicVectors.
    auto proposed_vec = dynamic_cast<const Value<BasicVector<T>>*>(&proposed);
    auto good_vec = dynamic_cast<const Value<BasicVector<T>>*>(good.get());
    if (proposed_vec && good_vec) {
      CheckValidBasicVector(good_vec->get_value(),
                            proposed_vec->get_value());
    } else {
      // At least one is not a BasicVector.
      CheckValidAbstractValue(*good, proposed);
    }
  }

  // Check that both type-erased arguments have the same underlying type.
  void CheckValidAbstractValue(const AbstractValue& good,
                               const AbstractValue& proposed) const {
    if (typeid(proposed) != typeid(good)) {
      std::ostringstream oss;
      oss << "Calc(): expected AbstractValue output type "
          << NiceTypeName::Get(good) << " but got " << NiceTypeName::Get(proposed)
          << " for " << GetPortIdString();
      throw std::logic_error(oss.str());
    }
  }

  // Check that both BasicVector arguments have the same underlying type.
  void CheckValidBasicVector(const BasicVector<T>& good,
                             const BasicVector<T>& proposed) const {
    if (typeid(proposed) != typeid(good)) {
      std::ostringstream oss;
      oss << "Calc(): expected BasicVector output type "
          << NiceTypeName::Get(good) << " but got " << NiceTypeName::Get(proposed)
          << " for " << GetPortIdString();
      throw std::logic_error(oss.str());
    }
  }

  const System<T>& system_;
  const OutputPortIndex index_;
  const PortDataType data_type_;
  const int size_;
};

}  // namespace systems
}  // namespace drake
