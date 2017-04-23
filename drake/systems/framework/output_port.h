#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/type_safe_index.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_port_descriptor.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

template <typename T>
class System;

template <typename T>
class Context;

using OutputPortIndex = TypeSafeIndex<class OutputPortTag>;

//==============================================================================
//                                OUTPUT PORT
//==============================================================================
/** An %OutputPort belongs to a System and represents the properties of one of
that System's output ports. %OutputPort objects are assigned OutputPortIndex
values in the order they are declared; these are unique within a single System.

An output port can be considered a "window" into a System that permits
controlled exposure of one of the values contained in that System's Context at
run time. Input ports of other subsystems may be connected to an output port to
construct system diagrams with carefully managed interdependencies.

The exposed value may be the result of a an output computation, or it may
simply expose some other value contained in the Context, such as the values
of state variables. The Context handles caching of output port values and tracks
dependencies to ensure that the values are valid with respect to their
prerequisites. Leaf systems provide for the production of output port values, by
computation or forwarding from other values within the associated leaf context.
A diagram's output ports, on the other hand, are exported from output ports of
its contained subsystems.

An output port's value may always be treated as an AbstractValue, but we also
provide special handling for output ports known to have numeric (vector) values.
Vector-valued ports may specify a particular vector length, or may leave that
to be determined at runtime.

%OutputPort objects support three important operations:
- Allocate() returns an object that can hold the port's value.
- Calc() unconditionally computes the port's value.
- Eval() updates a cached value if necessary.

Variants of the above are available for vector-valued ports. See the method
documentation for more information.

@tparam T The mathematical type of the containing System, which must be a valid
          Eigen scalar.
**/
template <typename T>
class OutputPort {
 public:
  /// @name Basic Concepts
  /// MoveConstructible only; not CopyConstructible; not Copy/Move-Assignable.
  /// @{
  // See InputPortDescriptor doc for implementation note and justification.
  OutputPort() = delete;
  OutputPort(OutputPort&&) = default;
  OutputPort(const OutputPort&) = delete;
  OutputPort& operator=(OutputPort&&) = default;
  OutputPort& operator=(const OutputPort&) = delete;
  virtual ~OutputPort() = default;
  /// @}

  /** Allocate a concrete object suitable for holding the value to be exposed
  by this output port, and return that as an AbstractValue. This works for any
  output port, even if it is vector valued. You must supply a Context if the
  port's allocator may depend on it, otherwise it will be null. **/
  std::unique_ptr<AbstractValue> Allocate(
      const Context<T>* context = nullptr) const;

  /** If you know this output port is vector valued, this method will give you
  a numerical vector object of the right size to hold a value of this port.
  An exception will be thrown if the port is not vector valued. You must supply
  a Context if the port's allocator may depend on it, otherwise it will be
  null. **/
  std::unique_ptr<BasicVector<T>> AllocateVector(
      const Context<T>* context = nullptr) const;

  /** Unconditionally computes the value of this output port with respect to the
  given context, into an already-allocated AbstractValue object whose concrete
  type must be the correct type for this output port. Commonly the output
  argument will be an entry in the given Context's output cache, but that is not
  required. **/
  void Calc(const Context<T>& context, AbstractValue* value) const;

  /** If you know this output port is vector valued, this method is a variant
  of Calc() that writes the result into a BasicVector object which must be the
  correct type for this output port. An exception will be thrown if the port is
  not vector valued. **/
  void CalcVector(const Context<T>& context, BasicVector<T>* value) const {
    Value<BasicVector<T>*> abstract(value);
    Calc(context, &abstract);  // Writes on the output argument.
    // Destruction of the abstract value is harmless.
  }

  /** Returns a reference to the value of this output port contained in the
  given Context. If that value is not up to date with respect to its
  prerequisites, the Calc() method above is used first to update the value
  before the reference is returned. **/
  const AbstractValue& Eval(const Context<T>& context) const;

  /** If you know this output port is vector valued, this method is a variant
  of Eval() that returns a reference directly to the BasicVector contained in
  this port's AbstractValue in the given Context. An exception will be thrown if
  the port is not vector valued. **/
  const BasicVector<T>& EvalVector(const Context<T>& context) const {
    const AbstractValue& abstract = Eval(context);
    return *abstract.template GetValueOrThrow<BasicVector<T>*>();
  }

  /** Returns a pointer to the System that owns this output port, or `nullptr`
  if this output port object has not been added to a System. Note that
  for a diagram output port this will be the diagram, not the leaf system whose
  output port was forwarded. **/
  const System<T>* get_system() const {
    return system_;
  }

  /** Returns the index of this output port within the owning System. This is
  not valid if this output port object has not been added to a System. Use
  get_system() to check if you aren't sure. **/
  OutputPortIndex get_index() const {
    DRAKE_DEMAND(system_ != nullptr);
    return index_;
  }

  /** This is the port data type specified at port construction. **/
  PortDataType get_data_type() const { return data_type_; }

  /** For vector-valued output ports, if this is non-negative it is the
  required runtime size of the value. Otherwise is may be kAutoSize in which
  case the size is determined upon allocation. **/
  int size() const { return size_; }

  /** Internal use only. **/
  void set_system_and_index(const System<T>* system, OutputPortIndex index) {
    DRAKE_DEMAND(system_ == nullptr && system != nullptr);
    system_ = system;
    index_ = index;
  }

 protected:
  /** Constructor is for use by derived classes for setting the base class
  members.
  @param data_type Whether the port described is vector or abstract valued.
  @param size If the port described is vector-valued, the number of
              elements, or kAutoSize if determined by connections. **/
  OutputPort(PortDataType data_type, int size)
      : data_type_(data_type), size_(size) {
    if (size_ == kAutoSize) {
      DRAKE_ABORT_MSG("Auto-size ports are not yet implemented.");
    }
  }

  /** A concrete %OutputPort must provide a way to allocate a suitable object
  for holding the runtime value of this output port. The particulars may depend
  on values and types of objects in the given Context, if one is supplied. It
  is up to the implementation to complain if a Context is required but wasn't
  supplied by the caller.

  @param context
     If non-null, a Context that has already been validated as compatible with
     the System whose output port this is.
  @returns A unique_ptr to the new value-holding object as an AbstractValue. **/
  virtual std::unique_ptr<AbstractValue> DoAllocate(
      const Context<T>* context) const = 0;

  /** A concrete %OutputPort must provide a way to write to `value` the value
  this output port should have given the contents of the supplied Context. The
  value may be determined by computation or by copying from a source value in
  the Context.

  @param context A Context that has already been validated as compatible with
                 the System whose output port this is.
  @param value   A pointer that has already be validated as non-null and
                 pointing to an object of the right type to hold a value of
                 this output port. **/
  virtual void DoCalc(const Context<T>& context,
                      AbstractValue* value) const = 0;

  /** A concrete %OutputPort must provide access to the current value of this
  output port stored within the given Context. If the value is already up to
  date with respect to its prerequisites in `context`, no computation should be
  performed. Otherwise, the implementation should arrange for the value to be
  computed, typically but not necessarily by invoking DoCalc().

  @param context A Context that has already been validated as compatible with
                 the System whose output port this is. **/
  virtual const AbstractValue& DoEval(const Context<T>& context) const = 0;

 private:
  const System<T>* system_{nullptr};
  OutputPortIndex index_{0};  // TODO(sherm1) Should be illegal value.
  const PortDataType data_type_;
  const int size_;
};

// TODO(sherm1) Get rid of this alias.
template <typename T>
using OutputPortDescriptor = OutputPort<T>;


//==============================================================================
//                            LEAF OUTPUT PORT
//==============================================================================
/** Adds methods for allocation, calculation, and caching of an output
port's value. **/
template <typename T>
class LeafOutputPort : public OutputPort<T> {
 public:
  /** Signature of a function suitable for allocating an object that can hold
  a value of a particular output port. The result is returned as an
  AbstractValue even if this is a vector-valued port. **/
  using AllocCallback = std::function<std::unique_ptr<AbstractValue>(
      const Context<T>*)>;

  /** Signature of a function suitable for allocating a BasicVector of the
  right size and concrete type for a particular vector-valued output port. **/
  using AllocVectorCallback = std::function<std::unique_ptr<BasicVector<T>>(
      const Context<T>*)>;

  /** Signature of a function suitable for calculating a value of a particular
  output port, given a place to put the value. **/
  using CalcCallback = std::function<void(const Context<T>&, AbstractValue*)>;

  /** Signature of a function suitable for obtaining the cached value of a
  particular output port. **/
  using EvalCallback = std::function<const AbstractValue&(const Context<T>&)>;

  /** Construct a fixed-size vector-valued, cached output port using the
  supplied BasicVector as a model for the port's value type and cache entry.
  You do not need to supply an allocation function. **/
  explicit LeafOutputPort(const BasicVector<T>& model_value)
      : OutputPort<T>(kVectorValued, model_value.size()),
        model_value_(new VectorValue<T>(model_value.Clone())) {
  }

  /** Construct an abstract-valued, cached output port using the
  supplied allocator function to provide an object to hold the value.
  If a particular size is required, supply it here; otherwise use kAutoSize.
  (Note: currently a fixed size is required.) **/
  explicit LeafOutputPort(AllocVectorCallback vector_alloc_function, int size)
      : OutputPort<T>(kVectorValued, size) {
    set_allocation_function(vector_alloc_function);
  }

  /** Construct an abstract-valued, cached output port using the
  supplied AbstractValue as a model for the port's value type and cache
  entry. You do not need to supply an allocation function; instead the model
  will be cloned as needed. **/
  explicit LeafOutputPort(const AbstractValue& model_value)
      : OutputPort<T>(kAbstractValued, 0 /* size */),
        model_value_(model_value.Clone()) {}


  /** Construct an abstract-valued, cached output port using the supplied
  allocator function to provide an object to hold the value. **/
  explicit LeafOutputPort(AllocCallback alloc_function)
      : OutputPort<T>(kAbstractValued, 0 /* size */),
        alloc_function_(alloc_function) {}

  /** Construct a port without specifying a model. If you provide an allocation
  function it will be used. Otherwise, if this is a vector port of known size
  an appropriate `BasicVector<T>` will be allocated. No default allocator is
  provided for an abstract port. **/
  LeafOutputPort(PortDataType data_type, int size,
                 AllocCallback alloc_function = nullptr)
      : OutputPort<T>(data_type, size), alloc_function_(alloc_function) {
    // If this is vector of known size, make a model to use if we don't get
    // an allocation function.
    if (data_type == kVectorValued && size >= 0) {
      model_value_.reset(
          new VectorValue<T>(std::make_unique<BasicVector<T>>(size)));
    }
  }

  /** Set or replace the allocation function for this output port, using
  a function that returns an AbstractValue. **/
  void set_allocation_function(AllocCallback alloc_function) {
    alloc_function_ = alloc_function;
  }

  /** Set or replace the allocation function for this vector-valued output port,
  using a function that returns an object derived from `BasicVector<T>`.**/
  void set_allocation_function(AllocVectorCallback vector_alloc_function) {
    if (!vector_alloc_function) {
      alloc_function_ = nullptr;
    } else {
      // Wrap the vector-returning function with an AbstractValue-returning
      // allocator.
      alloc_function_ = [vector_alloc_function](const Context<T>* context) {
        std::unique_ptr<BasicVector<T>> vector = vector_alloc_function(context);
        return std::unique_ptr<AbstractValue>(
            new VectorValue<T>(std::move(vector)));
      };
    }
  }

  /** Set or replace the calculation function for this output port, using
  a function that writes into an AbstractValue. **/
  void set_calculation_function(CalcCallback calc_function) {
    calc_function_ = calc_function;
  }

 private:
  // Invokes the supplied allocation function if there is one, otherwise clones
  // the model value if there is one, otherwise complains.
  std::unique_ptr<AbstractValue> DoAllocate(
      const Context<T>* context) const override;

  // Invokes the supplied calculation function if present, or attempts to use
  // obsolete CalcOutput() method for backwards compatibility.
  // TODO(sherm1) Get rid of CalcOutput() fallback.
  void DoCalc(const Context<T>& context, AbstractValue* value) const override;

  // If this output port has its own cache entry then this checks the validity
  // bit, invokes DoCalc() into the cache entry if necessary, and then returns
  // a reference to the cache entry's value. Otherwise it invokes a custom
  // evaluation function which must exist.
  const AbstractValue& DoEval(const Context<T>& context) const override;

  // Data.
  std::unique_ptr<const AbstractValue> model_value_;  // May be nullptr.

  AllocCallback alloc_function_;
  CalcCallback  calc_function_;
  EvalCallback  eval_function_;
};


//==============================================================================
//                          DIAGRAM OUTPUT PORT
//==============================================================================
/** Holds information about the subsystem output port that has been exported to
become one of this Diagram's output ports. The actual methods for determining
the port's value are supplied by the LeafSystem that ultimately underlies the
source port, although that may be any number of levels down. **/
template <typename T>
class DiagramOutputPort : public OutputPort<T> {
 public:
  explicit DiagramOutputPort(const OutputPort<T>* source_output_port)
      : OutputPort<T>(source_output_port->get_data_type(),
                      source_output_port->size()),
        source_output_port_(source_output_port) {}

  /** Obtain a reference to the subsystem output port that was exported to
  create this diagram port. Note that the source may itself be a diagram output
  port. **/
  const OutputPort<T>& get_source_output_port() const {
    DRAKE_DEMAND(source_output_port_ != nullptr);
    return *source_output_port_;
  }

 private:
  // These forward to the source system output port.
  std::unique_ptr<AbstractValue> DoAllocate(
      const Context<T>* context) const override {
    DRAKE_DEMAND(source_output_port_ != nullptr);
    return source_output_port_->Allocate(context);
  }
  void DoCalc(const Context<T>& context, AbstractValue* value) const override {
    DRAKE_DEMAND(source_output_port_ != nullptr);
    return source_output_port_->Calc(context, value);
  }
  const AbstractValue& DoEval(const Context<T>& context) const override {
    DRAKE_ASSERT(source_output_port_ != nullptr);
    return source_output_port_->Eval(context);
  }

  const OutputPort<T>* source_output_port_;
};

}  // namespace systems
}  // namespace drake
