#pragma once

#include <functional>
#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context_base.h"

namespace drake {
namespace systems {

/** %ValueProducer computes an AbstractValue output based on a ContextBase
input.

It provides two functions for that purpose:
- Allocate() returns new storage that is suitably typed to hold the output.
- Calc() takes a context as input and writes to an output pointer.

@note At the moment, this class only serves to consolidate its two function
objects within single class. In the future, we will add more convenient
syntactic sugar in support of simpler callback functions. */
class ValueProducer final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ValueProducer)

  /** Signature of a function suitable for allocating an object that can hold
  a value compatible with our Calc function. The result is always returned as
  an AbstractValue but must contain the correct concrete type. */
  using AllocateCallback = std::function<std::unique_ptr<AbstractValue>()>;

  /** Signature of a function suitable for calculating a context-dependent
  value, given a place to put the value. The function may presume that the
  storage pointed to by the second argument will be of the proper type (as
  returned by an AllocateCallback), but should not presume that the storage
  has been initialized with any particular value; the function should always
  fully overwrite the output storage with a new value. */
  using CalcCallback = std::function<void(const ContextBase&, AbstractValue*)>;

  /** Creates an invalid object; calls to Allocate or Calc will throw. */
  ValueProducer();

  /** Creates an object that allocates and calculates using the given functors.
  @throws std::exception if either argument is nullptr. */
  ValueProducer(AllocateCallback allocate, CalcCallback calc);

  // TODO(jwnimmer-tri) Add constructor sugar for using model values and/or
  // member function pointers.

  ~ValueProducer();

  /** Returns true iff the allocate and calc callbacks are both non-null.
  (The only way they can be null is if the ValueProducer was default constructed
  or moved from.) */
  bool is_valid() const;

  /** This static function is provided for users who need an empty CalcCallback.
  Passing `&ValueProducer::NoopCalc` as ValueProducer's last constructor
  argument will create a function that does not compute anything, but can still
  allocate. */
  static void NoopCalc(const ContextBase&, AbstractValue*);

  /** Invokes the allocate function provided to the constructor.
  @throws std::exception if is_valid() is false. */
  std::unique_ptr<AbstractValue> Allocate() const;

  /** Invokes the calc function provided to the constructor.
  @throws std::exception if is_valid() is false. */
  void Calc(const ContextBase& context, AbstractValue* output) const;

 private:
  AllocateCallback allocate_;
  CalcCallback calc_;
};

}  // namespace systems
}  // namespace drake
