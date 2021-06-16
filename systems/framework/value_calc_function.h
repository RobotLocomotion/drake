#pragma once

#include <functional>
#include <memory>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/abstract_value_cloner.h"
#include "drake/systems/framework/context_base.h"

namespace drake {
namespace systems {

/** A functor that calculates from a const ContextBase input into a mutable
AbstractValue output; the functor can also allocate properly-typed storage for
the output value.

For example, given this example calculator function:

@code
  void MyCalc(const Context<T>& context, std::string* output) {
    *output = std::to_string(context.get_time());
  }
@endcode

We can capture it into a functor and then call it:

@code
  ValueCalcFunction functor(&MyCalc);
  std::unique_ptr<AbstractValue> storage = functor.Allocate();
  const Context<T> context;
  functor.Calc(context, storage.get());
  EXPECT_EQ(storage->get_value<std::string>, "0.0");
@endcode

Sugar is provided to create %ValueCalcFunction objects from function pointers
that operate on un-erased types, so that the user can ignore the details of
type erasure and Context<T> downcasting. */
class ValueCalcFunction final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ValueCalcFunction)

  /** Signature of a function suitable for allocating an object that can hold
  a value compatible with our Calc function. The result is always returned as
  an AbstractValue but must contain the correct concrete type. */
  using AllocateCallback = std::function<std::unique_ptr<AbstractValue>()>;

  /** Signature of a function suitable for calculating a context-dependent
  value, given a place to put the value. The function's second arugment is
  output-only, i.e., the function should not presume any particular value
  when called, and should fully overwrite the output with a new value. */
  using CalcCallback = std::function<void(const ContextBase&, AbstractValue*)>;

  /** Creates an invalid object; calls to Allocate or Calc will throw. */
  ValueCalcFunction();

  /** @name Constructor overloads
  @anchor ValueCalcFunction_constructors

  Create a ValueCalcFunction by providing it a with calculation callback and
  (if necessary) a way to allocate storage, in cases where the storage cannot
  be default constructed.

  In the simplest case, your calculator function would look like this:
  @code
    void MyCalc(const Context<T>& context, std::string* output) {
      *output = std::to_string(context.get_time());
    }
  @endcode
  and wrapping it would look like this:
  @code
    ValueCalcFunction foo = ValueCalcFunction(&MyCalc);
  @endcode

  In many cases, your calculator function would be a class member function
  like this:
  @code
    class MyClass {
      void MyCalc(const Context<T>& context, std::string* output) const {
        *output = std::to_string(context.get_time());
      }
    };
  @endcode
  and wrapping it would look like this:
  @code
    MyClass my_class;
    ValueCalcFunction foo = ValueCalcFunction(&my_class, &MyClass::MyCalc);
  @endcode

  If the type of the output value is cheap to copy, then the function may
  return it by-value, instead of as an output argument:
  @code
    class MyClass {
      double MyCalc(const Context<T>& context) const {
        return context.get_time();
      }
    };

    MyClass my_class;
    ValueCalcFunction foo = ValueCalcFunction(&my_class, &MyClass::MyCalc);
  @endcode

  If the type of the output is not default constructible, then you must provide
  ValueCalcFunction with an example value to use for pre-allocation:
  @code
    class MyClass {
      void MyCalc(const Context<T>& context, BasicVector<T>* output) const {
        output->get_mutable_value()[0] = context.get_time();
      }
    };

    MyClass my_class;
    BasicVector<T> model_value(1);
    ValueCalcFunction foo = ValueCalcFunction(
        &my_class, model_value, &MyClass::MyCalc);
  @endcode

  In the rare case that you cannot provide an example value when creating the
  ValueCalcFunction, you may instead provide an allocation callback. Refer to
  more specific documentation below for an example.

  For ease of use, the constructor offers overloads to specify the allocate and
  calc functions.

  The permitted argument types to specify Calc are:

  - (1) `calc` is a member function pointer with an output argument
  - (2) `calc` is a member function pointer with a return value &dagger;
  - (3) `calc` is a std::function with an output argument
  - (4) `calc` is a std::function with a return value &dagger;
  - (5) `calc` is a generic CalcCallback

  &dagger; Do not (2) or (4) use unless SomeOutput is cheap to copy.

  The permitted argument types to specify Allocate are:

  - (a) `allocate` is via the default constructor
  - (b) `allocate` is via user-supplied model_value
  - (c) `allocate` is a member function pointer
  - (d) `allocate` is a generic AllocateCallback

  All combinations of (1..5) x (a..d) are permitted, except for (5a) because
  the output type cannot be inferred from a generic CalcCallback.

  All member function pointers must refer to member functions declared *const*.

  The constructors take the following arguments, in order:

  - The `instance` pointer (iff member function callback(s) are used).
  - The `model_value` example or `allocate` callback (may be omitted).
  - The `calc` callback (required).

  If either `allocate` or `calc` is a member function pointer, then the class
  `instance` to bind must be the first argument. This `instance` pointer is
  aliased by the ValueCalcFunction, so must outlive this object.

  The `model_value` and `allocate` callback may be omitted when the OutputType
  is default constructible; this is the most common case. The need for a custom
  `allocate` function (i.e., options c or d, above) is extremely rare; prefer
  to use the default constructor (option a) or model_value (option b) in almost
  all cases.
  
  @tparam SomeContext the subclass of ContextBase required by the calc callback
  @tparam SomeOutput the output type of the calc callback
  @tparam SomeClass the static type of the class to receive the callback(s)
  @tparam SomeInstance the type of the instance to receive the callback(s);
     must be castable into SomeClass

  @throws std::exception if any argument is nullptr. */
  //@{

  /** Overload (1a). This is the best choice. Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const)
      : allocate_(make_allocate_mode_a<SomeOutput>()),
        calc_(make_calc_mode_1(instance, calc)) {}

  /** Overload (1b). This is the second-best choice. Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      const SomeOutput& model_value,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const)
      : allocate_(make_allocate_mode_b(model_value)),
        calc_(make_calc_mode_1(instance, calc)) {}

  /** Overload (1c). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      std::unique_ptr<SomeOutput> (SomeClass::*allocate)() const,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const)
      : allocate_(make_allocate_mode_c(instance, allocate)),
        calc_(make_calc_mode_1(instance, calc)) {}

  /* Overload (1d). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      AllocateCallback allocate,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const)
      : allocate_(std::move(allocate)),
        calc_(make_calc_mode_1(instance, calc)) {}

  /** Overload (2a). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      SomeOutput (SomeClass::*calc)(const SomeContext&) const)
      : allocate_(make_allocate_mode_a<SomeOutput>()),
        calc_(make_calc_mode_2(instance, calc)) {}

  /** Overload (2b). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      const SomeOutput& model_value,
      SomeOutput (SomeClass::*calc)(const SomeContext&) const)
      : allocate_(make_allocate_mode_b(model_value)),
        calc_(make_calc_mode_2(instance, calc)) {}

  /** Overload (2c). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      std::unique_ptr<SomeOutput> (SomeClass::*allocate)() const,
      SomeOutput (SomeClass::*calc)(const SomeContext&) const)
      : allocate_(make_allocate_mode_c(instance, allocate)),
        calc_(make_calc_mode_2(instance, calc)) {}

  /** Overload (2d). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      AllocateCallback allocate,
      SomeOutput (SomeClass::*calc)(const SomeContext&) const)
      : allocate_(std::move(allocate)),
        calc_(make_calc_mode_2(instance, calc)) {}

  /** Overload (3a). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeContext, typename SomeOutput>
  explicit ValueCalcFunction(
      std::function<void(const SomeContext&, SomeOutput*)> calc)
      : allocate_(make_allocate_mode_a<SomeOutput>()),
        calc_(make_calc_mode_3(std::move(calc))) {}

  /** Overload (3b). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeContext, typename SomeOutput>
  ValueCalcFunction(
      const SomeOutput& model_value,
      std::function<void(const SomeContext&, SomeOutput*)> calc)
      : allocate_(make_allocate_mode_b(model_value)),
        calc_(make_calc_mode_3(std::move(calc))) {}

  /** Overload (3c). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      std::unique_ptr<SomeOutput> (SomeClass::*allocate)() const,
      std::function<void(const SomeContext&, SomeOutput*)> calc)
      : allocate_(make_allocate_mode_c(instance, allocate)),
        calc_(make_calc_mode_3(std::move(calc))) {}

  /** Overload (3d). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeContext, typename SomeOutput>
  ValueCalcFunction(
      AllocateCallback allocate,
      std::function<void(const SomeContext&, SomeOutput*)> calc)
      : allocate_(std::move(allocate)),
        calc_(make_calc_mode_3(std::move(calc))) {}

  /** Overload (4a). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeContext, typename SomeOutput>
  explicit ValueCalcFunction(
      std::function<SomeOutput(const SomeContext&)> calc)
      : allocate_(make_allocate_mode_a<SomeOutput>()),
        calc_(make_calc_mode_4(std::move(calc))) {}

  /** Overload (4b). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeContext, typename SomeOutput>
  ValueCalcFunction(
      const SomeOutput& model_value,
      std::function<SomeOutput(const SomeContext&)> calc)
      : allocate_(make_allocate_mode_b(model_value)),
        calc_(make_calc_mode_4(std::move(calc))) {}

  /** Overload (4c). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      std::unique_ptr<SomeOutput> (SomeClass::*allocate)() const,
      std::function<SomeOutput(const SomeContext&)> calc)
      : allocate_(make_allocate_mode_c(instance, allocate)),
        calc_(make_calc_mode_4(std::move(calc))) {}

  /** Overload (4d). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeContext, typename SomeOutput>
  ValueCalcFunction(
      AllocateCallback allocate,
      std::function<SomeOutput(const SomeContext&)> calc)
      : allocate_(std::move(allocate)),
        calc_(make_calc_mode_4(std::move(calc))) {}

  // Overload (5a) is omitted because we cannot infer the type of SomeOutput
  // from a generic CalcCallback.

  /** Overload (5b). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <typename SomeOutput>
  ValueCalcFunction(
      const SomeOutput& model_value, CalcCallback calc,
      // NOLINTNEXTLINE(whitespace/line_length)
      std::enable_if_t<!std::is_convertible_v<SomeOutput, AllocateCallback>>* = 0)
      : allocate_(make_allocate_mode_b(model_value)),
        calc_(std::move(calc)) {}

  /** Overload (5c). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  template <class SomeInstance, typename SomeClass, typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      std::unique_ptr<SomeOutput> (SomeClass::*allocate)() const,
      CalcCallback calc)
      : allocate_(make_allocate_mode_c(instance, allocate)),
        calc_(std::move(calc)) {}

  /** Overload (5d). Refer to the
  @ref ValueCalcFunction_constructors "Constructor overloads" for details. */
  ValueCalcFunction(AllocateCallback allocate, CalcCallback calc);

  //@}

  ~ValueCalcFunction();

  /** Returns true iff the allocate and calc callbacks are both non-null.
  (The only way they can be null is if the ValueCalcFunction was default
  constructed or moved from.) */
  bool is_valid() const;

  /** This static function is provided for users who need an empty CalcCallback.
  Passing `&ValueCalcFunction::NoopCalc` as ValueCalcFunction's last constructor
  argument will create a function that does not compute anything, but can still
  allocate. */
  static void NoopCalc(const ContextBase&, AbstractValue*);

  /** Invokes the allocate function provided to the constructor.
  @throws std::exception if is_valid is false. */
  std::unique_ptr<AbstractValue> Allocate() const;

  /** Invokes the calc function provided to the constructor.
  @throws std::exception if is_valid is false. */
  void Calc(const ContextBase& context, AbstractValue* output) const;

 private:
  template <typename SomeClass, class SomeInstance>
  static const SomeClass* instance_cast(const SomeInstance* instance) {
    // XXX Improve the error reporting.
    const auto* typed_instance = dynamic_cast<const SomeClass*>(instance);
    DRAKE_DEMAND(typed_instance != nullptr);
    return typed_instance;
  }

  template <class SomeContext>
  static const SomeContext& context_cast(const ContextBase& context) {
    // XXX Improve the error reporting.
    return dynamic_cast<const SomeContext&>(context);
  }

  // For overload series (1).
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  CalcCallback make_calc_mode_1(
      const SomeInstance* instance,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const) {
    const SomeClass* typed_instance = instance_cast<SomeClass>(instance);
    return [typed_instance, calc](const ContextBase& context,
                                  AbstractValue* result) {
      const SomeContext& typed_context = context_cast<SomeContext>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*calc)(typed_context, &typed_result);
    };
  }

  // For overload series (2).
  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  CalcCallback make_calc_mode_2(
      const SomeInstance* instance,
      SomeOutput (SomeClass::*calc)(const SomeContext&) const) {
    const SomeClass* typed_instance = instance_cast<SomeClass>(instance);
    return [typed_instance, calc](const ContextBase& context,
                                  AbstractValue* result) {
      const SomeContext& typed_context = context_cast<SomeContext>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      typed_result = (typed_instance->*calc)(typed_context);
    };
  }

  // For overload series (3).
  template <typename SomeContext, typename SomeOutput>
  CalcCallback make_calc_mode_3(
      std::function<void(const SomeContext&, SomeOutput*)>&& calc) {
    return [captured_calc = std::move(calc)](const ContextBase& context,
                                             AbstractValue* result) {
      const SomeContext& typed_context = context_cast<SomeContext>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      captured_calc(typed_context, &typed_result);
    };
  }

  // For overload series (4).
  template <typename SomeContext, typename SomeOutput>
  CalcCallback make_calc_mode_4(
      std::function<SomeOutput(const SomeContext&)>&& calc) {
    return [captured_calc = std::move(calc)](const ContextBase& context,
                                             AbstractValue* result) {
      const SomeContext& typed_context = context_cast<SomeContext>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      typed_result = captured_calc(typed_context);
    };
  }

  // For overload series (a).
  template <typename SomeOutput>
  AllocateCallback make_allocate_mode_a() {
    return static_cast<std::unique_ptr<drake::AbstractValue>(*)()>(
        &AbstractValue::Make<SomeOutput>);
  }

  // For overload series (b).
  template <typename SomeOutput>
  AllocateCallback make_allocate_mode_b(const SomeOutput& model_value) {
    return internal::AbstractValueCloner(model_value);
  }

  // For overload series (c).
  template <class SomeInstance, typename SomeClass, typename SomeOutput>
  AllocateCallback make_allocate_mode_c(
      const SomeInstance* instance,
      std::unique_ptr<SomeOutput> (SomeClass::*allocate)() const) {
    const SomeClass* typed_instance = instance_cast<SomeClass>(instance);
    return [typed_instance, allocate]() {
      // XXX Should we always copy the return value to prevent aliasing?
      return std::make_unique<Value<SomeOutput>>(
          (typed_instance->*allocate)());
    };
  }

  AllocateCallback allocate_;
  CalcCallback calc_;
};

}  // namespace systems
}  // namespace drake
