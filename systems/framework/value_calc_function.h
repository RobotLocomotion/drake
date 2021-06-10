#pragma once

#include <functional>
#include <memory>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context_base.h"

namespace drake {
namespace systems {
namespace internal {

class AbstractValueCloner final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AbstractValueCloner)

  /* Creates a clone functor for the given model value. */
  explicit AbstractValueCloner(std::unique_ptr<AbstractValue> model);

  ~AbstractValueCloner();

  /** Returns a Clone of the model value passed into the constructor. */
  std::unique_ptr<AbstractValue> operator()() const;

 private:
  copyable_unique_ptr<AbstractValue> model_;
};

}  // namespace internal

/** A functor that can calculate from a ContextBase into an AbstractValue,
as well as be able to pre-allocate storage for the type-erased output.

Sugar is provided to bind class member functions that use conveniently-typed
output arguments, so that the user can ignore the details of type erasure
and Context<T> downcasting.
*/
class ValueCalcFunction final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ValueCalcFunction)

  ValueCalcFunction();
  ~ValueCalcFunction();

  using AllocCallback = std::function<std::unique_ptr<AbstractValue>()>;
  using CalcCallback = std::function<void(const ContextBase&, AbstractValue*)>;

  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      void (SomeClass::*instance_func)(const SomeContext&, SomeOutput*) const)
      : ValueCalcFunction(instance, SomeOutput{}, instance_func) {}

  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      const SomeOutput& model_value,
      void (SomeClass::*instance_func)(const SomeContext&, SomeOutput*) const) {
    auto* typed_instance = dynamic_cast<const SomeClass*>(instance);
    DRAKE_DEMAND(typed_instance != nullptr);
    alloc_ = internal::AbstractValueCloner(
        AbstractValue::Make<SomeOutput>(model_value));
    calc_ = [typed_instance, instance_func](const ContextBase& context,
                                            AbstractValue* result) {
      const auto& typed_context = dynamic_cast<const SomeContext&>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*instance_func)(typed_context, &typed_result);
    };
  }

  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      std::unique_ptr<AbstractValue> (SomeClass::*alloc_fn)() const,
      void (SomeClass::*calc_fn)(const SomeContext&, SomeOutput*) const) {
    auto* typed_instance = dynamic_cast<const SomeClass*>(instance);
    DRAKE_DEMAND(typed_instance != nullptr);
    alloc_ = [typed_instance, alloc_fn]() {
      return (typed_instance->*alloc_fn)();
    };
    calc_ = [typed_instance, calc_fn](const ContextBase& context,
                                               AbstractValue* result) {
      const auto& typed_context = dynamic_cast<const SomeContext&>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*calc_fn)(typed_context, &typed_result);
    };
  }

  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  ValueCalcFunction(
      const SomeInstance* instance,
      SomeOutput (SomeClass::*alloc_fn)() const,
      void (SomeClass::*calc_fn)(const SomeContext&, SomeOutput*) const) {
    auto* typed_instance = dynamic_cast<const SomeClass*>(instance);
    DRAKE_DEMAND(typed_instance != nullptr);
    alloc_ = [typed_instance, alloc_fn]() {
      return AbstractValue::Make<SomeOutput>((typed_instance->*alloc_fn)());
    };
    calc_ = [typed_instance, calc_fn](const ContextBase& context,
                                               AbstractValue* result) {
      const auto& typed_context = dynamic_cast<const SomeContext&>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*calc_fn)(typed_context, &typed_result);
    };
  }

  template <typename SomeClass, typename SomeContext, typename SomeOutput>
  ValueCalcFunction(
      AllocCallback alloc,
      const SomeClass* typed_instance,
      void (SomeClass::*calc_fn)(const SomeContext&, SomeOutput*) const)
      : alloc_(std::move(alloc)) {
    calc_ = [typed_instance, calc_fn](const ContextBase& context,
                                      AbstractValue* result) {
      const auto& typed_context = dynamic_cast<const SomeContext&>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*calc_fn)(typed_context, &typed_result);
    };
  }

  template <typename SomeOutput>
  static ValueCalcFunction MakeAllocator() {
    return MakeAllocator<SomeOutput>(SomeOutput{});
  }

  template <typename SomeOutput>
  static ValueCalcFunction MakeAllocator(const SomeOutput& model_value) {
    ValueCalcFunction result;
    result.alloc_ = internal::AbstractValueCloner(
       AbstractValue::Make<SomeOutput>(model_value));
    result.calc_ = [](const ContextBase&, AbstractValue*) {};
    return result;
  }

  ValueCalcFunction(AllocCallback alloc, CalcCallback calc)
      : alloc_(std::move(alloc)), calc_(std::move(calc)) {}

  /** XXX */
  std::unique_ptr<AbstractValue> Allocate() const;

  /** XXX */
  void Calc(const ContextBase& context, AbstractValue* output) const;

 private:
  AllocCallback alloc_;
  CalcCallback calc_;
};

}  // namespace systems
}  // namespace drake
