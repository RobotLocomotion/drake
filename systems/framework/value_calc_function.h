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

/** A functor that can calculate from a ContextBase into an AbstractValue,
as well as be able to pre-allocate storage for the type-erased output.

Sugar is provided to bind class member functions that use conveniently-typed
output arguments, so that the user can ignore the details of type erasure
and Context<T> downcasting.
*/
struct ValueCalcFunction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ValueCalcFunction)

  using AllocCallback = std::function<std::unique_ptr<AbstractValue>()>;
  using CalcCallback = std::function<void(const ContextBase&, AbstractValue*)>;

  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  static ValueCalcFunction Bind(
      const SomeInstance* instance,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const,
      const SomeOutput& model_value = {}) {
    auto* typed_instance = dynamic_cast<const SomeClass*>(instance);
    DRAKE_DEMAND(typed_instance != nullptr);
    copyable_unique_ptr<AbstractValue> copyable_model_value(
        AbstractValue::Make<SomeOutput>(model_value));
    auto alloc_lambda = [value = std::move(copyable_model_value)]() {
      return value->Clone();
    };
    auto calc_lambda = [typed_instance, calc](const ContextBase& context,
                                              AbstractValue* result) {
      const auto& typed_context = dynamic_cast<const SomeContext&>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*calc)(typed_context, &typed_result);
    };
    return {std::move(alloc_lambda), std::move(calc_lambda)};
  }

  template <class SomeInstance, typename SomeClass, typename SomeContext,
            typename SomeOutput>
  static ValueCalcFunction Bind(
      const SomeInstance* instance,
      SomeOutput (SomeClass::*alloc)() const,
      void (SomeClass::*calc)(const SomeContext&, SomeOutput*) const) {
    auto* typed_instance = dynamic_cast<const SomeClass*>(instance);
    DRAKE_DEMAND(typed_instance != nullptr);
    auto alloc_lambda = [typed_instance, alloc]() {
      return AbstractValue::Make<SomeOutput>((typed_instance->*alloc)());
    };
    auto calc_lambda = [typed_instance, calc](const ContextBase& context,
                                              AbstractValue* result) {
      const auto& typed_context = dynamic_cast<const SomeContext&>(context);
      SomeOutput& typed_result = result->get_mutable_value<SomeOutput>();
      (typed_instance->*calc)(typed_context, &typed_result);
    };
    return {std::move(alloc_lambda), std::move(calc_lambda)};
  }

  AllocCallback alloc;
  CalcCallback calc;
};

}  // namespace systems
}  // namespace drake
