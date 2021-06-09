#pragma once

#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/value.h"
#include "drake/systems/framework/context_base.h"
#include "drake/systems/framework/framework_common.h"

namespace drake {
namespace systems {

template <class MyClass, typename MyContext, typename MyOutput>
std::pair<AllocValueFunction, CalcContextBaseValueFunction> BindCalcFunction(
    const MyClass* this_ptr,
    void (MyClass::*calc)(const MyContext&, MyOutput*) const,
    const MyOutput& model_value = {}) {
  copyable_unique_ptr<AbstractValue> copyable_model_value(
      AbstractValue::Make<MyOutput>(model_value));
  auto alloc_lambda = [value = std::move(copyable_model_value)]() {
    return value->Clone();
  };
  auto calc_lambda = [this_ptr, calc](const ContextBase& context,
                                      AbstractValue* result) {
    const auto& typed_context = dynamic_cast<const MyContext&>(context);
    MyOutput& typed_result = result->get_mutable_value<MyOutput>();
    (this_ptr->*calc)(typed_context, &typed_result);
  };
  return std::make_pair(alloc_lambda, calc_lambda);
}

}  // namespace systems
}  // namespace drake
