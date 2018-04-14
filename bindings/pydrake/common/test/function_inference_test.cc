#include "drake/bindings/pydrake/common/function_inference.h"

#include <functional>
#include <type_traits>

#include <gtest/gtest.h>

namespace drake {
namespace pydrake {

using detail::infer_function_info;

// Compares types, generating a static_assert that should have a helpful
// context of which types do not match.
template <typename Actual, typename Expected>
void check_type() {
  // Use this function to inspect types when failure is encountered.
  static_assert(std::is_same<Actual, Expected>::value, "Mismatch");
}

// Checks signature of a generic functor.
template <typename... Actual, typename... Expected>
void check_pack(type_pack<Actual...>, type_pack<Expected...>) {
  using Dummy = int[];
  (void)Dummy{(check_type<Actual, Expected>(), 0)...};
}

template <typename ReturnExpected, typename... ArgsExpected, typename InfoT>
void check_signature(InfoT&& info) {
  using Info = std::decay_t<InfoT>;
  check_type<typename Info::Return, ReturnExpected>();
  check_pack(typename Info::Args{}, type_pack<ArgsExpected...>{});
}

// Simple function.
void IntToVoid(int) {}

GTEST_TEST(WrapFunction, FunctionPointer) {
  auto info = infer_function_info(IntToVoid);
  check_signature<void, int>(info);
  int value = 1;
  info.func(value);
}

// Lambdas / basic functors.
GTEST_TEST(WrapFunction, Lambda) {
  int value{0};
  auto lambda = [](int) {};
  {
    auto info = infer_function_info(lambda);
    check_signature<void, int>(info);
    info.func(value);
  }

  {
    std::function<void(int)> func = lambda;
    auto info = infer_function_info(func);
    info.func(value);
  }
}

// Class methods.
class MyClass {
 public:
  static int MethodStatic(int value) { return value; }
  int MethodMutable(int value) { return value + value_; }
  int MethodConst(int value) const { return value * value_; }

 private:
  int value_{10};
};

GTEST_TEST(WrapFunction, Methods) {
  int value = 2;

  MyClass c;
  const MyClass& c_const{c};

  auto info1 = infer_function_info(&MyClass::MethodStatic);
  check_signature<int, int>(info1);
  EXPECT_EQ(info1.func(value), 2);

  auto info2 = infer_function_info(&MyClass::MethodMutable);
  check_signature<int, MyClass&, int>(info2);
  EXPECT_EQ(info2.func(c, value), 12);

  auto info3 = infer_function_info(&MyClass::MethodConst);
  check_signature<int, const MyClass&, int>(info3);
  EXPECT_EQ(info3.func(c_const, value), 20);
}

}  // namespace pydrake
}  // namespace drake
