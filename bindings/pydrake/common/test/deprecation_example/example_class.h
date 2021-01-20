#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"

namespace drake {
namespace example_class {

/// Example class.
class ExampleCppClass {
 public:
  /// Good constructor.
  ExampleCppClass() {}

  // N.B. This spacer is to ensure that the class's documentation does not pick
  // up the following deprecation below.

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(int).")
  explicit ExampleCppClass(int x) {
    // In a non-trivial case, this function body would have
    // backwards-compatible functionality based on the value of `x`. However,
    // since this is trivial, we instead pass it to `unused`.
    unused(x);
  }

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(double).")
  explicit ExampleCppClass(double y) {
    // In a non-trivial case, this function body would have
    // backwards-compatible functionality based on the value of `y`. However,
    // since this is trivial, we instead pass it to `unused`.
    unused(y);
  }

  DRAKE_DEPRECATED("2038-01-19", "Do not use DeprecatedMethod().")
  void DeprecatedMethod() {}

  /// Good overload.
  void overload() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use overload(int).")
  void overload(int x) {
    // In a non-trivial example, this function body would have
    // backwards-compatible functionality based on the value of `x`. However,
    // since this is trivial, we instead pass it to `unused`.
    unused(x);
  }

  /// Good property.
  int prop{};
};

/// Serves as an example for binding (and deprecating) a simple struct. This
/// allows the struct to be constructed with ParamInit and deprecated using
/// the corresponding DeprecatedParamInit.
struct DRAKE_DEPRECATED(
    "2038-01-19", "Do not use ExampleCppStruct") ExampleCppStruct {
  int i{};
  int j{};
};

}  // namespace example_class
}  // namespace drake
