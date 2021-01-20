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
  explicit ExampleCppClass(int x) { unused(x); }

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(double).")
  explicit ExampleCppClass(double y) { unused(y); }

  DRAKE_DEPRECATED("2038-01-19", "Do not use DeprecatedMethod().")
  void DeprecatedMethod() {}

  /// Good overload.
  void overload() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use overload(int).")
  void overload(int x) { unused(x); }

  /// Good property.
  int prop{};

  // N.B. It doesn't really make sense to show a DRAKE_DEPRECATED property here
  // because that causes the whole class to be deprecated.
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
