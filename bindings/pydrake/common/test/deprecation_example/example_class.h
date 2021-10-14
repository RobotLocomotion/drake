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
    // This toy function does no actual work where a real function would do
    // work with the parameters. Using unused minimally satisfies the
    // compiler's need that all named parameters be "used". We can't simply
    // omit the parameter name because python doc extraction depends on those
    // names.
    unused(x);
  }

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(double).")
  explicit ExampleCppClass(double y) {
    // This toy function does no actual work where a real function would do
    // work with the parameters. Using unused minimally satisfies the
    // compiler's need that all named parameters be "used". We can't simply
    // omit the parameter name because python doc extraction depends on those
    // names.
    unused(y);
  }

  DRAKE_DEPRECATED("2038-01-19", "Do not use DeprecatedMethod.")
  void DeprecatedMethod() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use DeprecatedMethod.")
  void DeprecatedMethod(int) {}

  /// Good overload.
  void overload() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use overload(int).")
  void overload(int x) {
    // This toy function does no actual work where a real function would do
    // work with the parameters. Using unused minimally satisfies the
    // compiler's need that all named parameters be "used". We can't simply
    // omit the parameter name because python doc extraction depends on those
    // names.
    unused(x);
  }

  // This will be bound with two spellings in Python:
  // - FunctionWithArgumentName(new_name) - not deprecated
  // - FunctionWithArgumentName(old_name) - deprecated
  int FunctionWithArgumentName(int new_name) { return new_name + 1; }

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
