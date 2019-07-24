#pragma once

#include "drake/common/drake_deprecated.h"

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
  explicit ExampleCppClass(int) {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use ExampleCppClass(double).")
  explicit ExampleCppClass(double) {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use DeprecatedMethod().")
  void DeprecatedMethod() {}

  /// Good property.
  int prop{};

  /// Good overload.
  void overload() {}

  DRAKE_DEPRECATED("2038-01-19", "Do not use overload(int).")
  void overload(int) {}
};

}  // namespace example_class
}  // namespace drake
