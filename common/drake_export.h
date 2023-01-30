#pragma once

/** DRAKE_NO_EXPORT sets C++ code to use hidden linker visibility.

Hidden visibility is appropriate for code that will be completely invisible to
users, e.g., for header files that are bazel-private, not installed, and only
used as implementation_deps.

This macro is most useful when Drake code includes externals that themselves
have hidden linker visibility and compilers complain about mismatched
visibility attributes.

For example, to un-export all classes and functions in a namespace:
<pre>
namespace internal DRAKE_NO_EXPORT {
class Foo {
  // ...
};
}  // namespace internal
</pre>

To un-export just one class:
<pre>
namespace internal {
class DRAKE_NO_EXPORT Foo {
  // ...
};
}  // namespace internal
</pre>

To un-export just one function:
<pre>
DRAKE_NO_EXPORT void CalcFoo(double arg) { ... }
</pre>

For the related CMake module, see:
https://cmake.org/cmake/help/latest/module/GenerateExportHeader.html
*/
#define DRAKE_NO_EXPORT __attribute__((visibility("hidden")))
