#pragma once

#include <string>
#include <string_view>

// N.B. The spelling of the macro name between doc/Doxyfile_CXX.in and this
// file must be kept in sync.

/** @file
Provides a portable macro for use in generating compile-time warnings for
use of code that is permitted but discouraged. */
#if defined(DRAKE_DOXYGEN_CXX)
/** Use `DRAKE_DEPRECATED("removal_date", "message")` to discourage use of
certain APIs. It can be used on classes, typedefs, functions, arguments,
enumerations, and template specializations. It must not be used on non-static
data members of structs or classes. Instead, add a comment on the preceding line
noting that the data member is deprecated and stating its planned removal date.
When code refers to the deprecated item, a compile time warning will be issued
displaying the given message, preceded by "DRAKE DEPRECATED: ". The Doxygen API
reference will show that the API is deprecated, along with the message.

The compile-time severity of the warning can be controlled via the
`DRAKE_DEPRECATION_IS_ERROR` and `DRAKE_DEPRECATION_IS_SILENT` preprocessor
defines; see @ref deprecation_severity for details.

This is typically used for constructs that have been replaced by something
better and it is good practice to suggest the appropriate replacement in the
deprecation message. Deprecation warnings are conventionally used to convey to
users that a feature they are depending on will be removed in a future release.

Every deprecation notice must include a date (as YYYY-MM-DD string) where the
deprecated item is planned for removal.  Future commits may change the date
(e.g., delaying the removal) but should generally always reflect the best
current expectation for removal.

Absent any other particular need, we prefer to use a deprecation period of
three months by default, often rounded up to the next first of the month.  So
for code announced as deprecated on 2018-01-22 the removal_date would nominally
be set to 2018-05-01.

Try to keep the date string immediately after the DRAKE_DEPRECATED macro name,
even if the message itself must be wrapped to a new line:
@code
  DRAKE_DEPRECATED("2038-01-19",
      "foo is being replaced with a safer, const-method named bar().")
  int foo();
@endcode

Sample uses: @code
  // Attribute comes *before* declaration of a deprecated function or variable;
  // no semicolon is allowed.
  DRAKE_DEPRECATED("2038-01-19", "f() is slow; use g() instead.")
  int f(int arg);

  // Attribute comes *after* struct, class, enum keyword.
  class DRAKE_DEPRECATED("2038-01-19", "Use MyNewClass instead.")
  MyClass {
  };

  // Type alias goes before the '='.
  using OldType
      DRAKE_DEPRECATED("2038-01-19", "Use NewType instead.")
      = NewType;
@endcode
*/
#define DRAKE_DEPRECATED(removal_date, message)

#elif defined(DRAKE_DEPRECATION_IS_SILENT) && \
    defined(DRAKE_DEPRECATION_IS_ERROR)
#error Conflicting deprecation warnings flags
#elif defined(DRAKE_DEPRECATION_IS_SILENT)
#define DRAKE_DEPRECATED(removal_date, message)

#elif defined(DRAKE_DEPRECATION_IS_ERROR)
#if defined(__clang__)
#define DRAKE_DEPRECATED(removal_date, message)                     \
  [[clang::unavailable("\nDRAKE DEPRECATED: " message               \
                       "\nThe deprecated code will be removed from" \
                       " Drake on or after " removal_date ".")]]
#else
#define DRAKE_DEPRECATED(removal_date, message)                   \
  [[gnu::unavailable("\nDRAKE DEPRECATED: " message               \
                     "\nThe deprecated code will be removed from" \
                     " Drake on or after " removal_date ".")]]
#endif

#else
#define DRAKE_DEPRECATED(removal_date, message)                   \
  [[deprecated("\nDRAKE DEPRECATED: " message                     \
               "\nThe deprecated code will be removed from Drake" \
               " on or after " removal_date ".")]]

#endif  // DRAKE_DOXYGEN_CXX

namespace drake {
namespace internal {

/** When constructed, logs a deprecation message; the destructor is guaranteed
to be trivial. This is useful for declaring an instance of this class as a
function-static global, so that a warning is logged the first time the program
encounters some code, but does not repeat the warning on subsequent encounters
within the same process.

For example:
@code{.cc}
void OldCalc(double data) {
  static const drake::internal::WarnDeprecated warn_once(
      "2038-01-19", "The method OldCalc() has been renamed to NewCalc().");
  return NewCalc(data);
}
@endcode

The runtime severity of the warning can be controlled via the
`DRAKE_DEPRECATION_IS_ERROR` and `DRAKE_DEPRECATION_IS_SILENT` environment
variables; see @ref deprecation_severity for details. */
class [[maybe_unused]] WarnDeprecated {
 public:
  /** The removal_date must be in the form YYYY-MM-DD. */
  WarnDeprecated(std::string_view removal_date, std::string_view message);
};

}  // namespace internal

/** @defgroup deprecation_severity Deprecation Severity Controls
@ingroup environment_variables
@{

Drake provides two mechanisms to control the severity of deprecated API usage,
both using the same identifier names: a compile-time C++ preprocessor define
and a runtime environment variable.

<h3>DRAKE_DEPRECATION_IS_ERROR</h3>

At compile time, when `-DDRAKE_DEPRECATION_IS_ERROR` is defined on a target,
the DRAKE_DEPRECATED() C++ attribute expands to `[[clang::unavailable(...)]]`
or `[[gnu::unavailable(...)]]`, making any call to a deprecated Drake API a
hard compile error.

At runtime, when set to `"1"` as an environment variable, deprecation warnings
from drake::internal::WarnDeprecated and the pydrake equivalent are thrown as
exceptions instead of being logged.

<h3>DRAKE_DEPRECATION_IS_SILENT</h3>

At compile time, when `-DDRAKE_DEPRECATION_IS_SILENT` is defined on a target,
the DRAKE_DEPRECATED() C++ attribute expands to a no-op, silencing all
compile-time deprecation warnings.

At runtime, when set to `"1"` as an environment variable, deprecation warnings
are silently discarded instead of being logged.

---

The two values are mutually exclusive — defining or setting both simultaneously
is an error.

Compile-time defines should be applied per-target (not globally), so they do
not affect Drake's own compilation units:

Bazel:
@code{.py}
cc_binary(
    name = "my_app",
    local_defines = ["DRAKE_DEPRECATION_IS_ERROR"],
    deps = ["@drake//..."],
)
@endcode

CMake:
@code{.cmake}
target_compile_definitions(my_app PRIVATE DRAKE_DEPRECATION_IS_ERROR)
@endcode

@} */

}  // namespace drake
