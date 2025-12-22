#pragma once

#include <string>
#include <string_view>

// N.B. The spelling of the macro name between doc/Doxyfile_CXX.in and this
// file must be kept in sync.

/** @file
Provides a portable macro for use in generating compile-time warnings for
use of code that is permitted but discouraged. */

#ifdef DRAKE_DOXYGEN_CXX
/** Use `DRAKE_DEPRECATED("removal_date", "message")` to discourage use of
certain APIs. It can be used on classes, typedefs, functions, arguments,
enumerations, and template specializations. It must not be used on non-static
data members of structs or classes. Instead, add a comment on the preceding line
noting that the data member is deprecated and stating its planned removal date.
When code refers to the deprecated item, a compile time warning will be issued
displaying the given message, preceded by "DRAKE DEPRECATED: ". The Doxygen API
reference will show that the API is deprecated, along with the message.

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

#else  // DRAKE_DOXYGEN_CXX

#define DRAKE_DEPRECATED(removal_date, message)                   \
  [[deprecated("\nDRAKE DEPRECATED: " message                     \
               "\nThe deprecated code will be removed from Drake" \
               " on or after " removal_date ".")]]

#endif  // DRAKE_DOXYGEN_CXX

namespace drake {
namespace internal {

/* When constructed, logs a deprecation message; the destructor is guaranteed
to be trivial. This is useful for declaring an instance of this class as a
function-static global, so that a warning is logged the first time the program
encounters some code, but does not repeat the warning on subsequent encounters
within the same process.

For example:
<pre>
void OldCalc(double data) {
  static const drake::internal::WarnDeprecated warn_once(
      "2038-01-19", "The method OldCalc() has been renamed to NewCalc().");
  return NewCalc(data);
}
</pre> */
class [[maybe_unused]] WarnDeprecated {
 public:
  /* The removal_date must be in the form YYYY-MM-DD. */
  WarnDeprecated(std::string_view removal_date, std::string_view message);
};

}  // namespace internal
}  // namespace drake
