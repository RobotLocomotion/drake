#pragma once

/** @file
Provides a portable macro for use in generating compile-time warnings for
use of code that is permitted but discouraged. */

#ifdef DRAKE_DOXYGEN_CXX
/** Use `DRAKE_DEPRECATED("removal_date", "message")` to discourage use of
certain APIs.  It can be used on classes, typedefs, variables, non-static data
members, functions, arguments, enumerations, and template specializations. When
code refers to the deprecated item, a compile time warning will be issued
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

#define DRAKE_DEPRECATED(removal_date, message)         \
  [[deprecated(                                         \
  "\nDRAKE DEPRECATED: " message                        \
  "\nThe deprecated code will be removed from Drake"    \
  " on or after " removal_date ".")]]

#endif  // DRAKE_DOXYGEN_CXX
