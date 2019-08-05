drake/common - note to developers
=================================

Use this directory for general purpose C++ utilities that are used throughout Drake. Appropriate contents:
- general purpose macros
- utility classes that hide platform dependencies
- generally-useful containers and adapters
- preferred aliases for long type names
- predefined constants
- and similar code.

Do not include:
- Example programs
- Code that is specific to a Drake subarea that could be localized to that subarea.

Put unit tests for the above in the `test` subdirectory. Tests should have names that match the component being tested, with `_test` added at the end. Example: `common/some_useful_stuff.h` has test code `common/test/some_useful_stuff_test.cc` that generates an executable named `some_useful_stuff_test`.

Don't forget to add Doxygen documentation for every class and method explaining what they are for. Provide an `@code` example of how the utility should be used unless it is obvious.
