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
- Matlab code
- Example programs
- Code that is specific to a Drake subarea that could be localized to that subarea.

Put unit tests for the above in the `test` subdirectory. Test executables should begin with `test-` so they will be easy to locate in an IDE.

Don't forget to add Doxygen documentation for every class and method explaining what they are for. Provide a `@code` example of how the utility should be used unless it is obvious.
