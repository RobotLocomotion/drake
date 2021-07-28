#pragma once

// ============================================================================
// N.B. The spelling of the macro names between doc/Doxyfile_CXX.in and this
// file must be kept in sync!
// ============================================================================

/** @file
Provides careful macros to selectively enable or disable the special member
functions for copy-construction, copy-assignment, move-construction, and
move-assignment.

http://en.cppreference.com/w/cpp/language/member_functions#Special_member_functions

When enabled via these macros, the `= default` implementation is provided.
Code that needs custom copy or move functions should not use these macros.
*/

/** DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN deletes the special member functions for
copy-construction, copy-assignment, move-construction, and move-assignment.
Drake's Doxygen is customized to render the deletions in detail, with
appropriate comments.  Invoke this macro in the public section of the class
declaration, e.g.:
<pre>
class Foo {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Foo)

  // ...
};
</pre>
*/
#define DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Classname)      \
  Classname(const Classname&) = delete;                 \
  void operator=(const Classname&) = delete;            \
  Classname(Classname&&) = delete;                      \
  void operator=(Classname&&) = delete;

/** DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN defaults the special member
functions for copy-construction, copy-assignment, move-construction, and
move-assignment.  This macro should be used only when copy-construction and
copy-assignment defaults are well-formed.  Note that the defaulted move
functions could conceivably still be ill-formed, in which case they will
effectively not be declared or used -- but because the copy constructor exists
the type will still be MoveConstructible.  Drake's Doxygen is customized to
render the functions in detail, with appropriate comments.  Invoke this macro
in the public section of the class declaration, e.g.:
<pre>
class Foo {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Foo)

  // ...
};
</pre>
*/
#define DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Classname)       \
  Classname(const Classname&) = default;                        \
  Classname& operator=(const Classname&) = default;             \
  Classname(Classname&&) = default;                             \
  Classname& operator=(Classname&&) = default;                  \
  /* Fails at compile-time if default-copy doesn't work. */     \
  static void DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE() {        \
    (void) static_cast<Classname& (Classname::*)(               \
        const Classname&)>(&Classname::operator=);              \
  }
