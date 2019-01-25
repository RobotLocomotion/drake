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
appropriate comments.  Invoke this this macro in the public section of the
class declaration, e.g.:
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
render the functions in detail, with appropriate comments.  Invoke this this
macro in the public section of the class declaration, e.g.:
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

/** DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN declares (but does not define) the
special member functions for copy-construction, copy-assignment,
move-construction, and move-assignment.  Drake's Doxygen is customized to
render the declarations with appropriate comments.

This is useful when paired with DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T
to work around https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 whereby the
declaration and definition must be split.  Once Drake no longer supports GCC
versions prior to 6.3, this macro could be removed.

Invoke this this macro in the public section of the class declaration, e.g.:
<pre>
template <typename T>
class Foo {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(Foo)

  // ...
};
DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(Foo)
</pre>
*/
#define DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(Classname)       \
  Classname(const Classname&);                                          \
  Classname& operator=(const Classname&);                               \
  Classname(Classname&&);                                               \
  Classname& operator=(Classname&&);                                    \
  /* Fails at compile-time if default-copy doesn't work. */             \
  static void DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE() {                \
    (void) static_cast<Classname& (Classname::*)(                       \
        const Classname&)>(&Classname::operator=);                      \
  }

/** Helper for DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN.  Provides defaulted
definitions for the four special member functions of a templated class.
*/
#define DRAKE_DEFINE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN_T(Classname)      \
  template <typename T>                                                 \
  Classname<T>::Classname(const Classname<T>&) = default;               \
  template <typename T>                                                 \
  Classname<T>& Classname<T>::operator=(const Classname<T>&) = default; \
  template <typename T>                                                 \
  Classname<T>::Classname(Classname<T>&&) = default;                    \
  template <typename T>                                                 \
  Classname<T>& Classname<T>::operator=(Classname<T>&&) = default;
