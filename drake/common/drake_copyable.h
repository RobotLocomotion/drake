#pragma once

#include "drake/common/drake_compat.h"

/// @file
/// Provides careful macros to selectively enable or disable copy-construction
/// and copy-assignment and/or move-construction and move-assignment.  When
/// enabled, the `= default` implementation is provided.

// Note that doc/Doxyfile_CXX.in must be kept in sync with this file!

#define DRAKE_COPYABLE_INTERNAL_NO_COPY(Classname)      \
  Classname(const Classname&) = delete;                 \
  void operator=(const Classname&) = delete;

#define DRAKE_COPYABLE_INTERNAL_NO_MOVE(Classname)      \
  Classname(Classname&&) = delete;                      \
  void operator=(Classname&&) = delete;

#define DRAKE_COPYABLE_INTERNAL_DEFAULT_COPY(Classname)         \
  Classname(const Classname&) = default;                        \
  Classname& operator=(const Classname&) = default;             \
  /* Fail at compile-time if default-copy doesn't work. */      \
  static void DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE() {        \
    (void) static_cast<Classname& (Classname::*)(               \
        const Classname&)>(&Classname::operator=);              \
  }

#define DRAKE_COPYABLE_INTERNAL_DEFAULT_MOVE(Classname) \
  Classname(Classname&&) = default;                     \
  Classname& operator=(Classname&&) = default;

// Disable copy, assign, and move.
#define DRAKE_NONCOPYABLE(Classname)                    \
  DRAKE_COPYABLE_INTERNAL_NO_COPY(Classname)            \
  DRAKE_COPYABLE_INTERNAL_NO_MOVE(Classname)

// Default copy-construction and copy-assignment, but (implicitly) disable
// move-construction and move-assignment.  We omit (rather than delete) the
// move functions, because if we declared and deleted them, they would
// participate in overload resolution -- so cases where a move was possible
// would select the move and then fail to compile due to the delete.  By
// leaving the move functions undeclared, overload selection chooses the
// copy functions instead, which matches the mnemonic of this macro.
#define DRAKE_COPYABLE(Classname)                       \
  DRAKE_COPYABLE_INTERNAL_DEFAULT_COPY(Classname)

// Default all copy-construction, copy-assignment, move-construction, and
// move-assignment.
#define DRAKE_COPYMOVEABLE(Classname)                   \
  DRAKE_COPYABLE_INTERNAL_DEFAULT_COPY(Classname)       \
  DRAKE_COPYABLE_INTERNAL_DEFAULT_MOVE(Classname)
