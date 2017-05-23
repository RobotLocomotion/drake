// This file contains the implementation of both drake_assert and drake_throw.
// clang-format off
#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"
// clang-format on

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace drake {
namespace detail {
namespace {

// Stream into @p out the given failure details; only @p condition may be null.
void PrintFailureDetailTo(std::ostream& out, const char* condition,
                          const char* func, const char* file, int line) {
  out << "Failure at " << file << ":" << line << " in " << func << "()";
  if (condition) {
    out << ": condition '" << condition << "' failed.";
  } else {
    out << ".";
  }
}
}  // namespace

// Declared in drake_assert.h.
void Abort(const char* condition, const char* func, const char* file,
           int line) {
  std::cerr << "abort: ";
  PrintFailureDetailTo(std::cerr, condition, func, file, line);
  std::cerr << std::endl;
  std::abort();
}

// Declared in drake_throw.h.
void Throw(const char* condition, const char* func, const char* file,
           int line) {
  std::ostringstream what;
  PrintFailureDetailTo(what, condition, func, file, line);
  throw std::runtime_error(what.str());
}

}  // namespace detail
}  // namespace drake
