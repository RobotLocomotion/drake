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

void Abort(const char* condition, const char* func, const char* file,
           int line) {
  std::cerr << "abort: failure at " << file << ":" << line
            << " in " << func << "()";
  if (condition) {
    std::cerr << ": assertion '" << condition << "' failed.";
  } else {
    std::cerr << ".";
  }
  std::cerr << std::endl;

  std::abort();
}

void Throw(const char* condition, const char* func, const char* file,
           int line) {
  std::stringstream message;
  message
      << "Failure at " << file << ":" << line << " in " << func << "(): "
      << "condition '" << condition << "' failed.";
  throw std::runtime_error(message.str());
}

}  // namespace detail
}  // namespace drake
