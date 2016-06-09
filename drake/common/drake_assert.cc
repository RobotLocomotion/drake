#include "drake/common/drake_assert.h"

#include <cstdlib>
#include <iostream>

namespace drake {

void Fail(const char* condition, const char* func, const char* file, int line) {
  std::cerr << "error: Fail at " << file << ":" << line << " in " << func;
  if (condition) {
    std::cerr << ": assertion '" << condition << "' failed.";
  } else {
    std::cerr << ".";
  }
  std::cerr << std::endl;

  std::abort();
}

}  // namespace drake
