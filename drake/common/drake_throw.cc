#include "drake/common/drake_throw.h"

#include <stdexcept>
#include <sstream>

namespace drake {
namespace detail {

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
