#include "drake/automotive/maliput/multilane/cubic_polynomial.h"

namespace drake {
namespace maliput {
namespace multilane {

std::ostream& operator<<(std::ostream& out,
                         const CubicPolynomial& cubic_polynomial) {
  return out << "(a: " << std::to_string(cubic_polynomial.a())
             << ", b: " << std::to_string(cubic_polynomial.b())
             << ", c: " << std::to_string(cubic_polynomial.c())
             << ", d: " << std::to_string(cubic_polynomial.d()) << ")";
}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
