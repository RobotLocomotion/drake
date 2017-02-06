#include "drake/common/symbolic_polynomial.h"

using std::ostringstream;

namespace drake {
namespace symbolic {
int degree(const symbolic::Expression& e, const Variables& vars) {
  if (!e.is_polynomial()) {
    ostringstream oss;
    Display(oss) << "is not a polynomial.";
    throw std::runtime_error(oss.str());
  }

}
}  //
}  // namespace drake