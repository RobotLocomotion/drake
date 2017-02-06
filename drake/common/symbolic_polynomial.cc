#include "drake/common/symbolic_polynomial.h"

using std::ostringstream;
using std::ostream;

namespace drake {
namespace symbolic {
int degree(const symbolic::Expression& e, const Variables& vars) {
  if (!e.is_polynomial()) {
    ostringstream oss;
    e.Display(oss) << "is not a polynomial.";
    throw std::runtime_error(oss.str());
  }
  if (!vars.IsStrictSubsetOf(e.GetVariables())) {
    // Find out the variables in @p vars not included in e.GetVariables();
    const auto& not_included_vars = vars - e.GetVariables();
    ostream oss;
    oss << not_included_vars << "is not a subset of " << e <<".";
    throw std::runtime_error(oss.str());
  }
  int deg = 0;
  if (is_addition(e)) {
    for (const auto& p : get_expr_to_coeff_map_in_addition(e)) {
      int deg_p = 0;
      for (const auto& q : get_base_to_expnt_map_in_multiplication(e)) {
        
      }
    }
  } else if (is_multiplication(e)) {

  } else if (is_variable(e)) {

  } else if (is_constant(e)) {

  }
}
}  //
}  // namespace drake