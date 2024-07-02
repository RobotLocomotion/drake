#include "drake/common/symbolic/lex_monomial.h"

#include <map>

namespace drake {
namespace symbolic {

std::unique_ptr<OrderedMonomial> LexMonomial::DoGetNextMonomial() const {
  std::map<Variable, int> powers{powers_};
  powers.rbegin()->second += 1;
  return std::unique_ptr<OrderedMonomial>(new LexMonomial(powers));
}

bool LexMonomial::DoLessThanComparison(const OrderedMonomial& m) const {
  // We have already checked that this and @p m have the same variables.
  // Moreover, the powers are stored as a map and therefore the variables are
  // already lexicographically ordered.
  for (auto it = powers_.rbegin(); it != powers_.rend(); ++it) {
    const int p_m = m.degree(it->first);
    if (p_m != it->second) {
      return it->second < p_m;
    }
  }
  return false;
}

}  // namespace symbolic
}  // namespace drake
