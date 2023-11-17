#include "drake/common/symbolic/lex_monomial.h"

#include <iostream>
namespace drake {
namespace symbolic {

std::unique_ptr<OrderedMonomial> LexMonomial::DoGetNextMonomial() const {
  std::map<Variable, int> powers{powers_};
  powers.rbegin()->second += 1;
  return std::unique_ptr<OrderedMonomial>(new LexMonomial(powers));
};

std::optional<std::unique_ptr<OrderedMonomial>>
LexMonomial::DoMaybeGetPreviousMonomial() const {
  std::map<Variable, int> powers{powers_};
  for (auto it = powers_.rbegin(); it != powers_.rend(); ++it) {
    if (it->second > 0) {
      // When the largest non-zero power variable in the monomial is 1, then we
      // require an infinite degree monomial to represent the previous monomial.
      // The only except is if we are at the smallest lexicographic monomial, in
      // which case the previous monomial is the 1 monomial.
      if (it->second == 1 && next(it) != powers_.rend()) {
        return std::nullopt;
      } else {
        powers.at(it->first) -= 1;
      }
    }
  }
  return std::unique_ptr<OrderedMonomial>(new LexMonomial(powers));
};

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
};

}  // namespace symbolic
}  // namespace drake