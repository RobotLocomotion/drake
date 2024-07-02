#include "drake/common/symbolic/grev_lex_monomial.h"

#include <map>

namespace drake {
namespace symbolic {

std::unique_ptr<OrderedMonomial> GrevLexMonomial::DoGetNextMonomial() const {
  std::map<Variable, int> powers{};
  if (total_degree_ == 0) {
    // Return the smallest monomial in the lexicographic order. Do this check
    // first to avoid having to iterate over all the variables in the monomial
    // if this has degree 0.
    powers = powers_;
    powers.at(powers_.cbegin()->first) += 1;
    return std::unique_ptr<OrderedMonomial>(new GrevLexMonomial(powers));
  }
  // Find the smallest non-zero power in lex order monomial.
  auto powers_it = powers_.begin();
  while (powers_it->second == 0) {
    powers.emplace_hint(powers.cend(), powers_it->first, 0);
    ++powers_it;
  }
  // Since this is not the constant monomial, there must be a non-zero power.
  DRAKE_ASSERT(powers_it != powers_.end());
  const Variable smallest_non_zero_variable{powers_it->first};
  const int smallest_non_zero_power{powers_it->second};

  // If the smallest non-zero power is the largest variable, then we need to
  // increase the grade. The first monomial at the next grade is the smallest
  // monomial raised to the power of grade + 1.
  if (smallest_non_zero_variable.equal_to(powers_.crbegin()->first)) {
    powers.emplace_hint(powers.end(), smallest_non_zero_variable, 0);
    powers.at(powers_.cbegin()->first) = total_degree_ + 1;
  } else {
    ++powers_it;
    std::map<Variable, int> remaining_powers{};
    while (powers_it != powers_.cend()) {
      remaining_powers.emplace_hint(remaining_powers.cend(), powers_it->first,
                                    powers_it->second);
      ++powers_it;
    }
    const GrevLexMonomial leading_variable_monomial{remaining_powers};
    std::unique_ptr<OrderedMonomial> next_leading_monomial{
        leading_variable_monomial.GetNextMonomial()};
    for (const auto& [v, p] : next_leading_monomial->get_powers()) {
      powers.emplace_hint(powers.cend(), v, p);
    }
    if (next_leading_monomial->total_degree() >
        leading_variable_monomial.total_degree()) {
      powers.emplace_hint(powers.cend(), smallest_non_zero_variable,
                          smallest_non_zero_power - 1);
    } else {
      powers.emplace_hint(powers.cend(), smallest_non_zero_variable,
                          smallest_non_zero_power);
    }
  }
  return std::unique_ptr<OrderedMonomial>(new GrevLexMonomial(powers));
}

bool GrevLexMonomial::DoLessThanComparison(const OrderedMonomial& m) const {
  // We have already checked that this and @p m have the same variables.
  // Moreover, the powers are stored as a map and therefore the variables are
  // already lexicographically ordered.
  if (total_degree_ != m.total_degree()) {
    return total_degree_ < m.total_degree();
  }
  for (const auto& [v, p] : powers_) {
    const int m_power = m.degree(v);
    if (p != m_power) {
      return p > m_power;
    }
  }
  return false;
}

}  // namespace symbolic
}  // namespace drake
