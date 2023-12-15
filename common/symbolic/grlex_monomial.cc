#include "drake/common/symbolic/grlex_monomial.h"

#include <iostream>
#include <map>
namespace drake {
namespace symbolic {

std::unique_ptr<OrderedMonomial> GrLexMonomial::DoGetNextMonomial() const {
  std::map<Variable, int> powers{powers_};
  if (powers_.crbegin()->second == total_degree_) {
    // If the largest monomial has degree equal to the grade, we need to
    // increase the grade. The first monomial at the next grade is the smallest
    // monomial with power = grade + 1.
    if (!powers.empty()) {
      powers.at(powers_.crbegin()->first) = 0;
      powers.at(powers_.cbegin()->first) = total_degree_ + 1;
    }
  } else {
    // Find the largest non-zero power in lex order monomial.
    auto powers_it = powers.rbegin();
    while (powers_it->second == 0) {
      ++powers_it;
    }
    // Since this is not the constant monomial, there must be a non-zero power.
    DRAKE_ASSERT(powers_it != powers.crend());

    if (powers_it->second == total_degree_) {
      // If the largest non-zero power is equal to the grade, then increment the
      // next variable's power by one, reset the rest of the variables to zero,
      // and set the smallest monomials power to grade - 1. Notice that there
      // must be a "next variable", since we cannot be at the largest variable
      // due to checking that first.
      powers.at(std::prev(powers_it)->first) += 1;
      powers.at(powers_it->first) = 0;
      powers.at(powers.cbegin()->first) = total_degree_ - 1;
    } else {
      // Otherwise form the trailing powers monomial. Let i be the index of the
      // largest, non-zero power lex order of the current monomial. The trailing
      // power monomial has the same variables as the current monomial, except
      // that the power αⱼ is 0 if j ≤ i and αⱼ = αᵢ if j > i.
      const int largest_monomial_power = powers_it->second;
      powers.at(powers_it->first) = 0;
      GrLexMonomial trailing_monomial{powers};
      std::unique_ptr<OrderedMonomial> next_trailing_monomial =
          trailing_monomial.GetNextMonomial();
      powers.at(powers_it->first) =
          largest_monomial_power +
          next_trailing_monomial->degree(powers_it->first);
      const bool grade_increased{powers.at(powers_it->first) > total_degree_};
      for (++powers_it; powers_it != powers.crend(); ++powers_it) {
        // If adding the powers of the next trailing monomial would increase the
        // grade, then zero out the remaining powers. Otherwise, the next
        // monomial has the same trailing powers as the next trailing monomials
        // powers.
        powers.at(powers_it->first) =
            grade_increased ? 0
                            : next_trailing_monomial->degree(powers_it->first);
      }
    }
  }

  return std::unique_ptr<OrderedMonomial>(new GrLexMonomial(powers));
}

std::optional<std::unique_ptr<OrderedMonomial>>
GrLexMonomial::DoMaybeGetPreviousMonomial() const {
  // We only need to handle the case when the total degree is more than 0.
  std::map<Variable, int> powers{powers_};
  if (powers.cbegin()->second == total_degree_) {
    // If the smallest monomial has degree equal to the grade, we need to
    // decrease the grade. The first monomial at the next grade is the largest
    // monomial with power = grade - 1.
    if (!powers.empty()) {
      powers.at(powers_.crbegin()->first) = total_degree_ - 1;
      powers.at(powers_.cbegin()->first) = 0;
    }
  } else {
    // Find the largest non-zero power in lex order monomial
    auto powers_it = powers.rbegin();
    while (powers_it->second == 0) {
      ++powers_it;
    }
    DRAKE_ASSERT(powers_it != powers.crend());
    const int largest_monomial_power = powers_it->second;

    // Form the trailing powers monomial. Let i be the index of the
    // largest, non-zero power lex order of the current monomial. The trailing
    // power monomial has the same variables as the current monomial, except
    // that the power αⱼ is 0 if j ≤ i and αⱼ = αᵢ if j > i.
    powers.at(powers_it->first) = 0;
    GrLexMonomial trailing_monomial{powers};
    std::optional<std::unique_ptr<OrderedMonomial>> prev_trailing_monomial =
        trailing_monomial.MaybeGetPreviousMonomial();
    if (!prev_trailing_monomial.has_value()) {
      powers.at(powers_it->first) = largest_monomial_power - 1;
      powers.at((++powers_it)->first) += 1;
      for (++powers_it; powers_it != powers.crend(); ++powers_it) {
        powers.at(powers_it->first) = 0;
      }
    } else if (prev_trailing_monomial->get()->total_degree() ==
               trailing_monomial.total_degree()) {
      powers.at(powers_it->first) = largest_monomial_power;
      for (++powers_it; powers_it != powers.crend(); ++powers_it) {
        powers.at(powers_it->first) =
            prev_trailing_monomial->get()->degree(powers_it->first);
      }
    } else {
      powers.at(powers_it->first) = largest_monomial_power - 1;
      powers.at((++powers_it) -> first) = 1 + trailing_monomial.total_degree();
//      if (powers.at(powers_it->first) == 0) {
//        powers.at((++powers_it)->first) = total_degree_;
//        for (++powers_it; powers_it != powers.crend(); ++powers_it) {
//          powers.at(powers_it->first) = 0;
//        }
//      } else {
//        powers.at((++powers_it)->first) += 1;
//        for (++powers_it; powers_it != powers.crend(); ++powers_it) {
//          powers.at(powers_it->first) =
//              prev_trailing_monomial->get()->degree(powers_it->first);
//        }
//      }
    }
  }
  return std::unique_ptr<OrderedMonomial>(new GrLexMonomial(powers));
}

bool GrLexMonomial::DoLessThanComparison(const OrderedMonomial& m) const {
  // We have already checked that this and @p m have the same variables.
  // Moreover, the powers are stored as a map and therefore the variables are
  // already lexicographically ordered.
  if (total_degree_ != m.total_degree()) {
    return total_degree_ < m.total_degree();
  }
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
