#include "drake/common/symbolic/grlex_monomial.h"

#include <iostream>
#include <map>
namespace drake {
namespace symbolic {
//
// std::unique_ptr<OrderedMonomial> GrLexMonomial::DoGetNextMonomial() const {
//  std::map<Variable, int> powers{};
//  if (total_degree_ == 0) {
//    // Return the smallest monomial in the lexicographic order. Do this check
//    // first to avoid having to iterate over all the variables in the monomial
//    // if this has degree 0.
//    powers = powers_;
//    if (!powers.empty()) {
//      powers.at(powers.cbegin()->first) += 1;
//    }
//    return std::unique_ptr<OrderedMonomial>(new GrLexMonomial(powers));
//  }
//  // Find the largest non-zero power in lex order monomial.
//  auto powers_it = powers_.rbegin();
//  while (powers_it->second == 0) {
//    powers.emplace_hint(powers.cend(), powers_it->first, 0);
//    ++powers_it;
//  }
//  // Since this is not the constant monomial, there must be a non-zero power.
//  DRAKE_ASSERT(powers_it != powers_.crend());
//  const Variable largest_non_zero_variable{powers_it->first};
//  const int largest_non_zero_power{powers_it->second};
//
//  std::cout << "largest variable is " << largest_non_zero_variable <<
//  std::endl;
//
//  if (largest_non_zero_variable.equal_to(powers_.crbegin()->first) &&
//      largest_non_zero_power == total_degree_) {
//    // If the largest monomial has total degree equal to the grade, we need to
//    // increase the grade. The first variable at the next grade is the
//    smallest
//    // variable raise to the grade + 1.
//    while (!powers_it->first.equal_to(
//        powers_.cbegin()->first /* the smallest monomial */)) {
//      powers.emplace_hint(powers.cbegin(), powers_it->first, 0);
//      ++powers_it;
//    }
//    // We are now at the smallest monomial.
//    powers.emplace_hint(powers.cbegin(), powers_it->first, total_degree_ + 1);
//  } else if (largest_non_zero_variable.equal_to(powers_.cbegin()->first)) {
//    // We are at the smallest monomial. So we increment the second smallest
//    // monomial and decrement the smallest monomial.
//    powers.at(prev(powers_it)->first) += 1;
//    powers.emplace_hint(powers.cbegin(), powers_it->first,
//                        powers_it->second - 1);
//
//  } else {
//    ++powers_it;
//    auto trailing_powers_it = powers_it;
//    std::map<Variable, int> trailing_powers{};
//    while (trailing_powers_it != powers_.crend()) {
//      trailing_powers.emplace(trailing_powers_it->first,
//                                   trailing_powers_it->second);
//      ++trailing_powers_it;
//    }
//    // There must be monomials in the trailing powers since we have handled
//    the
//    // case of no trailing powers in the second clause of this if statement.
//    DRAKE_ASSERT(trailing_powers.size() > 0);
//    const GrLexMonomial trailing_variables_monomial{trailing_powers};
//    std::unique_ptr<OrderedMonomial> next_trailing_monomial{
//        trailing_variables_monomial.GetNextMonomial()};
//
//
//    if (next_trailing_monomial->total_degree() >
//        trailing_variables_monomial.total_degree()) {
//      // The next trailing monomial increased the grade. This is a sign that
//      we
//      // actually need to increase the power of the largest_non_zero_variable
//      // and reset the trailing variables to be all zero until the last
//      // monomial.
//      powers.emplace_hint(powers.cbegin(), largest_non_zero_variable,
//                          largest_non_zero_power + 1);
//      while (powers_it != powers_.crend()) {
//        if (powers_it->first.equal_to(powers_.cbegin()->first)) {
//          powers.emplace_hint(powers.cbegin(), powers_it->first, 1);
//        } else {
//          powers.emplace_hint(powers.cbegin(), powers_it->first, 0);
//        }
//        ++powers_it;
//      }
//    } else {
//      powers.emplace_hint(powers.cbegin(), largest_non_zero_variable,
//                          largest_non_zero_power);
//      // Otherwise we set the rest of the monomial to the next trailing
//      // monomial.
//      while (powers_it != powers_.crend()) {
//        powers.emplace_hint(powers.cbegin(), powers_it->first,
//                            next_trailing_monomial->degree(powers_it->first));
//        ++powers_it;
//      }
//    }
//  }
//  for (const auto& [v, p] : powers) {
//    std::cout << fmt::format("powers before construction {} : {}", v, p)
//              << std::endl;
//  }
//  return std::unique_ptr<OrderedMonomial>(new GrLexMonomial(powers));
//}

std::unique_ptr<OrderedMonomial> GrLexMonomial::DoGetNextMonomial() const {
  std::map<Variable, int> powers{};
  if (powers_.crbegin()->second == total_degree_) {
    // If the largest monomial has degree equal to the grade, we need to
    // increase the grade. The first variable at the next grade is the
    for(const auto& [v, _]: powers_) {
      powers.emplace_hint(powers.cend(), v, 0);
    }
    if (!powers.empty()) {
      powers.at(powers.cbegin()->first) = total_degree_ + 1;
    }
    return std::unique_ptr<OrderedMonomial>(new GrLexMonomial(powers));
  } else {
    // Find the largest non-zero power in lex order monomial.
    auto powers_it = powers_.rbegin();
    while (powers_it->second == 0) {
      powers.emplace_hint(powers.cend(), powers_it->first, 0);
      ++powers_it;
    }
    // Since this is not the constant monomial, there must be a non-zero power.
    DRAKE_ASSERT(powers_it != powers_.crend());
    auto trailing_powers_it = powers_it;
    ++trailing_powers_it;
    std::map<Variable, int> trailing_powers{};
    while (trailing_powers_it != powers_.crend()) {
      trailing_powers.emplace(trailing_powers_it->first,
                              trailing_powers_it->second);
      ++trailing_powers_it;
    }
    const GrLexMonomial trailing_variables_monomial{trailing_powers};
    std::unique_ptr<OrderedMonomial> next_trailing_monomial;

    if (trailing_powers.size() == 0) {
      next_trailing_monomial =
          std::unique_ptr<OrderedMonomial>(new GrLexMonomial());
    } else {
      next_trailing_monomial = trailing_variables_monomial.GetNextMonomial();
    }
    if (next_trailing_monomial->total_degree() == 0 ||
        next_trailing_monomial->total_degree() >
            trailing_variables_monomial.total_degree()) {
      powers.at(prev(powers_it)->first) += 1;
      while (powers_it != prev(powers_.crend())) {
        //        powers.emplace_hint(powers.cend(), powers_it->first, 0);
        powers.emplace(powers_it->first, 0);
        ++powers_it;
      }
      //      powers.emplace_hint(powers.cend(), powers_it->first, total_degree_
      //      - 1);
      powers.emplace(powers_it->first, total_degree_ - 1);
    } else {
      powers.emplace_hint(powers.cend(), powers_it->first, powers_it->second);
      for (const auto& [v, p] : next_trailing_monomial->get_powers()) {
        //        powers.emplace_hint(powers.cend(), v, p);
        powers.emplace(v, p);
      }
    }
  }

  return std::unique_ptr<OrderedMonomial>(new GrLexMonomial(powers));
}

std::optional<std::unique_ptr<OrderedMonomial>>
GrLexMonomial::DoMaybeGetPreviousMonomial() const {
  return std::nullopt;
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
