#pragma once
#include <memory>
#include <optional>

#include "drake/common/symbolic/ordered_monomial.h"

namespace drake {
namespace symbolic {

/**
 * A monomial sorted according to the lexicographic ordering. A monomial xᵝ < xᵞ
 * if the left most non-zero entry γ − β > 0.
 *
 * This monomial will fail to get the previous monomial if the leading variable
 * of the monomial has power 1 as we cannot represent this monomial. For
 * example, in the lexicographic order with x < y, we have that, xⁿy² < xy³ for
 * all powers n. Therefore, to represent the previous monomial, we would need n
 * to go to infinity.
 */
class LexMonomial : public OrderedMonomial {
 public:
  using OrderedMonomial::OrderedMonomial;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LexMonomial)
 protected:
  std::unique_ptr<OrderedMonomial> DoGetNextMonomial() const final;

  bool DoLessThanComparison(const OrderedMonomial& m) const final;
};

}  // namespace symbolic
}  // namespace drake
