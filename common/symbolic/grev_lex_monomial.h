#pragma once
#include "drake/common/symbolic/ordered_monomial.h"

namespace drake {
namespace symbolic {

/**
 * A monomial sorted according to the graded reverse lexicographic ordering. A
 * monomial xᵝ < xᵞ if |β| < |γ|. If |β| = |γ|, then xᵝ < xᵞ if the right most
 * non-zero entry of γ - β is negative.
 *
 * This monomial ordering can always get both the next and previous monomial.
 */
class GrevLexMonomial : public OrderedMonomial {
 public:
  using OrderedMonomial::OrderedMonomial;
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(GrevLexMonomial)
 protected:
  std::unique_ptr<OrderedMonomial> DoGetNextMonomial() const final;
  std::optional<std::unique_ptr<OrderedMonomial>> DoMaybeGetPreviousMonomial()
      const final;
  bool DoLessThanComparison(const OrderedMonomial& m) const final;
};

}  // namespace symbolic
}  // namespace drake