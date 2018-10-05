#pragma once

#include <functional>
#include <utility>

#include "drake/common/symbolic.h"

namespace drake {
namespace symbolic {

/// A pattern is an expression which possibly includes variables which represent
/// placeholders. It is used to construct a `RewritingRule`.
using Pattern = Expression;

/// A `RewritingRule`, `lhs => rhs`, consists of two Patterns `lhs` and `rhs`. A
/// rewriting rule instructs a rewriter how to transform a given expression `e`.
/// First, the rewriter tries to find a match between the expression `e` and the
/// pattern `lhs`. If such a match is found, it applies the match result
/// (substitution) to `rhs`. Otherwise, the same expression `e` is returned.
class RewritingRule {
 public:
  /// Constructs a rewriting rule `lhs => rhs`.
  RewritingRule(Pattern lhs, Pattern rhs)
      : lhs_{std::move(lhs)}, rhs_{std::move(rhs)} {}

  /// Default copy constructor.
  RewritingRule(const RewritingRule&) = default;

  /// Default move constructor.
  RewritingRule(RewritingRule&&) = default;

  /// Deleted copy-assign operator.
  RewritingRule& operator=(const RewritingRule&) = delete;

  /// Deleted move-assign operator.
  RewritingRule& operator=(RewritingRule&&) = delete;

  /// Default destructor.
  ~RewritingRule() = default;

  /// Returns the const reference of the LHS of the rewriting rule.
  const Pattern& lhs() const { return lhs_; }
  /// Returns the const reference of the RHS of the rewriting rule.
  const Pattern& rhs() const { return rhs_; }

 private:
  const Pattern lhs_;
  const Pattern rhs_;
};

/// A `Rewriter` is a function from an Expression to an Expression.
using Rewriter = std::function<Expression(const Expression&)>;

/// Constructs a rewriter based on a rewriting rule @p r.
Rewriter MakeRuleRewriter(const RewritingRule& r);

}  // namespace symbolic
}  // namespace drake
