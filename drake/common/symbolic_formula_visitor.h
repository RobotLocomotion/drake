#pragma once

#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"
#include "drake/common/symbolic_formula_cell.h"

namespace drake {
namespace symbolic {

/// Calls visitor object @p v with a symbolic formula @p f, and arguments @p
/// args. Visitor object is expected to implement <tt>Result operator(...)</tt>
/// which takes a shared pointer of a class in FormulaCell's inheritance
/// hierarchy.
///
/// Check the implementation of @c NegationNormalFormConverter class in
/// drake/common/test/symbolic_formula_visitor_test.cc file to find an example.
template <typename Result, typename Visitor, typename... Args>
Result VisitFormula(const Visitor& v, const Formula& f, Args&&... args) {
  switch (f.get_kind()) {
    case FormulaKind::False:
      return v(to_false(f), std::forward<Args>(args)...);
    case FormulaKind::True:
      return v(to_true(f), std::forward<Args>(args)...);
    case FormulaKind::Var:
      return v(to_variable(f), std::forward<Args>(args)...);
    case FormulaKind::Eq:
      return v(to_equal_to(f), std::forward<Args>(args)...);
    case FormulaKind::Neq:
      return v(to_not_equal_to(f), std::forward<Args>(args)...);
    case FormulaKind::Gt:
      return v(to_greater_than(f), std::forward<Args>(args)...);
    case FormulaKind::Geq:
      return v(to_greater_than_or_equal_to(f), std::forward<Args>(args)...);
    case FormulaKind::Lt:
      return v(to_less_than(f), std::forward<Args>(args)...);
    case FormulaKind::Leq:
      return v(to_less_than_or_equal_to(f), std::forward<Args>(args)...);
    case FormulaKind::And:
      return v(to_conjunction(f), std::forward<Args>(args)...);
    case FormulaKind::Or:
      return v(to_disjunction(f), std::forward<Args>(args)...);
    case FormulaKind::Not:
      return v(to_negation(f), std::forward<Args>(args)...);
    case FormulaKind::Forall:
      return v(to_forall(f), std::forward<Args>(args)...);
    case FormulaKind::Isnan:
      return v(to_isnan(f), std::forward<Args>(args)...);
    case FormulaKind::PositiveSemidefinite:
      return v(to_positive_semidefinite(f), std::forward<Args>(args)...);
  }
  // Should not be reachable. But we need the following to avoid "control
  // reaches end of non-void function" gcc-warning.
  DRAKE_ABORT();
}

}  // namespace symbolic
}  // namespace drake
