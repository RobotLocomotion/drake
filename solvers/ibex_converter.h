#pragma once

#include <memory>
#include <unordered_map>

#include <Eigen/Core>
#include <ibex.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {
namespace internal {

// Custom deleter for `ibex::ExprCtr`. It deletes the internal
// `ibex::ExprNode`s while keeping the `ExprSymbol`s intact.
struct ExprCtrDeleter {
  void operator()(const ibex::ExprCtr* const p) const {
    if (p) {
      ibex::cleanup(p->e, false);
      delete p;
    }
  }
};

// Custom deleter for `ibex::ExprNode`. It deletes the internal subnodes while
// keeping the `ExprSymbol`s intact.
struct ExprNodeDeleter {
  void operator()(const ibex::ExprNode* const p) const {
    if (p) {
      ibex::cleanup(*p, false);
    }
  }
};

using UniquePtrToExprNode =
    std::unique_ptr<const ibex::ExprNode, ExprNodeDeleter>;

using UniquePtrToExprCtr = std::unique_ptr<const ibex::ExprCtr, ExprCtrDeleter>;

// Visitor class which converts Drake's symbolic formulas and expressions into
// corresponding `ibex::ExprCtr`s and `ibex::ExprNode`s.
class IbexConverter {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IbexConverter)

  // Constructs a converter from `variables`. The constructed converter can
  // only handle symbolic expressions and formulas which include the passed
  // `variables`. If an expression or a formula includes a variable not
  // appeared in `variables`, Convert methods throw an exception.
  explicit IbexConverter(
      const Eigen::Ref<const VectorX<symbolic::Variable>>& variables);

  // Destructor. It frees all the ibex::symbols created at the time of
  // construction.
  ~IbexConverter();

  // Converts `f` into a corresponding IBEX data structure ibex::ExprCtr*.
  //
  // @throws std::exception if `f` includes a variable which is not included in
  // the converter's variables.
  // @throws std::exception if `f` is of the following unsupported formula
  // types, {True, False, (Boolean) Variable, Non-Equality, Conjuntion,
  // Disjunction, Forall, Isnan, PositiveSemidefinite}.
  UniquePtrToExprCtr Convert(const symbolic::Formula& f);

  // Converts `e` into the corresponding IBEX data structure,
  // `ibex::ExprNode*`.
  //
  // @throws std::exception if `e` includes a variable which is not included in
  // the converter's variables.
  // @throws std::exception if `e` is of the following unsupported expression
  // types, {Ceil, Floor, IfThenElse, UninterpretedFunction}.
  UniquePtrToExprNode Convert(const symbolic::Expression& e);

  // Returns its variables (`ibex::ExprSymbol`s).
  [[nodiscard]] const ibex::Array<const ibex::ExprSymbol>& variables() const;

 private:
  const ibex::ExprNode* Visit(const symbolic::Expression& e);
  const ibex::ExprNode* VisitVariable(const symbolic::Expression& e);
  const ibex::ExprNode* VisitConstant(const symbolic::Expression& e);
  const ibex::ExprNode* VisitAddition(const symbolic::Expression& e);
  const ibex::ExprNode* VisitMultiplication(const symbolic::Expression& e);
  const ibex::ExprNode* VisitDivision(const symbolic::Expression& e);
  const ibex::ExprNode* VisitLog(const symbolic::Expression& e);
  const ibex::ExprNode* VisitAbs(const symbolic::Expression& e);
  const ibex::ExprNode* VisitExp(const symbolic::Expression& e);
  const ibex::ExprNode* VisitSqrt(const symbolic::Expression& e);
  const ibex::ExprNode* ProcessPow(const symbolic::Expression& base,
                                   const symbolic::Expression& exponent);
  const ibex::ExprNode* VisitPow(const symbolic::Expression& e);
  const ibex::ExprNode* VisitSin(const symbolic::Expression& e);
  const ibex::ExprNode* VisitCos(const symbolic::Expression& e);
  const ibex::ExprNode* VisitTan(const symbolic::Expression& e);
  const ibex::ExprNode* VisitAsin(const symbolic::Expression& e);
  const ibex::ExprNode* VisitAcos(const symbolic::Expression& e);
  const ibex::ExprNode* VisitAtan(const symbolic::Expression& e);
  const ibex::ExprNode* VisitAtan2(const symbolic::Expression& e);
  const ibex::ExprNode* VisitSinh(const symbolic::Expression& e);
  const ibex::ExprNode* VisitCosh(const symbolic::Expression& e);
  const ibex::ExprNode* VisitTanh(const symbolic::Expression& e);
  const ibex::ExprNode* VisitMin(const symbolic::Expression& e);
  const ibex::ExprNode* VisitMax(const symbolic::Expression& e);
  const ibex::ExprNode* VisitIfThenElse(const symbolic::Expression&);
  const ibex::ExprNode* VisitCeil(const symbolic::Expression& e);
  const ibex::ExprNode* VisitFloor(const symbolic::Expression& e);
  const ibex::ExprNode* VisitUninterpretedFunction(const symbolic::Expression&);

  const ibex::ExprCtr* Visit(const symbolic::Formula& f, bool polarity);
  const ibex::ExprCtr* VisitFalse(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitTrue(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitVariable(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitEqualTo(const symbolic::Formula& f, bool polarity);
  const ibex::ExprCtr* VisitNotEqualTo(const symbolic::Formula& f,
                                       bool polarity);
  const ibex::ExprCtr* VisitGreaterThan(const symbolic::Formula& f,
                                        bool polarity);
  const ibex::ExprCtr* VisitGreaterThanOrEqualTo(const symbolic::Formula& f,
                                                 bool polarity);
  const ibex::ExprCtr* VisitLessThan(const symbolic::Formula& f, bool polarity);
  const ibex::ExprCtr* VisitLessThanOrEqualTo(const symbolic::Formula& f,
                                              bool polarity);
  const ibex::ExprCtr* VisitConjunction(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitDisjunction(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitNegation(const symbolic::Formula& f, bool polarity);
  const ibex::ExprCtr* VisitForall(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitIsnan(const symbolic::Formula&, bool);
  const ibex::ExprCtr* VisitPositiveSemidefinite(const symbolic::Formula&,
                                                 bool);

  std::unordered_map<symbolic::Variable::Id, const ibex::ExprSymbol*>
      symbolic_var_to_ibex_var_;

  ibex::Array<const ibex::ExprSymbol> var_array_;

  // Represents the value `0.0`. We use this to avoid a possible
  // memory leak caused by IBEX code: See
  // https://github.com/ibex-team/ibex-lib/blob/af48e38847414818913b6954e1b1b3050aa14593/src/symbolic/ibex_ExprCtr.h#L53-L55
  const std::unique_ptr<const ibex::ExprNode> zero_;

  // Makes VisitFormula a friend of this class so that it can use private
  // operator()s.
  friend const ibex::ExprCtr* ::drake::symbolic::VisitFormula<
      const ibex::ExprCtr*>(IbexConverter*, const symbolic::Formula&,
                            const bool&);
  // Makes VisitExpression a friend of this class so that it can use private
  // operator()s.
  friend const ibex::ExprNode* ::drake::symbolic::VisitExpression<
      const ibex::ExprNode*>(IbexConverter*, const symbolic::Expression&);
};

}  // namespace internal
}  // namespace solvers
}  // namespace drake
