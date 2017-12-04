#include <algorithm>
#include <functional>
#include <iterator>
#include <memory>
#include <set>

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"
#include "drake/common/test_utilities/symbolic_test_util.h"

namespace drake {
namespace symbolic {
namespace {

using std::function;
using std::inserter;
using std::make_shared;
using std::set;
using std::shared_ptr;
using std::transform;

using test::FormulaEqual;

// Given formulas = {f₁, ..., fₙ} and a func : Formula → Formula,
// map(formulas, func) returns a set {func(f₁), ... func(fₙ)}.
set<Formula> map(const set<Formula>& formulas,
                 const function<Formula(const Formula&)>& func) {
  set<Formula> result;
  transform(formulas.cbegin(), formulas.cend(),
            inserter(result, result.begin()), func);
  return result;
}

// A class implementing NNF (Negation Normal Form) conversion. See
// https://en.wikipedia.org/wiki/Negation_normal_form for more information on
// NNF.
//
// TODO(soonho-tri): Consider moving this class into a separate file in
// `drake/common` directory when we have a use-case of this class in Drake. For
// now, we have it here to test `drake/common/symbolic_formula_visitor.h`.
class NegationNormalFormConverter {
 public:
  // Converts @p f into an equivalent formula @c f' in NNF.
  Formula Visit(const Formula& f) const { return Visit(f, true); }

 private:
  // Converts @p f into an equivalent formula @c f' in NNF. The parameter @p
  // polarity is to indicate whether it processes @c f (if @p polarity is
  // true) or @c ¬f (if @p polarity is false).
  Formula Visit(const Formula& f, const bool polarity) const {
    return VisitFormula<Formula>(this, f, polarity);
  }
  Formula VisitFalse(const Formula& f, const bool polarity) const {
    // NNF(False)  = False
    // NNF(¬False) = True
    return polarity ? Formula::False() : Formula::True();
  }
  Formula VisitTrue(const Formula& f, const bool polarity) const {
    // NNF(True)  = True
    // NNF(¬True) = False
    return polarity ? Formula::True() : Formula::False();
  }
  Formula VisitVariable(const Formula& f, const bool polarity) const {
    // NNF(b)  = b
    // NNF(¬b) = ¬b
    return polarity ? f : !f;
  }
  Formula VisitEqualTo(const Formula& f, const bool polarity) const {
    // NNF(e1 = e2)    = (e1 = e2)
    // NNF(¬(e1 = e2)) = (e1 != e2)
    return polarity ? f : get_lhs_expression(f) != get_rhs_expression(f);
  }
  Formula VisitNotEqualTo(const Formula& f, const bool polarity) const {
    // NNF(e1 != e2)    = (e1 != e2)
    // NNF(¬(e1 != e2)) = (e1 = e2)
    return polarity ? f : get_lhs_expression(f) == get_rhs_expression(f);
  }
  Formula VisitGreaterThan(const Formula& f, const bool polarity) const {
    // NNF(e1 > e2)    = (e1 > e2)
    // NNF(¬(e1 > e2)) = (e1 <= e2)
    return polarity ? f : get_lhs_expression(f) <= get_rhs_expression(f);
  }
  Formula VisitGreaterThanOrEqualTo(const Formula& f,
                                    const bool polarity) const {
    // NNF(e1 >= e2)    = (e1 >= e2)
    // NNF(¬(e1 >= e2)) = (e1 < e2)
    return polarity ? f : get_lhs_expression(f) < get_rhs_expression(f);
  }
  Formula VisitLessThan(const Formula& f, const bool polarity) const {
    // NNF(e1 < e2)    = (e1 < e2)
    // NNF(¬(e1 < e2)) = (e1 >= e2)
    return polarity ? f : get_lhs_expression(f) >= get_rhs_expression(f);
  }
  Formula VisitLessThanOrEqualTo(const Formula& f, const bool polarity) const {
    // NNF(e1 <= e2)    = (e1 <= e2)
    // NNF(¬(e1 <= e2)) = (e1 > e2)
    return polarity ? f : get_lhs_expression(f) > get_rhs_expression(f);
  }
  Formula VisitConjunction(const Formula& f, const bool polarity) const {
    // NNF(f₁ ∧ ... ∨ fₙ)    = NNF(f₁) ∧ ... ∧ NNF(fₙ)
    // NNF(¬(f₁ ∧ ... ∨ fₙ)) = NNF(¬f₁) ∨ ... ∨ NNF(¬fₙ)
    const set<Formula> new_operands{
        map(get_operands(f), [this, &polarity](const Formula& formula) {
          return this->Visit(formula, polarity);
        })};
    return polarity ? make_conjunction(new_operands)
                    : make_disjunction(new_operands);
  }
  Formula VisitDisjunction(const Formula& f, const bool polarity) const {
    // NNF(f₁ ∨ ... ∨ fₙ)    = NNF(f₁) ∨ ... ∨ NNF(fₙ)
    // NNF(¬(f₁ ∨ ... ∨ fₙ)) = NNF(¬f₁) ∧ ... ∧ NNF(¬fₙ)
    const set<Formula> new_operands{
        map(get_operands(f), [this, &polarity](const Formula& formula) {
          return this->Visit(formula, polarity);
        })};
    return polarity ? make_disjunction(new_operands)
                    : make_conjunction(new_operands);
  }
  Formula VisitNegation(const Formula& f, const bool polarity) const {
    // NNF(¬f, ⊤) = NNF(f, ⊥)
    // NNF(¬f, ⊥) = NNF(f, ⊤)
    return Visit(get_operand(f), !polarity);
  }
  Formula VisitForall(const Formula& f, const bool polarity) const {
    // NNF(∀v₁...vₙ. f)    =  ∀v₁...vₙ. f
    // NNF(¬(∀v₁...vₙ. f)) = ¬∀v₁...vₙ. f
    //
    // TODO(soonho-tri): The second case can be further reduced into
    // ∃v₁...vₙ. NNF(¬f). However, we do not have a representation
    // FormulaExists(∃) yet. Revisit this when we add FormulaExists.
    return polarity ? f : !f;
  }
  Formula VisitIsnan(const Formula& f, const bool polarity) const {
    // NNF(isnan(f))  =  isnan(f)
    // NNF(¬isnan(f)) = ¬isnan(f)
    return polarity ? f : !f;
  }
  Formula VisitPositiveSemidefinite(const Formula& f,
                                    const bool polarity) const {
    // NNF(psd(M))  =  psd(M)
    // NNF(¬psd(M)) = ¬psd(M)
    return polarity ? f : !f;
  }

  // Makes VisitFormula a friend of this class so that it can use private
  // methods.
  friend Formula drake::symbolic::VisitFormula<Formula>(
      const NegationNormalFormConverter*, const Formula&, const bool&);
};

class SymbolicFormulaVisitorTest : public ::testing::Test {
 protected:
  const Variable x_{"x", Variable::Type::CONTINUOUS};
  const Variable y_{"y", Variable::Type::CONTINUOUS};
  const Variable z_{"z", Variable::Type::CONTINUOUS};

  const Variable b1_{"b1", Variable::Type::BOOLEAN};
  const Variable b2_{"b2", Variable::Type::BOOLEAN};
  const Variable b3_{"b3", Variable::Type::BOOLEAN};

  Eigen::Matrix<Expression, 2, 2, Eigen::DontAlign> M_;

  const NegationNormalFormConverter converter_{};

  void SetUp() override {
    // clang-format off
    M_ << (x_ + y_),  -1.0,
               -1.0,   y_;
    // clang-format on
  }
};

TEST_F(SymbolicFormulaVisitorTest, NNFNoChanges) {
  // No Changes: True
  EXPECT_PRED2(FormulaEqual, converter_.Visit(Formula::True()),
               Formula::True());

  // No Changes: False
  EXPECT_PRED2(FormulaEqual, converter_.Visit(Formula::False()),
               Formula::False());

  // No Changes: Variables
  EXPECT_PRED2(FormulaEqual, converter_.Visit(Formula{b1_}), Formula{b1_});
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!b1_), !b1_);

  // No Changes: x ≥ y ∧ y ≤ z
  const Formula f1{x_ >= y_ && y_ <= z_};
  EXPECT_PRED2(FormulaEqual, converter_.Visit(f1), f1);

  // No Changes.: x > y ∨ y < z
  const Formula f2{x_ > y_ || y_ < z_};
  EXPECT_PRED2(FormulaEqual, converter_.Visit(f2), f2);

  // No Changes: isnan(x)
  const Formula f3{isnan(x_)};
  EXPECT_PRED2(FormulaEqual, converter_.Visit(f3), f3);
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!f3), !f3);

  // No Changes: ∀x. x + y ≥ x
  const Formula f4{forall({x_}, x_ + y_ >= x_)};
  EXPECT_PRED2(FormulaEqual, converter_.Visit(f4), f4);
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!f4), !f4);

  // No Changes: psd(M)
  const Formula psd{positive_semidefinite(M_)};
  EXPECT_PRED2(FormulaEqual, converter_.Visit(psd), psd);
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!psd), !psd);
}

TEST_F(SymbolicFormulaVisitorTest, NNFRelational) {
  // ¬(x ≥ y) ∧ ¬(y ≤ z)  ==>  x < y ∧ y > z
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(x_ >= y_) && !(y_ <= z_)),
               x_ < y_ && y_ > z_);

  // ¬(x ≥ y ∧ y ≤ z)  ==>  x < y ∨ y > z
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(x_ >= y_ && y_ <= z_)),
               x_ < y_ || y_ > z_);

  // ¬(x > y) ∨ ¬(y < z)  ==>  x ≤ y ∨ y ≥ z
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(x_ > y_) || !(y_ < z_)),
               x_ <= y_ || y_ >= z_);

  // ¬(x > y ∨ y < z)  ==>  x ≤ y ∧ y ≥ z
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(x_ > y_ || y_ < z_)),
               x_ <= y_ && y_ >= z_);

  // ¬(x ≠ y) ∧ ¬(y = z)  ==>  x = y ∧ y ≠ z
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(x_ != y_) && !(y_ == z_)),
               x_ == y_ && y_ != z_);

  // ¬(x ≠ y ∧ y = z)  ==>  x = y ∨ y ≠ z
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(x_ != y_ && y_ == z_)),
               x_ == y_ || y_ != z_);
}

TEST_F(SymbolicFormulaVisitorTest, NNFBoolean) {
  // ¬(b₁ ∨ ¬(b₂ ∧ b₃))  ==>  ¬b₁ ∧ b₂ ∧ b₃
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(b1_ || !(b2_ && b3_))),
               !b1_ && b2_ && b3_);

  // ¬(b₁ ∨ ¬(b₂ ∨ b₃))  ==>  ¬b₁ ∧ (b₂ ∨ b₃)
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(b1_ || !(b2_ || b3_))),
               !b1_ && (b2_ || b3_));

  // ¬(b₁ ∧ ¬(b₂ ∨ b₃))  ==>  ¬b₁ ∨ b₂ ∨ b₃
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(b1_ && !(b2_ || b3_))),
               !b1_ || b2_ || b3_);

  // ¬(b₁ ∧ ¬(b₂ ∧ b₃))  ==>  ¬b₁ ∨ (b₂ ∧ b₃)
  EXPECT_PRED2(FormulaEqual, converter_.Visit(!(b1_ && !(b2_ && b3_))),
               !b1_ || (b2_ && b3_));
}
}  // namespace
}  // namespace symbolic
}  // namespace drake
