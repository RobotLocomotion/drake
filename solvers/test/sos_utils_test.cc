/* clang-format off to disable clang-format-includes */
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/sos_utils.h"
#include "drake/common/symbolic_monomial_util.h"
/* clang-format on */

#include <gtest/gtest.h>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {
namespace {

using drake::symbolic::Expression;
using drake::symbolic::Variable;
using drake::symbolic::Variables;
using drake::symbolic::Monomial;

typedef std::set<Monomial,
                drake::symbolic::GradedReverseLexOrder<std::less<Variable>>>
                MonomialSet;

MonomialSet VectorToSet(const drake::VectorX<Monomial> & x) {
  MonomialSet x_set;
  for (int i = 0; i < x.size(); i++) {
    x_set.insert(x(i));
  }
  return x_set;
}

class SosUtilsTest : public ::testing::Test {
 public:
  void CheckMonomialBasis(symbolic::Polynomial poly, MonomialSet basis_ref) {
    drake::VectorX<Monomial> basis = ConstructMonomialBasis(poly);
    MonomialSet basis_set = VectorToSet(basis);
    EXPECT_EQ(basis_set, basis_ref);
  }

 protected:
  void SetUp() override {
    x_ = prog_.NewIndeterminates<3>();
  }

  MathematicalProgram prog_;
  VectorIndeterminate<3> x_;
};

TEST_F(SosUtilsTest, MotzkinPoly) {
  const symbolic::Polynomial poly{
    pow(x_(0), 2)*pow(x_(1), 4)  +
    pow(x_(0), 4)*pow(x_(1), 2)  +
    pow(x_(0), 2)*pow(x_(1), 2)  +
    + 1
  };
  MonomialSet basis_ref;
  basis_ref.insert(Monomial());
  basis_ref.insert(Monomial(pow(x_(0), 1)*pow(x_(1), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 2)*pow(x_(1), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 1)*pow(x_(1), 2)));
  CheckMonomialBasis(poly, basis_ref);
}

TEST_F(SosUtilsTest, Univariate) {
  const symbolic::Polynomial poly{
    1 + pow(x_(0), 1) + pow(x_(0), 2) + pow(x_(0), 8)};
  MonomialSet basis_ref;
  basis_ref.insert(Monomial());
  basis_ref.insert(Monomial(pow(x_(0), 1)));
  basis_ref.insert(Monomial(pow(x_(0), 2)));
  basis_ref.insert(Monomial(pow(x_(0), 3)));
  basis_ref.insert(Monomial(pow(x_(0), 4)));
  CheckMonomialBasis(poly, basis_ref);
}


TEST_F(SosUtilsTest, Empty) {
  const symbolic::Polynomial poly{ pow(x_(0), 3) };
  MonomialSet basis_ref;
  CheckMonomialBasis(poly, basis_ref);
}

TEST_F(SosUtilsTest, Singleton) {
  const symbolic::Polynomial poly{1 + pow(x_(0), 1) };
  MonomialSet basis_ref;
  basis_ref.insert(Monomial());
  CheckMonomialBasis(poly, basis_ref);
}

}  // namespace
}  // namespace solvers
}  // namespace drake



