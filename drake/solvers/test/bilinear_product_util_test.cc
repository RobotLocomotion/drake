#include "drake/solvers/bilinear_product_util.h"

#include <gtest/gtest.h>
#include "drake/common/test/symbolic_test_util.h"

using drake::symbolic::Variable;
using drake::symbolic::test::ExprEqual;

namespace drake {
namespace solvers {
namespace {
class BilinearProductTest : public ::testing::Test {
 public:
  BilinearProductTest()
      : x_{Variable{"x0"}, Variable{"x1"}, Variable{"x2"}},
        y_{Variable{"y0"}, Variable{"y1"}, Variable{"y2"}, Variable{"y3"}},
        z_{Variable{"z0"}, Variable{"z1"}} {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 4; ++j) {
        xy_(i, j) = symbolic::Variable("xy(" + std::to_string(i) + "," +
                                       std::to_string(j) + ")");
      }
    }

    for (int i = 0; i < 3; ++i) {
      for (int j = i; j < 3; ++j) {
        xx_(i, j) = symbolic::Variable("xx(" + std::to_string(i) + "," +
                                       std::to_string(j) + ")");
      }
    }
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < i; ++j) {
        xx_(i, j) = xx_(j, i);
      }
    }
  }

 protected:
  VectorDecisionVariable<3> x_;
  VectorDecisionVariable<4> y_;
  VectorDecisionVariable<2> z_;
  MatrixDecisionVariable<3, 4> xy_;
  MatrixDecisionVariable<3, 3> xx_;
};

TEST_F(BilinearProductTest, ConstantTerm) {
  const std::vector<symbolic::Expression> expressions{
      0, 1, z_(0) + z_(1), pow(z_(0), 3) + z_(0) * z_(1)};
  for (const auto& ei : expressions) {
    EXPECT_PRED2(ExprEqual, ei, ReplaceBilinearTerms(ei, x_, y_, xy_));
    EXPECT_PRED2(ExprEqual, ei, ReplaceBilinearTerms(ei, x_, x_, xx_));
  }
}

TEST_F(BilinearProductTest, LinearTerm) {
  const std::vector<symbolic::Expression> expressions{
      x_(0), y_(0), x_(0) + y_(1) + z_(0) + 2,
      2 * x_(0) + 3 * y_(1) * z_(1) + 3,
      2 * x_(0) + 3 * y_(1) * z_(0) + pow(z_(1), 3)};
  for (const auto& ei : expressions) {
    EXPECT_PRED2(ExprEqual, ei, ReplaceBilinearTerms(ei, x_, y_, xy_));
    EXPECT_PRED2(ExprEqual, ei, ReplaceBilinearTerms(ei, x_, x_, xx_));
  }
}

TEST_F(BilinearProductTest, QuadraticTerm0) {
  symbolic::Expression e{x_(0) * x_(0)};
  symbolic::Expression e_expected{xx_(0, 0)};
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = x_(0) * x_(0) + 2 * x_(1) * x_(1);
  e_expected = xx_(0, 0) + 2 * xx_(1, 1);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = 2 * x_(0) * x_(0) * y_(0) * y_(1);
  e_expected = 2 * xx_(0, 0) * y_(0) * y_(1);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = 2 * x_(2) * x_(2) * y_(0) * y_(1) + 3 * y_(1) * x_(2) + 2;
  e_expected = 2 * xx_(2, 2) * y_(0) * y_(1) + 3 * y_(1) * x_(2) + 2;
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = (2 + y_(0) + y_(0) * y_(1)) * x_(0) * x_(0);
  e_expected = (2 + y_(0) + y_(0) * y_(1)) * xx_(0, 0);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));
}

TEST_F(BilinearProductTest, QuadraticTerm1) {
  symbolic::Expression e{x_(0) * x_(1)};
  symbolic::Expression e_expected{xx_(0, 1)};
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = 2 * x_(0) * x_(1);
  e_expected = 2 * xx_(0, 1);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = 2 * y_(0) * y_(1) * x_(1) * x_(2);
  e_expected = 2 * y_(0) * y_(1) * xx_(1, 2);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = 2 * pow(y_(0), 3) * x_(1) * x_(2) + y_(1) * x_(1) * x_(1) + 2 * x_(0) +
      3 + 3 * x_(1);
  e_expected = 2 * pow(y_(0), 3) * xx_(1, 2) + y_(1) * xx_(1, 1) + 2 * x_(0) +
               3 + 3 * x_(1);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = (2 + y_(0) + y_(0) * y_(1)) * x_(0) * x_(1);
  e_expected = (2 + y_(0) + y_(0) * y_(1)) * xx_(0, 1);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = 2 * x_(0) * x_(0) + xx_(0, 0);
  e_expected = 3 * xx_(0, 0);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));

  e = xx_(0, 0) * x_(0) * x_(0);
  e_expected = xx_(0, 0) * xx_(0, 0);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, x_, xx_));
}

TEST_F(BilinearProductTest, QuadraticTerm2) {
  symbolic::Expression e{x_(0) * y_(0)};
  symbolic::Expression e_expected{xy_(0, 0)};
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, y_, xy_));

  e = 4 * x_(0) * y_(1);
  e_expected = 4 * xy_(0, 1);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, y_, xy_));

  e = 2 * z_(0) * z_(1) * z_(1) * x_(0) * y_(2);
  e_expected = 2 * z_(0) * z_(1) * z_(1) * xy_(0, 2);
  EXPECT_PRED2(ExprEqual, e_expected, ReplaceBilinearTerms(e, x_, y_, xy_));

  e = 2 * pow(z_(1), 3) * x_(0) * y_(2) + z_(0) * x_(1) * y_(1) +
      (4 * z_(0) + z_(1)) * x_(2) * y_(3) + 3 * x_(2) + 4;
  e_expected = 2 * pow(z_(1), 3) * xy_(0, 2) + z_(0) * xy_(1, 1) +
               (4 * z_(0) + z_(1)) * xy_(2, 3) + 3 * x_(2) + 4;
  EXPECT_PRED2(ExprEqual, e_expected.Expand(),
               ReplaceBilinearTerms(e, x_, y_, xy_).Expand());
}

TEST_F(BilinearProductTest, HigherOrderTest) {
  EXPECT_THROW(ReplaceBilinearTerms(pow(x_(0), 3), x_, x_, xx_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * x_(0) * x_(1), x_, x_, xx_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * x_(1) * x_(2), x_, x_, xx_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * x_(0), x_, y_, xy_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * x_(1), x_, y_, xy_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * x_(1) * x_(1), x_, y_, xy_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * y_(1) * y_(2), x_, y_, xy_),
               std::runtime_error);

  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * y_(2) * y_(2), x_, y_, xy_),
               std::runtime_error);
}

TEST_F(BilinearProductTest, WIncludesXY) {
  const VectorDecisionVariable<2> x{x_(0), x_(1)};
  const VectorDecisionVariable<2> y{y_(0), y_(1)};
  MatrixDecisionVariable<2, 2> W;
  W << x(0), x(1), y(0), y(1);  // W contains entries in x or y.
  const symbolic::Expression e{x(0) * y(0) + W(0, 0)};
  EXPECT_PRED2(ExprEqual, ReplaceBilinearTerms(e, x, y, W), 2 * x(0));
}

TEST_F(BilinearProductTest, DuplicateEntry) {
  // x_duplicate contains duplicate entries.
  VectorDecisionVariable<2> x_duplicate(x_(0), x_(0));
  EXPECT_THROW(ReplaceBilinearTerms(x_(0) * x_(0), x_duplicate, x_duplicate,
                                    xx_.block<2, 2>(0, 0)),
               std::runtime_error);
}
}  // namespace
}  // namespace solvers
}  // namespace drake
