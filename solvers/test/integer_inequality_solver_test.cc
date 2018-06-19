/* clang-format off to disable clang-format-includes */
/* clang-format on */

#include "drake/solvers/integer_inequality_solver.h"

#include <gtest/gtest.h>
namespace drake {
namespace solvers {
namespace {

struct VectorLexCompare {
  bool operator()(Eigen::VectorXi v, Eigen::VectorXi w) const {
    for (int i = 0; i < v.size(); ++i) {
      if (v(i) < w(i)) return true;
      if (v(i) > w(i)) return false;
    }
    return false;
  }
};

typedef std::set<Eigen::VectorXi, VectorLexCompare> IntegerSet;

IntegerSet MatrixToSet(Eigen::MatrixXi M) {
  IntegerSet M_set;
  for (int i = 0; i < M.rows(); i++) {
    M_set.insert(M.row(i));
  }
  return M_set;
}

class IntegerLatticeTest : public ::testing::Test {
 public:
  void CheckEnumeration(IntegerSet ref) {
    auto y = EnumerateIntegerSolutions(A_, b_, lower_bound_, upper_bound_);
    auto y_set = MatrixToSet(y);
    EXPECT_EQ(y_set, ref);
  }
  void SetDimensions(int num_ineq, int num_var) {
    int m = num_ineq;
    int n = num_var;
    A_.resize(m, n);
    b_.resize(m);
    lower_bound_.resize(n);
    upper_bound_.resize(n);
  }

 protected:
  Eigen::MatrixXi A_;
  Eigen::VectorXi b_;
  Eigen::VectorXi lower_bound_;
  Eigen::VectorXi upper_bound_;
};

TEST_F(IntegerLatticeTest, EqualComponents) {
  SetDimensions(2, 2);
  A_ << 1, -1, -1, 1;

  b_ << 0, 0;

  lower_bound_ << 0, 0;
  upper_bound_ << 2, 2;
  IntegerSet ref;

  ref.insert(Eigen::VectorXi::Constant(2, 0));
  ref.insert(Eigen::VectorXi::Constant(2, 1));
  ref.insert(Eigen::VectorXi::Constant(2, 2));
  CheckEnumeration(ref);
}

TEST_F(IntegerLatticeTest, SumToConstant) {
  SetDimensions(2, 2);
  A_ << 1, 1, -1, -1;
  b_ << 2, -2;

  lower_bound_ << 0, 0;
  upper_bound_ << 2, 2;
  Eigen::MatrixXi ref_mat(3, 2);
  ref_mat << 0, 2, 2, 0, 1, 1;

  CheckEnumeration(MatrixToSet(ref_mat));
}

TEST_F(IntegerLatticeTest, Empty) {
  SetDimensions(1, 2);
  A_ << 1, 0;
  b_ << 0;

  lower_bound_ << 1, 0;
  upper_bound_ << 2, 0;
  IntegerSet ref;

  CheckEnumeration(ref);
}

}  // namespace
}  // namespace solvers
}  // namespace drake
