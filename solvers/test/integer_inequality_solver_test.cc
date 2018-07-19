#include "drake/solvers/integer_inequality_solver.h"

#include <gtest/gtest.h>
namespace drake {
namespace solvers {
namespace {

// Compare in Lexicographical (Lex) order
struct VectorLexCompare {
  bool operator()(const Eigen::VectorXi& v, const Eigen::VectorXi& w) const {
    for (int i = 0; i < v.size(); ++i) {
      if (v(i) < w(i)) return true;
      if (v(i) > w(i)) return false;
    }
    return false;
  }
};

typedef std::set<Eigen::VectorXi, VectorLexCompare> IntegerSet;

IntegerSet MatrixToSet(const Eigen::MatrixXi& M) {
  IntegerSet M_set;
  for (int i = 0; i < M.rows(); i++) {
    M_set.insert(M.row(i));
  }
  return M_set;
}

class IntegerLatticeTest : public ::testing::Test {
 public:
  IntegerSet EnumerationSolutions() {
    const auto y = EnumerateIntegerSolutions(A_, b_,
                                             lower_bound_, upper_bound_);
    return MatrixToSet(y);
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
  EXPECT_EQ(EnumerationSolutions(), ref);
}

TEST_F(IntegerLatticeTest, SumToConstant) {
  SetDimensions(2, 2);
  A_ << 1, 1, -1, -1;
  b_ << 2, -2;

  lower_bound_ << 0, 0;
  upper_bound_ << 2, 2;
  Eigen::MatrixXi ref_mat(3, 2);
  ref_mat << 0, 2, 2, 0, 1, 1;

  EXPECT_EQ(EnumerationSolutions(), MatrixToSet(ref_mat));
}

TEST_F(IntegerLatticeTest, Empty) {
  SetDimensions(1, 2);
  A_ << 1, 0;
  b_ << 0;

  lower_bound_ << 1, 0;
  upper_bound_ << 2, 0;
  IntegerSet ref;

  EXPECT_EQ(EnumerationSolutions(), ref);
}

TEST_F(IntegerLatticeTest, Singleton) {
  SetDimensions(1, 4);
  A_ << 0, 0, 0, 0;
  b_ << 0;

  lower_bound_ << 1, 2, 3, 4;
  upper_bound_ << 1, 2, 3, 4;
  IntegerSet ref;
  ref.insert(lower_bound_);
  EXPECT_EQ(EnumerationSolutions(), ref);
}


TEST_F(IntegerLatticeTest, InfeasProp) {
  // Tests that EnumerateIntegerSolutions avoids exhaustive enumeration of m^n
  // points for intractable choice of m and n. This is enabled by the
  // infeasibility propapation feature of EnumerateIntegerSolutions.
  int n = 10;
  int m = 8;
  SetDimensions(1, n);

  // These bounds define a box B with m^n points.
  lower_bound_ << Eigen::VectorXi::Constant(n, 1);
  upper_bound_ << lower_bound_ * m;

  // This constraint is satisfied by one point in B.
  A_ << Eigen::MatrixXi::Constant(1, n, 1);
  b_ << A_*lower_bound_;

  IntegerSet ref;
  ref.insert(lower_bound_);
  EXPECT_EQ(EnumerationSolutions(), ref);
}


TEST_F(IntegerLatticeTest, EntireBoxFeasible) {
  SetDimensions(2, 2);
  A_ << 1, 1,
        2, 2;
  b_ << 10,
        20;

  lower_bound_ << 0, 0;
  upper_bound_ << 1, 2;
  Eigen::MatrixXi ref_mat(6, 2);
  ref_mat << 0, 0,
             0, 1,
             0, 2,
             1, 0,
             1, 1,
             1, 2;
  EXPECT_EQ(EnumerationSolutions(), MatrixToSet(ref_mat));
}


}  // namespace
}  // namespace solvers
}  // namespace drake
