#include "drake/solvers/ipopt_solver_internal.h"

#include <IpIpoptApplication.hpp>
#include <IpTNLP.hpp>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

using Ipopt::Index;
using Ipopt::Number;

namespace drake {
namespace solvers {
namespace internal {

void GetConstraintJacobian(IpoptSolver_NLP* dut, Index n, Index m,
                           Index nnz_jac_g, const Number* x,
                           Eigen::SparseMatrix<double>* jac) {
  std::vector<Index> iRow(nnz_jac_g);
  std::vector<Index> jCol(nnz_jac_g);
  std::vector<double> values(nnz_jac_g);
  // IPOPT requires calling eval_jac_g for twice to get the Jacobian, the first
  // time to get iRow/jCol, and the second time to get values.
  bool status = dut->eval_jac_g(n, x, true /* new_x */, m, nnz_jac_g,
                                iRow.data(), jCol.data(), nullptr);
  ASSERT_TRUE(status);
  status = dut->eval_jac_g(n, x, false /* new_x */, m, nnz_jac_g, nullptr,
                           nullptr, values.data());
  std::vector<Eigen::Triplet<double>> jacobian_triplets;
  for (int i = 0; i < nnz_jac_g; ++i) {
    jacobian_triplets.emplace_back(iRow[i], jCol[i], values[i]);
  }
  jac->setFromTriplets(jacobian_triplets.begin(), jacobian_triplets.end());
}

GTEST_TEST(TestIpoptSolverNlp, LinearConstraints) {
  // Test a program with only linear constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  Eigen::Matrix<double, 2, 3> A;
  A << 1, 0, 3, 2, -4, 0;
  Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  Eigen::Vector2d lb(1, 3);
  Eigen::Vector2d ub(2, 5);
  prog.AddLinearConstraint(A, lb, ub, x.tail<3>());

  Eigen::Vector4d x_init(0, 1, 2, 3);
  MathematicalProgramResult result;
  IpoptSolver_NLP dut(prog, x_init, &result);
  Index n;
  Index m;
  Index nnz_jac_g;
  Index nnz_h_lag;
  Ipopt::TNLP::IndexStyleEnum index_style;
  bool status = dut.get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style);
  EXPECT_TRUE(status);
  EXPECT_EQ(n, 4);
  EXPECT_EQ(m, 2);
  EXPECT_EQ(nnz_jac_g, A_sparse.nonZeros());
  EXPECT_EQ(nnz_h_lag, 0);
  EXPECT_EQ(index_style, Ipopt::TNLP::IndexStyleEnum::C_STYLE);

  // Test eval_jac_g
  Eigen::SparseMatrix<double> jac(m, n);
  GetConstraintJacobian(&dut, n, m, nnz_jac_g, x_init.data(), &jac);
  Eigen::Matrix<double, 2, 4> jac_expected;
  jac_expected << 0, 1, 0, 3, 0, 2, -4, 0;
  EXPECT_TRUE(CompareMatrices(jac.toDense(), jac_expected));
}

GTEST_TEST(TestIpoptSolverNlp, LinearEqualityConstraints) {
  // Test a program with only linear equality constraints.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  Eigen::Matrix<double, 2, 3> A;
  A << 1, 0, 3, 2, -4, 0;
  Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  Eigen::Vector2d lb(1, 3);
  Eigen::Vector2d ub(2, 5);
  prog.AddLinearConstraint(A, lb, ub, x.tail<3>());
  prog.AddLinearConstraint(A, lb, ub, x.head<3>());

  Eigen::Vector4d x_init(0, 1, 2, 3);
  MathematicalProgramResult result;
  IpoptSolver_NLP dut(prog, x_init, &result);
  Index n;
  Index m;
  Index nnz_jac_g;
  Index nnz_h_lag;
  Ipopt::TNLP::IndexStyleEnum index_style;
  bool status = dut.get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style);
  EXPECT_TRUE(status);
  EXPECT_EQ(n, 4);
  EXPECT_EQ(m, 4);
  EXPECT_EQ(nnz_jac_g, 2 * A_sparse.nonZeros());
  EXPECT_EQ(nnz_h_lag, 0);
  EXPECT_EQ(index_style, Ipopt::TNLP::IndexStyleEnum::C_STYLE);

  // Test eval_jac_g
  Eigen::SparseMatrix<double> jac(m, n);
  GetConstraintJacobian(&dut, n, m, nnz_jac_g, x_init.data(), &jac);
  Eigen::Matrix<double, 4, 4> jac_expected;
  // clang-format off
  jac_expected << 0, 1, 0, 3,
                  0, 2, -4, 0,
                  1, 0, 3, 0,
                  2, -4, 0, 0;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(jac.toDense(), jac_expected));
}

GTEST_TEST(TestIpoptSolverNlp, LinearConstraintsDuplicatedVariable) {
  // Test a program with linear constraints containing duplicated variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<4>();
  Eigen::Matrix<double, 2, 3> A;
  A << 1, 0, 3, 0, 3, 5;
  const Eigen::SparseMatrix<double> A_sparse = A.sparseView();
  Eigen::Vector2d lb(1, 2);
  Eigen::Vector2d ub(3, 4);
  prog.AddLinearConstraint(A, lb, ub,
                           Vector3<symbolic::Variable>(x(0), x(1), x(0)));
  Eigen::Vector4d x_init(0, 1, 2, 3);
  MathematicalProgramResult result;
  IpoptSolver_NLP dut(prog, x_init, &result);
  Index n;
  Index m;
  Index nnz_jac_g;
  Index nnz_h_lag;
  Ipopt::TNLP::IndexStyleEnum index_style;
  bool status = dut.get_nlp_info(n, m, nnz_jac_g, nnz_h_lag, index_style);
  EXPECT_TRUE(status);
  EXPECT_EQ(n, 4);
  EXPECT_EQ(m, 2);
  EXPECT_LE(nnz_jac_g, A_sparse.nonZeros());
  EXPECT_EQ(nnz_h_lag, 0);
  EXPECT_EQ(index_style, Ipopt::TNLP::IndexStyleEnum::C_STYLE);

  // Test eval_jac_g
  Eigen::SparseMatrix<double> jac(m, n);
  GetConstraintJacobian(&dut, n, m, nnz_jac_g, x_init.data(), &jac);
  Eigen::Matrix<double, 2, 4> jac_expected;
  // clang-format off
  jac_expected << 4, 0, 0, 0,
                  5, 3, 0, 0;
  // clang-format on
  EXPECT_TRUE(CompareMatrices(jac.toDense(), jac_expected));
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake
