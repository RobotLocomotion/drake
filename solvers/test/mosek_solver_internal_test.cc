#include "drake/solvers/mosek_solver_internal.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace solvers {
namespace internal {
using BarFType = std::vector<std::unordered_map<
    MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>;

void CheckParseLinearExpression(
    const MosekSolverProgram& dut, const MathematicalProgram& prog,
    const Eigen::SparseMatrix<double>& A, const Eigen::SparseMatrix<double>& B,
    const VectorX<symbolic::Variable>& decision_vars,
    const std::vector<MSKint32t>& slack_vars_mosek_indices,
    const std::vector<MSKint32t>& F_subi, const std::vector<MSKint32t>& F_subj,
    const std::vector<MSKrealt>& F_valij, const BarFType& bar_F) {
  VectorX<symbolic::Variable> slack_vars(slack_vars_mosek_indices.size());
  for (int i = 0; i < slack_vars.rows(); ++i) {
    slack_vars(i) = symbolic::Variable("slack" + std::to_string(i));
  }
  VectorX<symbolic::Expression> linear_exprs_expected =
      A * decision_vars + B * slack_vars;

  std::vector<Eigen::Triplet<double>> F_triplets(F_subi.size());
  for (int i = 0; i < static_cast<int>(F_triplets.size()); ++i) {
    F_triplets[i] = Eigen::Triplet<double>(F_subi[i], F_subj[i], F_valij[i]);
  }
  int num_mosek_vars;
  const auto rescode = MSK_getnumvar(dut.task(), &num_mosek_vars);
  ASSERT_EQ(rescode, MSK_RES_OK);
  Eigen::SparseMatrix<double> F(A.rows(), num_mosek_vars);
  F.setFromTriplets(F_triplets.begin(), F_triplets.end());
  // mosek_vars are the non-matrix variables stored inside Mosek.
  VectorX<symbolic::Variable> mosek_vars(num_mosek_vars);
  // bar_X are Mosek matrix variables.
  int num_barvar = 0;
  MSK_getnumbarvar(dut.task(), &num_barvar);
  std::vector<MatrixX<symbolic::Variable>> bar_X(num_barvar);
  // Now set up mosek_vars and bar_X.
  for (int i = 0; i < slack_vars.rows(); ++i) {
    mosek_vars(slack_vars_mosek_indices[i]) = slack_vars(i);
  }
  for (int i = 0; i < prog.num_vars(); ++i) {
    auto it1 = dut.decision_variable_to_mosek_nonmatrix_variable().find(i);
    if (it1 != dut.decision_variable_to_mosek_nonmatrix_variable().end()) {
      mosek_vars(it1->second) = prog.decision_variable(i);
    } else {
      auto it2 = dut.decision_variable_to_mosek_matrix_variable().find(i);
      ASSERT_NE(it2, dut.decision_variable_to_mosek_matrix_variable().end());
      bar_X[it2->second.bar_matrix_index()].resize(
          it2->second.num_matrix_rows(), it2->second.num_matrix_rows());
      bar_X[it2->second.bar_matrix_index()](it2->second.row_index(),
                                            it2->second.col_index()) =
          prog.decision_variable(i);
      bar_X[it2->second.bar_matrix_index()](it2->second.col_index(),
                                            it2->second.row_index()) =
          prog.decision_variable(i);
    }
  }
  for (int i = 0; i < mosek_vars.rows(); ++i) {
    if (mosek_vars(i).is_dummy()) {
      mosek_vars(i) = symbolic::Variable("unused_slack");
    }
  }
  // Compute the linear expression.
  VectorX<symbolic::Expression> linear_exprs = F * mosek_vars;
  if (!bar_F.empty()) {
    // For each row i, compute ∑ⱼ <F̅ᵢⱼ, X̅ⱼ>
    EXPECT_EQ(bar_F.size(), A.rows());
    for (int i = 0; i < A.rows(); ++i) {
      for (const auto& [j, sub_weights] : bar_F[i]) {
        const std::vector<MSKint64t>& sub = sub_weights.first;
        const std::vector<MSKrealt>& weights = sub_weights.second;
        Eigen::MatrixXd F_bar_ij =
            Eigen::MatrixXd::Zero(bar_X[j].rows(), bar_X[j].cols());
        ASSERT_EQ(sub.size(), weights.size());
        for (int k = 0; k < static_cast<int>(sub.size()); ++k) {
          // First construct the selection matrix E from the index sub[k].
          // I know the selection matrix E always has 1 non-zero entry in the
          // lower triangular part.
          MSKint32t subi{0};
          MSKint32t subj{0};
          MSKrealt valij{0};
          MSK_getsparsesymmat(dut.task(), sub[k], 1, &subi, &subj, &valij);
          F_bar_ij(subi, subj) += valij * weights[k];
          if (subi != subj) {
            F_bar_ij(subj, subi) += valij * weights[k];
          }
        }
        linear_exprs(i) += (F_bar_ij.transpose() * bar_X[j]).trace();
      }
    }
  }

  EXPECT_EQ(linear_exprs.rows(), linear_exprs_expected.rows());
  for (int i = 0; i < linear_exprs.rows(); ++i) {
    EXPECT_PRED2(symbolic::test::ExprEqual, linear_exprs(i).Expand(),
                 linear_exprs_expected(i).Expand());
  }
}

GTEST_TEST(ParseLinearExpression, Test1) {
  // Test with non-empty A matrix and empty B matrix, no Mosek matrix variable.
  MathematicalProgram prog;
  auto dummy = prog.NewContinuousVariables<2>();
  auto x = prog.NewContinuousVariables<3>();

  Eigen::Matrix<double, 3, 2> A_dense;
  A_dense << 1, 2, 0, 1, 3, 0;
  Eigen::SparseMatrix<double> A_sparse = A_dense.sparseView();
  Eigen::SparseMatrix<double> B(3, 0);

  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  MosekSolverProgram dut(prog, env);
  MSK_appendvars(dut.task(),
                 dut.decision_variable_to_mosek_nonmatrix_variable().size());

  std::vector<MSKint32t> F_subi;
  std::vector<MSKint32t> F_subj;
  std::vector<MSKrealt> F_valij;
  BarFType bar_F;
  const auto rescode = dut.ParseLinearExpression(
      prog, A_sparse, B, x.tail<2>(), {}, &F_subi, &F_subj, &F_valij, &bar_F);
  ASSERT_EQ(rescode, MSK_RES_OK);
  EXPECT_EQ(F_subi.size(), A_sparse.nonZeros());
  EXPECT_EQ(F_subj.size(), A_sparse.nonZeros());
  EXPECT_EQ(F_valij.size(), A_sparse.nonZeros());
  EXPECT_TRUE(bar_F.empty());

  CheckParseLinearExpression(dut, prog, A_sparse, B, x.tail<2>(), {}, F_subi,
                             F_subj, F_valij, bar_F);
  MSK_deleteenv(&env);
}

GTEST_TEST(ParseLinearExpression, Test2) {
  // Test with non-empty A matrix and empty B matrix, with only Mosek matrix
  // variable and no non-matrix variable.
  MathematicalProgram prog;
  auto X1 = prog.NewSymmetricContinuousVariables<2>();
  auto X2 = prog.NewSymmetricContinuousVariables<3>();
  auto X3 = prog.NewSymmetricContinuousVariables<4>();
  prog.AddPositiveSemidefiniteConstraint(X1);
  prog.AddPositiveSemidefiniteConstraint(X2);
  prog.AddPositiveSemidefiniteConstraint(X3);

  Eigen::Matrix<double, 3, 3> A_dense;
  A_dense << 1, 2, 0, -1, 3, 0, 0, 2, -1;
  Eigen::SparseMatrix<double> A_sparse = A_dense.sparseView();
  Eigen::SparseMatrix<double> B(3, 0);

  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  MosekSolverProgram dut(prog, env);
  std::vector<MSKint32t> bar_var_dimension = {2, 3, 4};
  MSK_appendbarvars(dut.task(), 3, bar_var_dimension.data());

  Vector3<symbolic::Variable> decision_vars(X2(0, 1), X3(1, 1), X3(1, 2));
  std::vector<MSKint32t> F_subi;
  std::vector<MSKint32t> F_subj;
  std::vector<MSKrealt> F_valij;
  BarFType bar_F;
  const auto rescode = dut.ParseLinearExpression(
      prog, A_sparse, B, decision_vars, {}, &F_subi, &F_subj, &F_valij, &bar_F);
  ASSERT_EQ(rescode, MSK_RES_OK);
  EXPECT_TRUE(F_subi.empty());
  EXPECT_TRUE(F_subj.empty());
  EXPECT_TRUE(F_valij.empty());
  EXPECT_FALSE(bar_F.empty());
  CheckParseLinearExpression(dut, prog, A_sparse, B, decision_vars, {}, F_subi,
                             F_subj, F_valij, bar_F);
  MSK_deleteenv(&env);
}

GTEST_TEST(ParseLinearExpression, Test3) {
  // Test with non-empty A matrix and non-empty B matrix, with both Mosek matrix
  // variable and non-matrix variable.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  auto y = prog.NewContinuousVariables<4>();
  auto X1 = prog.NewSymmetricContinuousVariables<2>();
  auto X2 = prog.NewSymmetricContinuousVariables<3>();
  prog.AddPositiveSemidefiniteConstraint(X1);
  prog.AddPositiveSemidefiniteConstraint(X2);

  Eigen::Matrix<double, 3, 3> A_dense;
  A_dense << 1, 2, 0, -1, 3, 0, 0, 2, -1;
  Eigen::SparseMatrix<double> A_sparse = A_dense.sparseView();
  Eigen::Matrix<double, 3, 2> B_dense;
  B_dense << 1, 3, -2, 1, 0, 2;
  Eigen::SparseMatrix<double> B_sparse = B_dense.sparseView();

  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  MosekSolverProgram dut(prog, env);
  std::vector<MSKint32t> bar_var_dimension = {2, 3, 4};
  MSK_appendbarvars(dut.task(), 3, bar_var_dimension.data());
  const int num_slack_vars = 3;
  MSK_appendvars(dut.task(),
                 dut.decision_variable_to_mosek_nonmatrix_variable().size() +
                     num_slack_vars);
  std::vector<MSKint32t> slack_vars_mosek_indices = {
      static_cast<int>(
          dut.decision_variable_to_mosek_nonmatrix_variable().size()) +
          1,
      static_cast<int>(
          dut.decision_variable_to_mosek_nonmatrix_variable().size()) +
          2};

  Vector3<symbolic::Variable> decision_vars(X2(0, 1), X1(1, 1), y(2));
  std::vector<MSKint32t> F_subi;
  std::vector<MSKint32t> F_subj;
  std::vector<MSKrealt> F_valij;
  BarFType bar_F;
  const auto rescode = dut.ParseLinearExpression(
      prog, A_sparse, B_sparse, decision_vars, slack_vars_mosek_indices,
      &F_subi, &F_subj, &F_valij, &bar_F);
  ASSERT_EQ(rescode, MSK_RES_OK);
  EXPECT_FALSE(F_subi.empty());
  EXPECT_FALSE(F_subj.empty());
  EXPECT_FALSE(F_valij.empty());
  EXPECT_FALSE(bar_F.empty());
  CheckParseLinearExpression(dut, prog, A_sparse, B_sparse, decision_vars,
                             slack_vars_mosek_indices, F_subi, F_subj, F_valij,
                             bar_F);
  MSK_deleteenv(&env);
}

/**
 * Returns all of the affine expressions stored inside dut.task().
 */
VectorX<symbolic::Expression> GetAffineExpression(
    const MathematicalProgram& prog, const MosekSolverProgram& dut) {
  // First set up mosek variable.
  int num_mosek_vars;
  MSK_getnumvar(dut.task(), &num_mosek_vars);
  // mosek_vars are the non-matrix variables stored inside Mosek.
  VectorX<symbolic::Variable> mosek_vars(num_mosek_vars);
  // bar_X are Mosek matrix variables.
  int num_barvar = 0;
  MSK_getnumbarvar(dut.task(), &num_barvar);
  std::vector<MatrixX<symbolic::Variable>> bar_X(num_barvar);
  // Now set up mosek_vars and bar_X.
  for (int i = 0; i < prog.num_vars(); ++i) {
    auto it1 = dut.decision_variable_to_mosek_nonmatrix_variable().find(i);
    if (it1 != dut.decision_variable_to_mosek_nonmatrix_variable().end()) {
      mosek_vars(it1->second) = prog.decision_variable(i);
    } else {
      auto it2 = dut.decision_variable_to_mosek_matrix_variable().find(i);
      EXPECT_NE(it2, dut.decision_variable_to_mosek_matrix_variable().end());
      bar_X[it2->second.bar_matrix_index()].resize(
          it2->second.num_matrix_rows(), it2->second.num_matrix_rows());
      bar_X[it2->second.bar_matrix_index()](it2->second.row_index(),
                                            it2->second.col_index()) =
          prog.decision_variable(i);
      bar_X[it2->second.bar_matrix_index()](it2->second.col_index(),
                                            it2->second.row_index()) =
          prog.decision_variable(i);
    }
  }
  for (int i = 0; i < mosek_vars.rows(); ++i) {
    if (mosek_vars(i).is_dummy()) {
      mosek_vars(i) = symbolic::Variable("unused_slack");
    }
  }
  MSKint64t afe_f_nnz;
  MSK_getafefnumnz(dut.task(), &afe_f_nnz);
  std::vector<MSKint64t> afe_idx(afe_f_nnz);
  std::vector<MSKint32t> var_idx(afe_f_nnz);
  std::vector<MSKrealt> val(afe_f_nnz);
  MSK_getafeftrip(dut.task(), afe_idx.data(), var_idx.data(), val.data());
  std::vector<Eigen::Triplet<double>> F_triplets(afe_f_nnz);
  for (int i = 0; i < afe_f_nnz; ++i) {
    F_triplets[i] = Eigen::Triplet<double>(afe_idx[i], var_idx[i], val[i]);
  }
  MSKint64t num_afe;
  MSK_getnumafe(dut.task(), &num_afe);
  Eigen::SparseMatrix<double> F(num_afe, num_mosek_vars);
  F.setFromTriplets(F_triplets.begin(), F_triplets.end());
  Eigen::VectorXd g(num_afe);
  MSK_getafegslice(dut.task(), 0, num_afe, g.data());
  // The affine expression is F*x + <bar_F, bar_X> + g. We first compute the
  // part F*x+g.
  VectorX<symbolic::Expression> affine_expressions = F * mosek_vars + g;
  // Compute <bar_F, bar_X>.
  for (MSKint64t i = 0; i < num_afe; ++i) {
    MSKint32t num_entry;
    MSKint64t num_term_total;
    MSK_getafebarfrowinfo(dut.task(), i, &num_entry, &num_term_total);
    std::vector<MSKint32t> barvar_idx(num_entry);
    std::vector<MSKint64t> ptr_term(num_entry);
    std::vector<MSKint64t> num_term(num_entry);
    std::vector<MSKint64t> term_idx(num_term_total);
    std::vector<MSKrealt> term_weight(num_term_total);
    MSK_getafebarfrow(dut.task(), i, barvar_idx.data(), ptr_term.data(),
                      num_term.data(), term_idx.data(), term_weight.data());
    for (int k = 0; k < num_entry; ++k) {
      const MSKint32t j = barvar_idx[k];
      Eigen::MatrixXd bar_F_ij =
          Eigen::MatrixXd::Zero(bar_X[j].rows(), bar_X[j].rows());
      for (int l = ptr_term[k]; l < ptr_term[k] + num_term[k]; ++l) {
        // I know the selection matrix E always has 1 non-zero entry in the
        // lower triangular part.
        MSKint32t subi{0};
        MSKint32t subj{0};
        MSKrealt valij{0};
        MSK_getsparsesymmat(dut.task(), term_idx[l], 1, &subi, &subj, &valij);
        bar_F_ij(subi, subj) += valij * term_weight[l];
        if (subi != subj) {
          bar_F_ij(subj, subi) += valij * term_weight[l];
        }
      }
      affine_expressions(i) += (bar_F_ij.transpose() * bar_X[j]).trace();
    }
  }

  return affine_expressions;
}

GTEST_TEST(AddConeConstraings, Test1) {
  // Test Lorentz cone constraint.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();

  Eigen::Matrix<double, 4, 2> A1;
  A1 << 1, 2, -1, 3, 0, 2, 1, 0;
  Eigen::Vector4d b1(1, 0, -2, 3);
  auto constraint1 = prog.AddLorentzConeConstraint(A1, b1, x.tail<2>());
  Eigen::Matrix<double, 3, 2> A2;
  A2 << 1, 2, -1, 3, 0, 1;
  Eigen::Vector3d b2(2, 1, 0);
  auto constraint2 = prog.AddLorentzConeConstraint(A2, b2, x.head<2>());
  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  MosekSolverProgram dut(prog, env);
  MSK_appendvars(dut.task(),
                 dut.decision_variable_to_mosek_nonmatrix_variable().size());
  std::unordered_map<Binding<LorentzConeConstraint>, MSKint64t> acc_indices;
  auto rescode = dut.AddConeConstraints(prog, prog.lorentz_cone_constraints(),
                                        &acc_indices);
  // Check the number of affine expressions, affine cone constraints, domains.
  ASSERT_EQ(rescode, MSK_RES_OK);
  MSKint64t num_afe;
  MSK_getnumafe(dut.task(), &num_afe);
  EXPECT_EQ(num_afe, 7);
  MSKint64t num_acc;
  MSK_getnumacc(dut.task(), &num_acc);
  EXPECT_EQ(num_acc, 2);
  MSKint64t acc_total;
  MSK_getaccntot(dut.task(), &acc_total);
  EXPECT_EQ(acc_total, 7);
  EXPECT_EQ(acc_indices.size(), 2);
  EXPECT_EQ(acc_indices.at(constraint1), 0);
  EXPECT_EQ(acc_indices.at(constraint2), 1);
  MSKint64t num_domain;
  MSK_getnumdomain(dut.task(), &num_domain);
  EXPECT_EQ(num_domain, 2);
  // Check domain types.
  for (MSKint64t i = 0; i < num_domain; ++i) {
    MSKdomaintypee domain_type;
    MSK_getdomaintype(dut.task(), i, &domain_type);
    EXPECT_EQ(domain_type, MSK_DOMAIN_QUADRATIC_CONE);
  }

  // Checks the affine expressions.
  MSKint64t afe_f_nnz;
  MSK_getafefnumnz(dut.task(), &afe_f_nnz);
  EXPECT_EQ(afe_f_nnz, constraint1.evaluator()->A().nonZeros() +
                           constraint2.evaluator()->A().nonZeros());
  const VectorX<symbolic::Expression> affine_expressions =
      GetAffineExpression(prog, dut);
  VectorX<symbolic::Expression> affine_expressions_expected(7);
  affine_expressions_expected.head<4>() = A1 * x.tail<2>() + b1;
  affine_expressions_expected.tail<3>() = A2 * x.head<2>() + b2;
  EXPECT_EQ(affine_expressions.rows(), affine_expressions_expected.rows());
  for (int i = 0; i < affine_expressions.rows(); ++i) {
    EXPECT_PRED2(symbolic::test::ExprEqual, affine_expressions(i).Expand(),
                 affine_expressions_expected(i).Expand());
  }

  MSK_deleteenv(&env);
}

GTEST_TEST(AddConeConstraints, Test2) {
  // Test rotated Lorentz cone constraint.
  // This program has positive semidefinite constraints, hence Mosek has matrix
  // variables.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  auto X1 = prog.NewSymmetricContinuousVariables<3>();
  auto X2 = prog.NewSymmetricContinuousVariables<4>();
  prog.AddPositiveSemidefiniteConstraint(X1);
  prog.AddPositiveSemidefiniteConstraint(X2);

  Eigen::Matrix<double, 4, 2> A1;
  A1 << 1, 2, -1, 3, 0, 2, 1, 0;
  Eigen::Vector4d b1(1, 0, -2, 3);
  auto constraint1 = prog.AddRotatedLorentzConeConstraint(
      A1, b1, Vector2<symbolic::Variable>(x(0), X1(1, 1)));
  Eigen::Matrix<double, 3, 3> A2;
  A2 << 1, 2, -1, 3, 0, 1, 0, 1, -4;
  Eigen::Vector3d b2(2, 1, 0);
  auto constraint2 = prog.AddRotatedLorentzConeConstraint(
      A2, b2, Vector3<symbolic::Variable>(x(2), X1(0, 1), X2(2, 1)));
  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  MosekSolverProgram dut(prog, env);
  MSK_appendvars(dut.task(),
                 dut.decision_variable_to_mosek_nonmatrix_variable().size());
  std::vector<MSKint32t> bar_var_dimension = {3, 4};
  MSK_appendbarvars(dut.task(), 2, bar_var_dimension.data());
  std::unordered_map<Binding<RotatedLorentzConeConstraint>, MSKint64t>
      acc_indices;
  auto rescode = dut.AddConeConstraints(
      prog, prog.rotated_lorentz_cone_constraints(), &acc_indices);
  ASSERT_EQ(rescode, MSK_RES_OK);

  // Check acc_indices.
  EXPECT_EQ(acc_indices.size(), 2);
  EXPECT_EQ(acc_indices.at(constraint1), 0);
  EXPECT_EQ(acc_indices.at(constraint2), 1);

  // Check domain types.
  MSKint64t num_domain;
  MSK_getnumdomain(dut.task(), &num_domain);
  EXPECT_EQ(num_domain, 2);
  for (MSKint64t i = 0; i < num_domain; ++i) {
    MSKdomaintypee domain_type;
    MSK_getdomaintype(dut.task(), i, &domain_type);
    EXPECT_EQ(domain_type, MSK_DOMAIN_RQUADRATIC_CONE);
  }
  // Check affine expressions in Mosek.
  const VectorX<symbolic::Expression> affine_expressions =
      GetAffineExpression(prog, dut);
  VectorX<symbolic::Expression> affine_expressions_expected(A1.rows() +
                                                            A2.rows());
  affine_expressions_expected.head(A1.rows()) =
      A1 * constraint1.variables() + b1;
  affine_expressions_expected(0) *= 0.5;
  affine_expressions_expected.tail(A2.rows()) =
      A2 * constraint2.variables() + b2;
  affine_expressions_expected(A1.rows()) *= 0.5;
  EXPECT_EQ(affine_expressions.rows(), affine_expressions_expected.rows());
  for (int i = 0; i < affine_expressions.rows(); ++i) {
    EXPECT_PRED2(symbolic::test::ExprEqual, affine_expressions(i).Expand(),
                 affine_expressions_expected(i).Expand());
  }

  MSK_deleteenv(&env);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the MOSEK license for the entire duration of this test,
  // so that we do not have to release and re-acquire the license for every
  // test.
  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
