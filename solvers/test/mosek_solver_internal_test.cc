#include "drake/solvers/mosek_solver_internal.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/math/quadratic_form.h"
#include "drake/solvers/mosek_solver.h"

namespace drake {
namespace solvers {
namespace internal {
const double kInf = std::numeric_limits<double>::infinity();

using BarFType = std::vector<std::unordered_map<
    MSKint64t, std::pair<std::vector<MSKint64t>, std::vector<MSKrealt>>>>;

// By default, the newly appended variables in Mosek are fixed to 0. Hence,
// their bounds need to be explicitly set to -inf and inf.
void AppendFreeVariable(MSKtask_t task, int num_vars) {
  int num_existing_vars;
  MSK_getnumvar(task, &num_existing_vars);
  MSK_appendvars(task, num_vars);
  for (int i = 0; i < num_vars; ++i) {
    MSK_putvarbound(task, num_existing_vars + i, MSK_BK_FR, -MSK_INFINITY,
                    MSK_INFINITY);
  }
}

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
  AppendFreeVariable(
      dut.task(), dut.decision_variable_to_mosek_nonmatrix_variable().size());

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
  AppendFreeVariable(
      dut.task(), dut.decision_variable_to_mosek_nonmatrix_variable().size() +
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
 * @param slack_vars Mosek can create variables that are not
 * prog.decision_variables(). We call them "slack variables". `slack_vars` maps
 * the index of the variables in Mosek to its symbolic form. Returns all of the
 * affine expressions stored inside dut.task().
 */
VectorX<symbolic::Expression> GetAffineExpression(
    const MathematicalProgram& prog, const MosekSolverProgram& dut,
    const std::unordered_map<MSKint32t, symbolic::Variable>& slack_vars) {
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
      mosek_vars(i) = slack_vars.at(i);
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
  AppendFreeVariable(
      dut.task(), dut.decision_variable_to_mosek_nonmatrix_variable().size());
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
      GetAffineExpression(prog, dut, {});
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
  AppendFreeVariable(
      dut.task(), dut.decision_variable_to_mosek_nonmatrix_variable().size());
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
      GetAffineExpression(prog, dut, {});
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

GTEST_TEST(AddQuadraticCostAsLinearCost, Test) {
  // Test AddQuadraticCostAsLinearCost.
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<2>();
  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  MosekSolverProgram dut(prog, env);
  AppendFreeVariable(dut.task(), x.rows());

  Eigen::Matrix2d Q;
  Q << 1, 2, 2, 5;
  Eigen::SparseMatrix<double> Q_sparse = Q.sparseView();

  MSKrescodee rescode = dut.AddQuadraticCostAsLinearCost(Q_sparse, x, prog);
  ASSERT_EQ(rescode, MSK_RES_OK);
  MSKint32t num_vars;
  MSK_getnumvar(dut.task(), &num_vars);
  EXPECT_EQ(num_vars, x.rows() + 1);

  MSKint64t num_acc;
  MSK_getnumacc(dut.task(), &num_acc);
  EXPECT_EQ(num_acc, 1);

  MSKint64t num_afe;
  MSK_getnumafe(dut.task(), &num_afe);
  EXPECT_EQ(num_afe, Q.rows() + 2);

  MSKdomaintypee domain_type;
  MSK_getdomaintype(dut.task(), 0, &domain_type);
  EXPECT_EQ(domain_type, MSK_DOMAIN_RQUADRATIC_CONE);

  // Check cost.
  Eigen::VectorXd c(x.rows() + 1);
  MSK_getc(dut.task(), c.data());
  EXPECT_TRUE(
      CompareMatrices(c.head(x.rows()), Eigen::VectorXd::Zero(x.rows())));
  EXPECT_EQ(c(c.rows() - 1), 1);

  // Check the affine expression.
  const Eigen::MatrixXd L = math::DecomposePSDmatrixIntoXtransposeTimesX(
      Q, std::numeric_limits<double>::epsilon());
  std::unordered_map<MSKint32t, symbolic::Variable> slack_vars;
  symbolic::Variable s("s");
  slack_vars.emplace(x.rows(), s);
  VectorX<symbolic::Expression> affine_expressions =
      GetAffineExpression(prog, dut, slack_vars);
  EXPECT_PRED2(symbolic::test::ExprEqual,
               (2 * affine_expressions(0) * affine_expressions(1)).Expand(),
               2 * s);
  // The sum-of-squares for affine_expressions(i), i> 1 is x'*Q*x.
  EXPECT_PRED3(symbolic::test::PolynomialEqual,
               symbolic::Polynomial(
                   affine_expressions.tail(affine_expressions.rows() - 2)
                       .array()
                       .square()
                       .sum()),
               symbolic::Polynomial(x.cast<symbolic::Expression>().dot(Q * x)),
               1E-10);

  // Add an arbitrary linear cost.
  const Eigen::SparseMatrix<double> linear_coeff =
      Eigen::Vector2d(1, 2).sparseView();
  dut.AddLinearCost(linear_coeff, x, prog);
  MSK_getc(dut.task(), c.data());
  EXPECT_TRUE(CompareMatrices(c.head(x.rows()), linear_coeff.toDense()));
  EXPECT_EQ(c(c.rows() - 1), 1);

  MSKrescodee terminal_code;
  MSK_optimizetrm(dut.task(), &terminal_code);

  MSKsoltypee solution_type = MSK_SOL_ITR;

  MSKsolstae solution_status;
  MSK_getsolsta(dut.task(), solution_type, &solution_status);
  EXPECT_EQ(solution_status, MSK_SOL_STA_OPTIMAL);

  Eigen::VectorXd acc_val(2 + Q.rows());
  MSK_evaluateacc(dut.task(), solution_type, 0, acc_val.data());
  Eigen::Vector3d mosek_var_sol;
  MSK_getxx(dut.task(), solution_type, mosek_var_sol.data());
  const Eigen::Vector2d x_sol = mosek_var_sol.head<2>();
  const double s_sol = mosek_var_sol(2);
  // Check the cost value.
  EXPECT_NEAR(0.5 * x_sol.dot(Q * x_sol), s_sol, 1E-8);
  // Check the optimality condition, the gradient of the cost is 0.
  EXPECT_TRUE(CompareMatrices(Q * x_sol + linear_coeff.toDense(),
                              Eigen::VectorXd::Zero(x.rows()), 1E-10));

  MSK_deleteenv(&env);
}

GTEST_TEST(AddQuadraticConstraint, Test) {
  MathematicalProgram prog;
  MSKenv_t env;
  MSK_makeenv(&env, nullptr);
  auto x = prog.NewContinuousVariables<3>();
  MosekSolverProgram dut(prog, env);
  AppendFreeVariable(
      dut.task(), dut.decision_variable_to_mosek_nonmatrix_variable().size());
  // Add a linear constraint.
  prog.AddLinearConstraint(Eigen::RowVector2d(1, 2), -10, 20, x.head<2>());
  std::unordered_map<Binding<LinearConstraint>, ConstraintDualIndices>
      linear_con_dual_indices;
  std::unordered_map<Binding<LinearEqualityConstraint>, ConstraintDualIndices>
      linear_eq_con_dual_indices;
  MSKrescodee rescode = dut.AddLinearConstraints(prog, &linear_con_dual_indices,
                                                 &linear_eq_con_dual_indices);
  // Add a quadratic constraint on a subset of variables.
  const Eigen::Matrix2d Q0 = Eigen::Vector2d(1, 2).asDiagonal();
  const Eigen::Vector2d b0(2, -3);
  const auto quadratic_con0 = prog.AddQuadraticConstraint(
      Q0, b0, -kInf, 10, Vector2<symbolic::Variable>(x(2), x(0)));
  // Add a quadratic constraint on duplicated variables.
  // I intentially use a non-symmetric Q.
  Eigen::Matrix2d Q1;
  // clang-format off
  Q1 << -1, -3,
        -1, -5;
  // clang-format on
  const Eigen::Vector2d b1(1, -2);
  const auto quadratic_con1 = prog.AddQuadraticConstraint(
      Q1, b1, -2, kInf, Vector2<symbolic::Variable>(x(1), x(1)));
  std::unordered_map<Binding<QuadraticConstraint>, MSKint64t>
      quadratic_constraint_dual_indices;
  rescode =
      dut.AddQuadraticConstraints(prog, &quadratic_constraint_dual_indices);
  EXPECT_EQ(rescode, MSK_RES_OK);
  EXPECT_EQ(quadratic_constraint_dual_indices.size(), 2);
  EXPECT_EQ(quadratic_constraint_dual_indices.at(quadratic_con0), 1);
  EXPECT_EQ(quadratic_constraint_dual_indices.at(quadratic_con1), 2);
  {
    // Check the Hessian in the first quadratic constraint.
    MSKint32t maxnumqcnz = 3;
    int numqcnz{0};
    std::vector<MSKint32t> qcsubi(maxnumqcnz);
    std::vector<MSKint32t> qcsubj(maxnumqcnz);
    std::vector<MSKrealt> qcval(maxnumqcnz);
    MSK_getqconk(dut.task(), 1, maxnumqcnz, &numqcnz, qcsubi.data(),
                 qcsubj.data(), qcval.data());
    EXPECT_EQ(numqcnz, 2);
    Eigen::Matrix3d Q0_mosek;
    Q0_mosek.setZero();
    for (int i = 0; i < numqcnz; ++i) {
      Q0_mosek(qcsubi[i], qcsubj[i]) = qcval[i];
    }
    Eigen::Matrix3d Q0_mosek_expected = Eigen::Vector3d(2, 0, 1).asDiagonal();
    EXPECT_TRUE(CompareMatrices(Q0_mosek, Q0_mosek_expected));
  }
  {
    // Check the Hessian in the second quadratic constraint.
    MSKint32t maxnumqcnz = 3;
    int numqcnz{0};
    std::vector<MSKint32t> qcsubi(maxnumqcnz);
    std::vector<MSKint32t> qcsubj(maxnumqcnz);
    std::vector<MSKrealt> qcval(maxnumqcnz);
    MSK_getqconk(dut.task(), 2, maxnumqcnz, &numqcnz, qcsubi.data(),
                 qcsubj.data(), qcval.data());
    EXPECT_EQ(numqcnz, 1);
    Eigen::Matrix3d Q1_mosek;
    Q1_mosek.setZero();
    for (int i = 0; i < numqcnz; ++i) {
      Q1_mosek(qcsubi[i], qcsubj[i]) = qcval[i];
    }
    Eigen::Matrix3d Q1_mosek_expected = Eigen::Vector3d(0, -10, 0).asDiagonal();
    EXPECT_TRUE(CompareMatrices(Q1_mosek, Q1_mosek_expected));
  }
  // Check the bound of the quadratic constraint.
  {
    MSKboundkeye bound_key;
    MSKrealt bl;
    MSKrealt bu;
    // Test the bound of the first quadratic constraint.
    MSK_getconbound(dut.task(), 1, &bound_key, &bl, &bu);
    EXPECT_EQ(bound_key, MSK_BK_UP);
    EXPECT_EQ(bu, quadratic_con0.evaluator()->upper_bound()(0));
    EXPECT_EQ(bl, -MSK_INFINITY);
    // Test the bound of the second quadratic constraint.
    MSK_getconbound(dut.task(), 2, &bound_key, &bl, &bu);
    EXPECT_EQ(bound_key, MSK_BK_LO);
    EXPECT_EQ(bl, quadratic_con1.evaluator()->lower_bound()(0));
    EXPECT_EQ(bu, MSK_INFINITY);
  }
  // Test the linear coefficient.
  {
    MSKint32t nzi;
    std::vector<MSKint32t> subi(prog.num_vars());
    std::vector<MSKrealt> vali(prog.num_vars());
    // Test the linear coefficient of the first quadratic constraint
    MSK_getarow(dut.task(), 1, &nzi, subi.data(), vali.data());
    EXPECT_EQ(nzi, 2);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(prog.num_vars());
    for (int i = 0; i < nzi; ++i) {
      a(subi[i]) = vali[i];
    }
    EXPECT_TRUE(CompareMatrices(a, Eigen::Vector3d(b0(1), 0, b0(0))));
    // Test the linear coefficient of the second quadratic constraint
    MSK_getarow(dut.task(), 2, &nzi, subi.data(), vali.data());
    EXPECT_EQ(nzi, 1);
    EXPECT_EQ(subi[0], 1);
    EXPECT_EQ(vali[0], b1(0) + b1(1));
  }
  // Solve the optimization problem
  MSKrescodee terminal_code;
  MSK_optimizetrm(dut.task(), &terminal_code);

  MSKsoltypee solution_type = MSK_SOL_ITR;

  MSKsolstae solution_status;
  MSK_getsolsta(dut.task(), solution_type, &solution_status);
  EXPECT_EQ(solution_status, MSK_SOL_STA_OPTIMAL);
  Eigen::Vector3d mosek_var_sol;
  MSK_getxx(dut.task(), solution_type, mosek_var_sol.data());
  const Eigen::Vector3d x_sol = mosek_var_sol;
  EXPECT_TRUE(quadratic_con0.evaluator()->CheckSatisfied(
      Eigen::Vector2d(x_sol(2), x_sol(0))));
  EXPECT_TRUE(quadratic_con1.evaluator()->CheckSatisfied(
      Eigen::Vector2d(x_sol(1), x_sol(1))));

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
