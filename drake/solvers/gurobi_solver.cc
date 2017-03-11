#include "drake/solvers/gurobi_solver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

// NOLINTNEXTLINE(build/include) False positive due to weird include style.
#include "gurobi_c++.h"

#include "drake/common/drake_assert.h"
#include "drake/math/eigen_sparse_triplet.h"

// TODO(hongkai.dai): GurobiSolver class should store data member such as
// GRB_model, GRB_env, is_new_variables, etc.
namespace drake {
namespace solvers {
namespace {
// Checks if the number of variables in the Gurobi model is as expected. This
// operation can be EXPENSIVE, since it requires calling GRBupdatemodel
// (Gurobi typically adopts lazy update, where it does not update the model
// until calling the optimize function).
// This function should only be used in DEBUG mode as a sanity check.
__attribute__((unused)) bool HasCorrectNumberOfVariables(
    GRBmodel* model, int num_vars_expected) {
  int error = GRBupdatemodel(model);
  if (error) return false;
  int num_vars{};
  error = GRBgetintattr(model, "NumVars", &num_vars);
  if (error) return false;
  return (num_vars == num_vars_expected);
}

/**
 * Adds a constraint of one of the following forms :
 * lb ≤ A*x ≤ ub
 * or
 * A*x == lb
 *
 * @param is_equality True if the imposed constraint is
 * A*x == lb, false otherwise.
 * @return error as an integer. The full set of error values are
 * described here :
 * http://www.gurobi.com/documentation/6.5/refman/error_codes.html#sec:ErrorCodes
 *
 * TODO(hongkai.dai): Use a sparse matrix A.
 */
template <typename DerivedA, typename DerivedLB, typename DerivedUB>
int AddLinearConstraint(const MathematicalProgram& prog, GRBmodel* model,
                        const Eigen::MatrixBase<DerivedA>& A,
                        const Eigen::MatrixBase<DerivedLB>& lb,
                        const Eigen::MatrixBase<DerivedUB>& ub,
                        const Eigen::Ref<const VectorXDecisionVariable>& vars,
                        bool is_equality, double sparseness_threshold) {
  for (int i = 0; i < A.rows(); i++) {
    int nonzero_coeff_count = 0;
    std::vector<int> nonzero_var_index(A.cols(), 0);
    std::vector<double> nonzero_coeff(A.cols(), 0.0);

    for (int j = 0; j < A.cols(); j++) {
      if (std::abs(A(i, j)) > sparseness_threshold) {
        nonzero_coeff[nonzero_coeff_count] = A(i, j);
        nonzero_var_index[nonzero_coeff_count++] =
            prog.FindDecisionVariableIndex(vars(j));
      }
    }
    // The sense of the constraint could be ==, <= or >=
    int error = 0;
    if (is_equality) {
      // Adds equality constraint.
      error = GRBaddconstr(model, nonzero_coeff_count, &nonzero_var_index[0],
                           &nonzero_coeff[0], GRB_EQUAL, lb(i), nullptr);
      DRAKE_ASSERT(!error);
      if (error) return error;
    } else {
      if (!std::isinf(ub(i)) || !std::isinf(lb(i))) {
        if (!std::isinf(lb(i))) {
          // Adds A.row(i)*x >= lb(i).
          error = GRBaddconstr(model, nonzero_coeff_count,
                               &nonzero_var_index[0], &nonzero_coeff[0],
                               GRB_GREATER_EQUAL, lb(i), nullptr);
          DRAKE_ASSERT(!error);
          if (error) return error;
        }
        if (!std::isinf(ub(i))) {
          // Adds A.row(i)*x <= ub(i).
          error =
              GRBaddconstr(model, nonzero_coeff_count, &nonzero_var_index[0],
                           &nonzero_coeff[0], GRB_LESS_EQUAL, ub(i), nullptr);
          DRAKE_ASSERT(!error);
          if (error) return error;
        }
      }
    }
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

/*
 * Add (rotated) Lorentz cone constraints, that z = A*x+b is in the (rotated)
 * Lorentz cone.
 * A vector z is in the Lorentz cone, if
 * z(0) >= sqrt(z(1)^2 + ... + z(N-1)^2)
 * A vector z is in the rotated Lorentz cone, if
 * z(0)*z(1) >= z(2)^2 + ... + z(N-1)^2
 * z(0) >= 0, z(1) >= 0
 * @tparam C  A constraint type, either LorentzConeConstraint or
 * RotatedLorentzConeConstraint.
 * @param second_order_cone_constraints  A vector of Binding objects, containing
 * either Lorentz cone constraints, or rotated Lorentz cone constraints.
 * @param sparseness_threshold. If the absolute value of an entry in A, b
 * matrices inside (rotated) Lorentz cone constraint is smaller than
 * \p sparseness_threshold, that entry is ignored.
 * @param second_order_cone_new_variable_indices. The indices of variable z in
 * the Gurobi model.
 * @param model The Gurobi model.
 */
template <typename C>
int AddSecondOrderConeConstraints(
    const MathematicalProgram& prog,
    const std::vector<Binding<C>>& second_order_cone_constraints,
    double sparseness_threshold,
    const std::vector<std::vector<int>>& second_order_cone_new_variable_indices,
    GRBmodel* model) {
  static_assert(
      std::is_same<C, LorentzConeConstraint>::value ||
          std::is_same<C, RotatedLorentzConeConstraint>::value,
      "Expects either LorentzConeConstraint or RotatedLorentzConeConstraint");
  bool is_rotated_cone = std::is_same<C, RotatedLorentzConeConstraint>::value;

  DRAKE_ASSERT(second_order_cone_constraints.size() ==
               second_order_cone_new_variable_indices.size());
  int second_order_cone_count = 0;
  for (const auto& binding : second_order_cone_constraints) {
    const auto& A = binding.constraint()->A();
    const auto& b = binding.constraint()->b();

    int num_x = A.cols();
    int num_z = A.rows();

    // Add the constraint z - A*x = b
    std::vector<int> xz_indices(num_x + 1,
                                0);  // Records the indices of [x;z(i)],
                                     // Namely the variables in the i'th
                                     // row of z - A*x = b
    for (int i = 0; i < num_x; ++i) {
      xz_indices[i] = prog.FindDecisionVariableIndex(binding.variables()(i));
    }
    Eigen::RowVectorXd coeff_i(num_x + 1);  // Records the coefficients of the
                                            // i'th row in z - A*x = b
    for (int i = 0; i < num_z; ++i) {
      coeff_i << -A.row(i), 1;
      xz_indices[num_x] =
          second_order_cone_new_variable_indices[second_order_cone_count]
                                                [i];  // index of z(i)
      int error = GRBaddconstr(model, num_x + 1, xz_indices.data(),
                               coeff_i.data(), GRB_EQUAL, b(i), nullptr);
      DRAKE_ASSERT(!error);
      if (error) return error;
    }

    // Gurobi uses a matrix Q to differentiate Lorentz cone and rotated Lorentz
    // cone constraint.
    // https://www.gurobi.com/documentation/7.0/refman/c_grbaddqconstr.html
    // For Lorentz cone constraint,
    // Q = [-1 0 0 ... 0]
    //     [ 0 1 0 ... 0]
    //     [ 0 0 1 ... 0]
    //          ...
    //     [ 0 0 0 ... 1]
    // namely Q = diag([-1; 1; 1; ...; 1], so
    // z' * Q * z = z(1)^2 + ... + z(n-1)^2 - z(0)^2.
    // For rotated Lorentz cone constraint
    // Q = [0 -1 0 0 ... 0]
    //     [0  0 0 0 ... 0]
    //     [0  0 1 0 ... 0]
    //     [0  0 0 1 ... 0]
    //           ...
    //     [0  0 0 0 ... 1]
    // so z' * Q * z = z(2)^2 + ... + z(n-1)^2 - z(0) * z(1).
    // We will store Q in a sparse format.
    // qrow stores the row    indices of the non-zero entries of Q.
    // qcol stores the column indices of the non-zero entries of Q.
    // qval stores the value          of the non-zero entries of Q.
    size_t num_Q_nonzero = is_rotated_cone ? num_z - 1 : num_z;
    std::vector<int> qrow(num_Q_nonzero);
    std::vector<int> qcol(num_Q_nonzero);
    std::vector<double> qval(num_Q_nonzero);
    for (int i = 0; i < num_z - 2; ++i) {
      int zi_index =
          second_order_cone_new_variable_indices[second_order_cone_count]
                                                [i + 2];
      qrow[i] = zi_index;
      qcol[i] = zi_index;
      qval[i] = 1.0;
    }
    int z0_index =
        second_order_cone_new_variable_indices[second_order_cone_count][0];
    int z1_index =
        second_order_cone_new_variable_indices[second_order_cone_count][1];
    if (is_rotated_cone) {
      qrow[num_z - 2] = z0_index;
      qcol[num_z - 2] = z1_index;
      qval[num_z - 2] = -1;
    } else {
      qrow[num_z - 2] = z0_index;
      qcol[num_z - 2] = z0_index;
      qval[num_z - 2] = -1;
      qrow[num_z - 1] = z1_index;
      qcol[num_z - 1] = z1_index;
      qval[num_z - 1] = 1;
    }
    int error =
        GRBaddqconstr(model, 0, nullptr, nullptr, num_Q_nonzero, qrow.data(),
                      qcol.data(), qval.data(), GRB_LESS_EQUAL, 0.0, NULL);
    if (error) {
      return error;
    }
    ++second_order_cone_count;
  }
  return 0;
}

/*
 * Add quadratic or linear costs to the optimization problem.
 */
int AddCosts(GRBmodel* model, const MathematicalProgram& prog,
             double sparseness_threshold) {
  // Aggregates the quadratic costs and linear costs in the form
  // 0.5 * x' * Q_all * x + linear_term' * x.
  using std::abs;
  // record the non-zero entries in the cost 0.5*x'*Q*x + b'*x.
  std::vector<Eigen::Triplet<double>> Q_nonzero_coefs;
  std::vector<Eigen::Triplet<double>> b_nonzero_coefs;
  for (const auto& binding : prog.quadratic_costs()) {
    const auto& constraint = binding.constraint();
    const int constraint_variable_dimension = binding.GetNumElements();
    const Eigen::MatrixXd& Q = constraint->Q();
    const Eigen::VectorXd& b = constraint->b();

    DRAKE_ASSERT(Q.rows() == constraint_variable_dimension);

    // constraint_variable_index[i] is the index of the i'th decision variable
    // binding.GetFlattendSolution(i).
    std::vector<int> constraint_variable_index(constraint_variable_dimension);

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      constraint_variable_index[i] =
          prog.FindDecisionVariableIndex(binding.variables()(i));
    }

    for (int i = 0; i < Q.rows(); i++) {
      const double Qii = 0.5 * Q(i, i);
      if (abs(Qii) > sparseness_threshold) {
        Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
            constraint_variable_index[i], constraint_variable_index[i], Qii));
      }
      for (int j = i + 1; j < Q.cols(); j++) {
        const double Qij = 0.5 * (Q(i, j) + Q(j, i));
        if (abs(Qij) > sparseness_threshold) {
          Q_nonzero_coefs.push_back(Eigen::Triplet<double>(
              constraint_variable_index[i], constraint_variable_index[j], Qij));
        }
      }
    }

    for (int i = 0; i < b.size(); i++) {
      if (abs(b(i)) > sparseness_threshold) {
        b_nonzero_coefs.push_back(
            Eigen::Triplet<double>(constraint_variable_index[i], 0, b(i)));
      }
    }
  }

  // Add linear cost in prog.linear_costs() to the aggregated cost.
  for (const auto& binding : prog.linear_costs()) {
    const auto& constraint = binding.constraint();
    Eigen::RowVectorXd c = constraint->A();

    for (int i = 0; i < static_cast<int>(binding.GetNumElements()); ++i) {
      b_nonzero_coefs.push_back(Eigen::Triplet<double>(
          prog.FindDecisionVariableIndex(binding.variables()(i)), 0, c(i)));
    }
  }

  Eigen::SparseMatrix<double> Q_all(prog.num_vars(), prog.num_vars());
  Eigen::SparseMatrix<double> linear_terms(prog.num_vars(), 1);
  Q_all.setFromTriplets(Q_nonzero_coefs.begin(), Q_nonzero_coefs.end());
  linear_terms.setFromTriplets(b_nonzero_coefs.begin(), b_nonzero_coefs.end());

  std::vector<Eigen::Index> Q_all_row;
  std::vector<Eigen::Index> Q_all_col;
  std::vector<double> Q_all_val;
  drake::math::SparseMatrixToRowColumnValueVectors(Q_all, Q_all_row, Q_all_col,
                                                   Q_all_val);

  std::vector<int> Q_all_row_indices_int(Q_all_row.size());
  std::vector<int> Q_all_col_indices_int(Q_all_col.size());
  for (int i = 0; i < static_cast<int>(Q_all_row_indices_int.size()); i++) {
    Q_all_row_indices_int[i] = static_cast<int>(Q_all_row[i]);
    Q_all_col_indices_int[i] = static_cast<int>(Q_all_col[i]);
  }

  std::vector<Eigen::Index> linear_row;
  std::vector<Eigen::Index> linear_col;
  std::vector<double> linear_val;
  drake::math::SparseMatrixToRowColumnValueVectors(linear_terms, linear_row,
                                                   linear_col, linear_val);

  std::vector<int> linear_row_indices_int(linear_row.size());
  for (int i = 0; i < static_cast<int>(linear_row_indices_int.size()); i++) {
    linear_row_indices_int[i] = static_cast<int>(linear_row[i]);
  }

  const int QPtermsError = GRBaddqpterms(
      model, static_cast<int>(Q_all_row.size()), Q_all_row_indices_int.data(),
      Q_all_col_indices_int.data(), Q_all_val.data());
  if (QPtermsError) {
    return QPtermsError;
  }
  for (int i = 0; i < static_cast<int>(linear_row.size()); i++) {
    const int LinearTermError = GRBsetdblattrarray(
        model, "Obj", linear_row_indices_int[i], 1, linear_val.data() + i);
    if (LinearTermError) {
      return LinearTermError;
    }
  }
  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

// Add both LinearConstraints and LinearEqualityConstraints to gurobi
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
int ProcessLinearConstraints(GRBmodel* model, MathematicalProgram& prog,
                             double sparseness_threshold) {
  // TODO(naveenoid) : needs test coverage.
  for (const auto& binding : prog.linear_equality_constraints()) {
    const auto& constraint = binding.constraint();

    const int error = AddLinearConstraint(
        prog, model, constraint->A(), constraint->lower_bound(),
        constraint->upper_bound(), binding.variables(), true,
        sparseness_threshold);
    if (error) {
      return error;
    }
  }

  for (const auto& binding : prog.linear_constraints()) {
    const auto& constraint = binding.constraint();

    const int error = AddLinearConstraint(
        prog, model, constraint->A(), constraint->lower_bound(),
        constraint->upper_bound(), binding.variables(), false,
        sparseness_threshold);
    if (error) {
      return error;
    }
  }

  // If loop completes, no errors exist so the value '0' must be returned.
  return 0;
}

// For Lorentz and rotated Lorentz cone constraints
// Ax + b in (rotated) Lorentz cone, we will introduce new variables z as
// z = Ax+b
// z in (rotated) Lorentz cone.
// So add the new variable z before constructing the Gurobi model, as
// recommended by the Gurobi manual, to add all decision variables at once
// when constructing the problem.
// @param second_order_cone_variable_indices
// second_order_cone_variable_indices[i]
// contains the indices of the newly added variable z for the i'th second order
// cone in @p second_order_cones[i].
// @p tparam C Either LorentzConeConstraint or RotatedLorentzConeConstraint.
// TODO(hongkai.dai): rewrite this function not templated on Binding, when
// Binding class is moved out from MathematicalProgram as a public class.
// @param second_order_cones A vector of bindings, containing either Lorentz
// cone constraint, or rotated Lorentz cone constraint.
// @param is_new_variable is_new_variable[i] is true if the i'th variable in
// Gurobi model is not included in MathematicalProgram.
// @param num_gurobi_vars Number of variables in Gurobi model.
// @param second_order_cone_variable_indices
// second_order_cone_variable_indices[i]
// contains the indices of variable z stored in Gurobi model, in \p
// second_order_cones[i].
// @param gurobi_var_type. The type of the Gurobi variables.
// @param xlow The lower bound of the Gurobi variables.
// @param xupp The upper bound of the Gurobi variables.
template <typename C>
void AddSecondOrderConeVariables(
    const std::vector<Binding<C>>& second_order_cones,
    std::vector<bool>* is_new_variable, int* num_gurobi_vars,
    std::vector<std::vector<int>>* second_order_cone_variable_indices,
    std::vector<char>* gurobi_var_type, std::vector<double>* xlow,
    std::vector<double>* xupp) {
  static_assert(
      std::is_same<C, LorentzConeConstraint>::value ||
          std::is_same<C, RotatedLorentzConeConstraint>::value,
      "Expects LorentzConeConstraint and RotatedLorentzConeConstraint.");
  bool is_rotated_cone = std::is_same<C, RotatedLorentzConeConstraint>::value;

  int num_new_second_order_cone_var = 0;
  second_order_cone_variable_indices->resize(second_order_cones.size());

  // The newly added variable z for the Lorentz cone constraint is appended
  // to the existing variables. So increment the variable indices
  // accordingly.
  int lorentz_cone_count = 0;
  for (const auto& binding : second_order_cones) {
    int num_new_lorentz_cone_var_i = binding.constraint()->A().rows();
    (*second_order_cone_variable_indices)[lorentz_cone_count].resize(
        num_new_lorentz_cone_var_i);
    for (int i = 0; i < num_new_lorentz_cone_var_i; ++i) {
      (*second_order_cone_variable_indices)[lorentz_cone_count][i] =
          *num_gurobi_vars + num_new_second_order_cone_var + i;
    }
    num_new_second_order_cone_var += num_new_lorentz_cone_var_i;
    ++lorentz_cone_count;
  }
  *num_gurobi_vars += num_new_second_order_cone_var;
  is_new_variable->resize(*num_gurobi_vars, true);

  // Newly added variable z is continuous variable.
  gurobi_var_type->resize(*num_gurobi_vars, GRB_CONTINUOUS);

  // For Lorentz cone constraint, z(0) >= 0.
  // For rotated Lorentz cone constraint, z(0) >= 0, z(1) >= 0.
  xlow->resize(*num_gurobi_vars, -std::numeric_limits<double>::infinity());
  xupp->resize(*num_gurobi_vars, std::numeric_limits<double>::infinity());
  for (int i = 0; i < static_cast<int>(second_order_cones.size()); ++i) {
    xlow->at((*second_order_cone_variable_indices)[i][0]) = 0;
    if (is_rotated_cone) {
      xlow->at((*second_order_cone_variable_indices)[i][1]) = 0;
    }
  }
}
}  // close namespace

bool GurobiSolver::available() const { return true; }

SolutionResult GurobiSolver::Solve(MathematicalProgram& prog) const {
  // We only process quadratic costs and linear / bounding box
  // constraints.

  GRBenv* env = nullptr;
  GRBloadenv(&env, nullptr);

  DRAKE_ASSERT(prog.generic_costs().empty());
  DRAKE_ASSERT(prog.generic_constraints().empty());

  const int num_prog_vars = prog.num_vars();
  int num_gurobi_vars = num_prog_vars;

  // Potentially Gurobi can add variables on top of the variables in
  // MathematicalProgram prog.
  // is_new_variable[i] is true if the i'th variable in Gurobi environment is
  // not stored in MathematicalProgram, but added by the GurobiSolver.
  // For example, for Lorentz cone and rotated Lorentz cone constraint,to impose
  // that A*x+b lies in the (rotated) Lorentz cone, we add decision variable z
  // to Gurobi, defined as z = A*x + b.
  // The size of is_new_variable should increase if we add new decision
  // variables to Gurobi model.
  // The invariant is
  // EXPECT_TRUE(HasCorrectNumberOfVariables(model, is_new_variables.size()))
  std::vector<bool> is_new_variable(num_prog_vars, false);

  // Bound constraints.
  std::vector<double> xlow(num_prog_vars,
                           -std::numeric_limits<double>::infinity());
  std::vector<double> xupp(num_prog_vars,
                           std::numeric_limits<double>::infinity());

  const std::vector<MathematicalProgram::VarType>& var_type =
      prog.DecisionVariableTypes();

  std::vector<char> gurobi_var_type(num_prog_vars);
  for (int i = 0; i < num_prog_vars; ++i) {
    switch (var_type[i]) {
      case MathematicalProgram::VarType::CONTINUOUS:
        gurobi_var_type[i] = GRB_CONTINUOUS;
        break;
      case MathematicalProgram::VarType::BINARY:
        gurobi_var_type[i] = GRB_BINARY;
        break;
      case MathematicalProgram::VarType::INTEGER:
        gurobi_var_type[i] = GRB_INTEGER;
    }
  }

  for (const auto& binding : prog.bounding_box_constraints()) {
    const auto& constraint = binding.constraint();
    const Eigen::VectorXd& lower_bound = constraint->lower_bound();
    const Eigen::VectorXd& upper_bound = constraint->upper_bound();

    for (int k = 0; k < static_cast<int>(binding.GetNumElements()); ++k) {
      const int idx = prog.FindDecisionVariableIndex(binding.variables()(k));
      xlow[idx] = std::max(lower_bound(k), xlow[idx]);
      xupp[idx] = std::min(upper_bound(k), xupp[idx]);
    }
  }

  // Our second order cone constraints imposes A*x+b lies within the (rotated)
  // Lorentz cone. Unfortunately Gurobi only supports a vector z lying within
  // the (rotated) Lorentz cone. So we create new variable z, with the
  // constraint z - A*x = b and z being within the (rotated) Lorentz cone.
  // Here lorentz_cone_new_varaible_indices and
  // rotated_lorentz_cone_new_variable_indices
  // record the indices of the newly created variable z in the Gurobi program.
  std::vector<std::vector<int>> lorentz_cone_new_variable_indices;
  AddSecondOrderConeVariables(
      prog.lorentz_cone_constraints(), &is_new_variable, &num_gurobi_vars,
      &lorentz_cone_new_variable_indices, &gurobi_var_type, &xlow, &xupp);

  std::vector<std::vector<int>> rotated_lorentz_cone_new_variable_indices;
  AddSecondOrderConeVariables(prog.rotated_lorentz_cone_constraints(),
                              &is_new_variable, &num_gurobi_vars,
                              &rotated_lorentz_cone_new_variable_indices,
                              &gurobi_var_type, &xlow, &xupp);

  GRBmodel* model = nullptr;
  char** var_names = new char*[num_gurobi_vars];
  int prog_var_count = 0;
  for (int i = 0; i < num_gurobi_vars; ++i) {
    if (!is_new_variable[i]) {
      var_names[i] = const_cast<char*>(prog.decision_variable(prog_var_count).get_name().c_str());
      prog_var_count++;
    } else {
      var_names[i] = "slack";
    }
  }
  GRBnewmodel(env, &model, "gurobi_model", num_gurobi_vars, nullptr, &xlow[0],
              &xupp[0], gurobi_var_type.data(), nullptr);

  int error = 0;
  // TODO(naveenoid) : This needs access externally.
  double sparseness_threshold = 1e-14;
  error = AddCosts(model, prog, sparseness_threshold);
  DRAKE_DEMAND(!error);

  error = ProcessLinearConstraints(model, prog, sparseness_threshold);
  DRAKE_DEMAND(!error);

  // Add Lorentz cone constraints.
  error = AddSecondOrderConeConstraints(
      prog, prog.lorentz_cone_constraints(), sparseness_threshold,
      lorentz_cone_new_variable_indices, model);
  DRAKE_DEMAND(!error);

  // Add rotated Lorentz cone constraints.
  error = AddSecondOrderConeConstraints(
      prog, prog.rotated_lorentz_cone_constraints(), sparseness_threshold,
      rotated_lorentz_cone_new_variable_indices, model);
  DRAKE_DEMAND(!error);

  DRAKE_ASSERT(HasCorrectNumberOfVariables(model, is_new_variable.size()));

  // The new model gets a copy of the Gurobi environment, so when we set
  // parameters, we have to be sure to set them on the model's environment,
  // not the global gurobi environment.
  // See: FAQ #11: http://www.gurobi.com/support/faqs
  // Note that it is not necessary to free this environment; rather,
  // we just have to call GRBfreemodel(model).
  GRBenv* model_env = GRBgetenv(model);
  DRAKE_DEMAND(model_env);

  // Corresponds to no console or file logging (this is the default, which
  // can be overridden by parameters set in the MathematicalProgram).
  GRBsetintparam(model_env, GRB_INT_PAR_OUTPUTFLAG, 0);

  for (const auto it : prog.GetSolverOptionsDouble(SolverType::kGurobi)) {
    error = GRBsetdblparam(model_env, it.first.c_str(), it.second);
    DRAKE_DEMAND(!error);
  }


  for (const auto it : prog.GetSolverOptionsInt(SolverType::kGurobi)) {
    error = GRBsetintparam(model_env, it.first.c_str(), it.second);
    DRAKE_DEMAND(!error);
  }


  error = GRBoptimize(model);

  GRBwrite(model, "model1.mps");
  SolutionResult result = SolutionResult::kUnknownError;

  // If any error exists so far, it's from calling GRBoptimize.
  // TODO(naveenoid) : Properly handle gurobi specific error.
  // message.
  if (error) {
    // TODO(naveenoid) : log error message using GRBgeterrormsg(env).
    result = SolutionResult::kInvalidInput;
  } else {
    int optimstatus = 0;
    GRBgetintattr(model, GRB_INT_ATTR_STATUS, &optimstatus);

    if (optimstatus != GRB_OPTIMAL && optimstatus != GRB_SUBOPTIMAL) {
      switch (optimstatus) {
        case GRB_INF_OR_UNBD : {
          result = SolutionResult::kInfeasible_Or_Unbounded;
          break;
        }
        case GRB_UNBOUNDED : {
          result = SolutionResult::kUnbounded;
          break;
        }
        case GRB_INFEASIBLE : {
          result = SolutionResult::kInfeasibleConstraints;
          break;
        }
      }
      if (optimstatus == GRB_INFEASIBLE || optimstatus == GRB_INF_OR_UNBD) {
        error = GRBcomputeIIS(model);
        DRAKE_DEMAND(!error);
        GRBwrite(model, "model1.ilp");
        DRAKE_DEMAND(!error);
      }
    } else {
      result = SolutionResult::kSolutionFound;
      int num_total_variables = is_new_variable.size();
      // Gurobi has solved not only for the decision variables in
      // MathematicalProgram prog, but also for any extra decision variables
      // that this GurobiSolver injected to craft certain constraints, such as
      // Lorentz cones.  We therefore filter out the optimized values for
      // injected variables, and report back values for the MathematicalProgram
      // variables only.
      // solver_sol_vector includes the potentially newly added variables, i.e.,
      // variables not in MathematicalProgram prog, but added to Gurobi by
      // GurobiSolver.
      // prog_sol_vector only includes the original variables in
      // MathematicalProgram prog.
      std::vector<double> solver_sol_vector(num_total_variables);
      GRBgetdblattrarray(model, GRB_DBL_ATTR_X, 0, num_total_variables,
                         solver_sol_vector.data());
      Eigen::VectorXd prog_sol_vector(num_prog_vars);
      int prog_var_count = 0;
      for (int i = 0; i < num_total_variables; ++i) {
        if (!is_new_variable[i]) {
          prog_sol_vector(prog_var_count) = solver_sol_vector[i];
          ++prog_var_count;
        }
      }
      prog.SetDecisionVariableValues(prog_sol_vector);
    }
  }

  prog.SetSolverResult(solver_type(), error);

  GRBfreemodel(model);
  GRBfreeenv(env);
  delete[] var_names;
  return result;
}

}  // namespace solvers
}  // namespace drake
