#include "drake/solvers/dual_convex_program.h"

#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "drake/common/fmt_eigen.h"
#include "drake/math/matrix_util.h"
#include "drake/solvers/aggregate_costs_constraints.h"
namespace drake {
namespace solvers {

namespace {

double kInf = std::numeric_limits<double>::infinity();

// If the program is compatible with conic standard forms.
void CheckSupported(const MathematicalProgram& prog) {
  std::string unsupported_message{};
  const ProgramAttributes supported_attributes(
      std::initializer_list<ProgramAttribute>{
          // Supported Constraints.
          ProgramAttribute::kLinearEqualityConstraint,
          ProgramAttribute::kLinearConstraint,
          ProgramAttribute::kLorentzConeConstraint,
          ProgramAttribute::kRotatedLorentzConeConstraint,
          ProgramAttribute::kPositiveSemidefiniteConstraint,
          // Supported Costs.
          ProgramAttribute::kLinearCost});
  if (!AreRequiredAttributesSupported(prog.required_capabilities(),
                                      supported_attributes,
                                      &unsupported_message)) {
    throw std::runtime_error(fmt::format(
        "CreateDualConvexProgram() does not (yet) support this program: {}.",
        unsupported_message));
  }
}
}  // namespace

std::unique_ptr<MathematicalProgram> CreateDualConvexProgram(
    const MathematicalProgram& prog,
    std::unordered_map<Binding<Constraint>, MatrixX<symbolic::Expression>>*
        constraint_to_dual_variable_map) {
  CheckSupported(prog);
  auto dual_prog = std::make_unique<MathematicalProgram>();

  internal::ConvexConstraintAggregationInfo info;
  internal::ConvexConstraintAggregationOptions options;
  // TODO(Alexandre.Amice) change to false.
  options.cast_rotated_lorentz_to_lorentz = true;
  options.preserve_psd_inner_product_vectorization = false;
  internal::DoAggregateConvexConstraints(prog, options, &info);
  Eigen::SparseMatrix<double> Aeq;
  Eigen::VectorXd beq;
  auto A_triplets_end_of_equalities_iterator = info.A_triplets.begin();
  std::advance(A_triplets_end_of_equalities_iterator,
               info.num_linear_equality_constraint_rows);

  // The first info.num_linear_equality_constraint_rows correspond to equality
  // constraints.
  Aeq.resize(info.num_linear_equality_constraint_rows, prog.num_vars());
  Aeq.setFromTriplets(info.A_triplets.begin(),
                      A_triplets_end_of_equalities_iterator);
  beq.resize(info.num_linear_equality_constraint_rows);
  beq = Eigen::Map<Eigen::VectorXd>(info.b_std.data(),
                                    info.num_linear_equality_constraint_rows);

  int num_conic_rows =
      info.A_row_count - info.num_linear_equality_constraint_rows;
  Eigen::VectorXd b;
  b.resize(num_conic_rows);
  b = Eigen::Map<Eigen::VectorXd>(
      info.b_std.data() + info.num_linear_equality_constraint_rows,
      num_conic_rows);

  std::vector<double> c_std(prog.num_vars(), 0);
  double d{0};
  VectorX<symbolic::Variable> vars{prog.decision_variables()};
  internal::ParseLinearCosts(prog, &c_std, &d);
  Eigen::VectorXd c = Eigen::Map<Eigen::VectorXd>(c_std.data(), c_std.size());

  // Our program is now collected into the form
  // min c^T x + d
  // s.t. -A x + b in K
  //      -Aeq x = beq
  // The dual of this program is
  // max -beqᵀy + d - bᵀλ
  // s.t. c+Aeqᵀy + Aᵀλ = 0
  // λ in K*
  // Since all the supported cones K are symmetric, then λ can be constrained in
  // the same cones as the primal variables.

  const auto y = dual_prog->NewContinuousVariables(
      info.num_linear_equality_constraint_rows, "y");
  const auto lam = dual_prog->NewContinuousVariables(num_conic_rows, "lambda");
  const VectorXDecisionVariable dual_vars = dual_prog->decision_variables();
  dual_prog->AddLinearCost(beq.transpose(), -d, y);
  dual_prog->AddLinearCost(b.transpose(), 0, lam);

  // The triplets returned by DoAggregateConvexConstraints represent the matrix
  // [beq - Aeq*x] = 0
  // [ b - A*x   ] in K
  // Therefore the dual subspace constraint can be written as
  // c + [Aeqᵀ, Aᵀ][y,λ] = 0. Which just requires transposing the triplets.
  std::vector<Eigen::Triplet<double>> A_transpose_triplets;
  A_transpose_triplets.reserve(info.A_triplets.size());
  for (const auto& triplet : info.A_triplets) {
    A_transpose_triplets.emplace_back(triplet.col(), triplet.row(),
                                      triplet.value());
  }
  Eigen::SparseMatrix<double> A_transpose(prog.num_vars(), info.A_row_count);
  A_transpose.setFromTriplets(A_transpose_triplets.begin(),
                              A_transpose_triplets.end());
  dual_prog->AddLinearEqualityConstraint(A_transpose, -c,
                                         dual_prog->decision_variables());
  int num_linear_constraints =
      info.num_bounding_box_inequality_constraint_rows +
      info.num_linear_constraint_rows;
  dual_prog->AddLinearConstraint(
      Eigen::MatrixXd::Identity(num_linear_constraints, num_linear_constraints),
      Eigen::VectorXd::Zero(num_linear_constraints),
      Eigen::VectorXd::Constant(num_linear_constraints, kInf),
      lam.head(num_linear_constraints));

  int current_dual_vars_start_index{0};
  // Now we populate the constraint_to_dual_variable_map
  // Linear Equality Constraints.
  for (int i = 0; i < ssize(prog.linear_equality_constraints()); i++) {
    constraint_to_dual_variable_map->emplace(
        prog.linear_equality_constraints()[i],
        dual_vars
            .segment(info.linear_eq_dual_variable_start_indices[i],
                     prog.linear_equality_constraints()[i]
                         .evaluator()
                         ->lower_bound()
                         .size())
            .cast<symbolic::Expression>());
  }
  current_dual_vars_start_index += info.num_linear_equality_constraint_rows;
  // Bounding Box Constraints.
  for (int i = 0; i < ssize(prog.bounding_box_constraints()); i++) {
    const int constraint_size =
        prog.bounding_box_constraints()[i].evaluator()->lower_bound().size();
    VectorX<symbolic::Expression> local_dual_var(constraint_size);
    for (int j = 0; j < constraint_size; j++) {
      if (info.bounding_box_constraint_dual_indices[i][j].first != -1 &&
          info.bounding_box_constraint_dual_indices[i][j].second != -1) {
        local_dual_var(j) = symbolic::Expression(0);
      }
      if (info.bounding_box_constraint_dual_indices[i][j].second != -1) {
        local_dual_var(j) =
            -dual_vars(info.bounding_box_constraint_dual_indices[i][j].second);
      }
      if (info.bounding_box_constraint_dual_indices[i][j].first != -1 &&
          info.bounding_box_constraint_dual_indices[i][j].first !=
              info.bounding_box_constraint_dual_indices[i][j].second) {
        local_dual_var(j) +=
            dual_vars(info.bounding_box_constraint_dual_indices[i][j].first);
      }
    }
    constraint_to_dual_variable_map->emplace(prog.bounding_box_constraints()[i],
                                             local_dual_var);
  }
  current_dual_vars_start_index +=
      info.num_bounding_box_inequality_constraint_rows;

  // Linear Constraints.
  for (int i = 0; i < ssize(info.linear_constraint_dual_indices); i++) {
    VectorX<symbolic::Expression> local_dual_vars = Eigen::VectorXd::Zero(
        prog.linear_constraints()[i].evaluator()->num_constraints());
    for (int j = 0; j < local_dual_vars.rows(); ++j) {
      if (info.linear_constraint_dual_indices[i][j].first != -1) {
        // lower bound is not infinity.
        local_dual_vars[j] +=
            dual_vars(info.linear_constraint_dual_indices[i][j].first);
      }
      if (info.linear_constraint_dual_indices[i][j].second != -1) {
        // upper bound is not infinity.
        local_dual_vars[j] -=
            dual_vars(info.linear_constraint_dual_indices[i][j].second);
      }
    }
    constraint_to_dual_variable_map->emplace(prog.linear_constraints()[i],
                                             local_dual_vars);
  }
  current_dual_vars_start_index += info.num_linear_constraint_rows;

  int second_order_cone_length_index = 0;
  // Lorentz Cone Constraints.
  for (int i = 0; i < ssize(info.lorentz_cone_dual_variable_start_indices);
       i++) {
    const int soc_length =
        info.second_order_cone_lengths[second_order_cone_length_index++];
    DRAKE_DEMAND(soc_length ==
                 prog.lorentz_cone_constraints()[i].evaluator()->A().rows());
    DRAKE_THROW_UNLESS(current_dual_vars_start_index ==
                       info.lorentz_cone_dual_variable_start_indices[i]);
    constraint_to_dual_variable_map->emplace(
        prog.lorentz_cone_constraints()[i],
        dual_vars.segment(info.lorentz_cone_dual_variable_start_indices[i],
                          soc_length));
    dual_prog->AddLorentzConeConstraint(
        Eigen::MatrixXd::Identity(soc_length, soc_length),
        Eigen::VectorXd::Zero(soc_length),
        dual_vars.segment(info.lorentz_cone_dual_variable_start_indices[i],
                          soc_length));
    current_dual_vars_start_index += soc_length;
  }

  // Rotated Lorentz Cone Constraints.
  for (int i = 0;
       i < ssize(info.rotated_lorentz_cone_dual_variable_start_indices); i++) {
    const int soc_length =
        info.second_order_cone_lengths[second_order_cone_length_index++];
    DRAKE_DEMAND(
        soc_length ==
        prog.rotated_lorentz_cone_constraints()[i].evaluator()->A().rows());
    DRAKE_THROW_UNLESS(
        current_dual_vars_start_index ==
        info.rotated_lorentz_cone_dual_variable_start_indices[i]);
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(soc_length, soc_length);
    Eigen::MatrixXd T_inv = Eigen::MatrixXd::Identity(soc_length, soc_length);
    //        const double inv_root_2 = 1 / sqrt(2.0);
    //    const double inv_root_2 = 1 / 2.0;
    //    const double inv_root_2 = 1;
    T(0, 0) = 1;
    T(0, 1) = 1;
    T(1, 0) = 1;
    T(1, 1) = -1;
    T_inv(0, 0) = 0.5;
    T_inv(0, 1) = 0.5;
    T_inv(1, 0) = 0.5;
    T_inv(1, 1) = -0.5;
    const VectorX<symbolic::Expression> tmp =
        T_inv *
        dual_vars
            .segment(info.rotated_lorentz_cone_dual_variable_start_indices[i],
                     soc_length)
            .cast<symbolic::Expression>();
    constraint_to_dual_variable_map->emplace(
        prog.rotated_lorentz_cone_constraints()[i], tmp);
    // The output of DoAggregateConvexConstraints casts rotated lorentz cones to
    // normal lorentz cones. Here we undo the transformation so that the dual
    // variables can be rotated lorentz cones.

    dual_prog->AddRotatedLorentzConeConstraint(
        T, Eigen::VectorXd::Zero(soc_length),
        dual_vars.segment(
            info.rotated_lorentz_cone_dual_variable_start_indices[i],
            soc_length));
    current_dual_vars_start_index += soc_length;
  }

  //  Positive semidefinite constraints.
  for (int i = 0; i < ssize(prog.positive_semidefinite_constraints()); ++i) {
    const int psd_row_size = info.psd_cone_lengths[i];
    const int lambda_rows_size = (psd_row_size * (psd_row_size + 1)) / 2;
    // Make Lam and add psd constraint
    const MatrixX<symbolic::Variable> Lam =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            dual_vars.segment(current_dual_vars_start_index, lambda_rows_size));
    // TODO(Alexandre.Amice) need to scale Lam appropriately.
    dual_prog->AddPositiveSemidefiniteConstraint(Lam);
    constraint_to_dual_variable_map->emplace(
        prog.positive_semidefinite_constraints()[i], Lam);
    current_dual_vars_start_index += lambda_rows_size;
  }
  for (int i = 0; i < ssize(prog.linear_matrix_inequality_constraints()); ++i) {
    const int psd_row_size =
        info.psd_cone_lengths[i +
                              ssize(prog.positive_semidefinite_constraints())];
    const int lambda_rows_size = (psd_row_size * (psd_row_size + 1)) / 2;
    // Make Lam and add psd constraint
    const MatrixX<symbolic::Expression> Lam =
        math::ToSymmetricMatrixFromLowerTriangularColumns(
            lam.segment(current_dual_vars_start_index, lambda_rows_size));
    dual_prog->AddPositiveSemidefiniteConstraint(Lam);
    constraint_to_dual_variable_map->emplace(
        prog.positive_semidefinite_constraints()[i], Lam);
    current_dual_vars_start_index += lambda_rows_size;
  }
  DRAKE_DEMAND(current_dual_vars_start_index == dual_vars.size());

  return dual_prog;
}

}  // namespace solvers
}  // namespace drake
