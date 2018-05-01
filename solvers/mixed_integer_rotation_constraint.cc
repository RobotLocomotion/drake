#include "drake/solvers/mixed_integer_rotation_constraint.h"

#include "drake/solvers/mixed_integer_rotation_constraint_internal.h"
#include "drake/solvers/bilinear_product_util.h"

namespace drake {
namespace solvers {
namespace {
// Returns true if n is positive and n is a power of 2.
bool IsPowerOfTwo(int n) {
  DRAKE_ASSERT(n > 0);
  return (n & (n - 1)) == 0;
}

// Relax the unit length constraint x₀² + x₁² + x₂² = 1, to a set of linear
// constraints. The relaxation is achieved by assuming the intervals of xᵢ has
// been cut into smaller intervals in the form [φ(i) φ(i+1)], and xᵢ is
// constrained to be within one of the intervals. Namely we assume that
// xᵢ = φᵀ * λᵢ, where λ is the auxiliary variables, satisfying the SOS2
// constraint already.
// We know that due to the convexity of the curve w = x², we get
//     x² ≥ 2φⱼ*x-φⱼ²
// where the right hand-side of the inequality is the tangent of the curve
// w = x² at φⱼ. Thus we have 1 ≥ sum_j sum_i 2φⱼ*xᵢ-φⱼ²
// Moreover, also due to the convexity of the curve w = x², we know
//   xᵢ² ≤ sum_j φ(j)² * λᵢ(j)
// So we have the constraint
// 1 ≤ sum_i sum_j φ(j)² * λᵢ(j)
void AddUnitLengthConstraintWithSos2Lambda(
    MathematicalProgram* prog, const Eigen::Ref<const Eigen::VectorXd>& phi,
    const Eigen::Ref<const VectorXDecisionVariable>& lambda0,
    const Eigen::Ref<const VectorXDecisionVariable>& lambda1,
    const Eigen::Ref<const VectorXDecisionVariable>& lambda2) {
  const int num_phi = phi.rows();
  DRAKE_ASSERT(num_phi == lambda0.rows());
  DRAKE_ASSERT(num_phi == lambda1.rows());
  DRAKE_ASSERT(num_phi == lambda2.rows());
  const symbolic::Expression x0{phi.dot(lambda0.cast<symbolic::Expression>())};
  const symbolic::Expression x1{phi.dot(lambda1.cast<symbolic::Expression>())};
  const symbolic::Expression x2{phi.dot(lambda2.cast<symbolic::Expression>())};
  for (int phi0_idx = 0; phi0_idx < num_phi; phi0_idx++) {
    const symbolic::Expression x0_square_lb{2 * phi(phi0_idx) * x0 -
                                            pow(phi(phi0_idx), 2)};
    for (int phi1_idx = 0; phi1_idx < num_phi; phi1_idx++) {
      const symbolic::Expression x1_square_lb{2 * phi(phi1_idx) * x1 -
                                              pow(phi(phi1_idx), 2)};
      for (int phi2_idx = 0; phi2_idx < num_phi; phi2_idx++) {
        const symbolic::Expression x2_square_lb{2 * phi(phi2_idx) * x2 -
                                                pow(phi(phi2_idx), 2)};
        symbolic::Expression x_sum_of_squares_lb{x0_square_lb + x1_square_lb +
                                                 x2_square_lb};
        if (!is_constant(x_sum_of_squares_lb)) {
          prog->AddLinearConstraint(x_sum_of_squares_lb <= 1);
        }
      }
    }
  }
  symbolic::Expression x_square_ub{0};
  for (int i = 0; i < num_phi; ++i) {
    x_square_ub += phi(i) * phi(i) * (lambda0(i) + lambda1(i) + lambda2(i));
  }
  prog->AddLinearConstraint(x_square_ub >= 1);
}

std::pair<int, int> Index2Subscripts(int index, int num_rows, int num_cols) {
  DRAKE_ASSERT(index >= 0 && index < num_rows * num_cols);
  int column_index = index / num_rows;
  int row_index = index - column_index * num_rows;
  return std::make_pair(row_index, column_index);
}

// Relax the orthogonal constraint
// R.col(i)ᵀ * R.col(j) = 0
// R.row(i)ᵀ * R.row(j) = 0.
// and the cross product constraint
// R.col(i) x R.col(j) = R.col(k)
// R.row(i) x R.row(j) = R.row(k)
// To handle this non-convex bilinear product, we relax any bilinear product
// in the form x * y, we relax (x, y, w) to be in the convex hull of the
// curve w = x * y, and replace all the bilinear term x * y with w. For more
// details, @see AddBilinearProductMcCormickEnvelopeSos2.
void AddOrthogonalAndCrossProductConstraintRelaxationReplacingBilinearProduct(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    const Eigen::Ref<const Eigen::VectorXd>& phi,
    const std::array<std::array<VectorXDecisionVariable, 3>, 3>& B,
    const std::array<std::array<VectorXDecisionVariable, 3>, 3>& lambda,
    IntervalBinning interval_binning) {
  VectorDecisionVariable<9> R_flat;
  R_flat << R.col(0), R.col(1), R.col(2);
  MatrixDecisionVariable<9, 9> W;
  // We cannot call W.cast<symbolic::Expression>() directly, since the diagonal
  // entries in W is un-assigned, namely they are dummy variables. Converting a
  // dummy variable to a symbolic expression is illegal. So we create this new
  // W_expr, containing W(i, j) on the off-diagonal entries.
  Eigen::Matrix<symbolic::Expression, 9, 9> W_expr;
  W_expr.setZero();
  for (int i = 0; i < 9; ++i) {
    int Ri_row, Ri_col;
    std::tie(Ri_row, Ri_col) = Index2Subscripts(i, 3, 3);
    for (int j = i + 1; j < 9; ++j) {
      int Rj_row, Rj_col;
      std::tie(Rj_row, Rj_col) = Index2Subscripts(j, 3, 3);
      std::string W_ij_name =
          "R(" + std::to_string(Ri_row) + "," + std::to_string(Ri_col) +
          ")*R(" + std::to_string(Rj_row) + "," + std::to_string(Rj_col) + ")";
      W(i, j) = prog->NewContinuousVariables<1>(W_ij_name)(0);
      W_expr(i, j) = symbolic::Expression(W(i, j));

      auto lambda_bilinear = AddBilinearProductMcCormickEnvelopeSos2(
          prog, R(Ri_row, Ri_col), R(Rj_row, Rj_col), W(i, j), phi, phi,
          B[Ri_row][Ri_col].template cast<symbolic::Expression>(),
          B[Rj_row][Rj_col].template cast<symbolic::Expression>(),
          interval_binning);
      // Both sum_n lambda_bilinear(m, n) and sum_m lambda_bilinear(m, n)
      // satisfy the SOS2 constraint, and
      // R(Ri_row, Ri_col) = φᵀ * (sum_n lambda_bilinear(m, n))
      // R(Rj_row, Rj_col) = φᵀ * (sum_m lambda_bilinear(m, n).transpose())
      // Since we also know that both lambda[Ri_row][Ri_col] and
      // lambda[Rj_row][Rj_col] satisfy the SOS2 constraint, and
      // R[Ri_row][Ri_col] = φᵀ * lambda[Ri_row][Ri_col]
      // R[Rj_row][Rj_col] = φᵀ * lambda[Rj_row][Rj_col]
      // So sum_n lambda_bilinear(m, n) = lambda[Ri_row][Ri_col]
      //    sum_m lambda_bilinear(m, n).transpose() = lambda[Rj_row][Rj_col]
      // TODO(hongkai.dai): I found the computation could be faster if we
      // comment out the following two constraints on lambda_bilinear, at least
      // for some cases. Should investigate why there is a speed difference.
      prog->AddLinearConstraint(
          lambda_bilinear.template cast<symbolic::Expression>()
              .rowwise()
              .sum() == lambda[Ri_row][Ri_col]);
      prog->AddLinearConstraint(
          lambda_bilinear.template cast<symbolic::Expression>()
              .colwise()
              .sum()
              .transpose() == lambda[Rj_row][Rj_col]);
      W(j, i) = W(i, j);
      W_expr(j, i) = W_expr(i, j);
    }
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      // Orthogonal constraint between R.col(i), R.col(j).
      prog->AddLinearConstraint(
          ReplaceBilinearTerms(
              R.col(i).dot(R.col(j).cast<symbolic::Expression>()), R_flat,
              R_flat, W_expr) == 0);
      // Orthogonal constraint between R.row(i), R.row(j)
      prog->AddLinearConstraint(
          ReplaceBilinearTerms(
              R.row(i).transpose().dot(
                  R.row(j).cast<symbolic::Expression>().transpose()),
              R_flat, R_flat, W_expr) == 0);
    }
  }

  for (int i = 0; i < 3; ++i) {
    int j = (i + 1) % 3;
    int k = (i + 2) % 3;
    Vector3<symbolic::Expression> cross_product1 =
        R.col(i).cross(R.col(j).cast<symbolic::Expression>());
    Vector3<symbolic::Expression> cross_product2 = R.row(i).transpose().cross(
        R.row(j).transpose().cast<symbolic::Expression>());
    for (int row = 0; row < 3; ++row) {
      // R.col(i) x R.col(j) = R.col(k).
      prog->AddLinearConstraint(ReplaceBilinearTerms(cross_product1(row),
                                                     R_flat, R_flat,
                                                     W_expr) == R(row, k));
      // R.row(i) x R.row(j) = R.row(k).
      prog->AddLinearConstraint(ReplaceBilinearTerms(cross_product2(row),
                                                     R_flat, R_flat,
                                                     W_expr) == R(k, row));
    }
  }
}

/**
 * Add the constraint that vector R.col(i) and R.col(j) are not in the
 * same or opposite orthants. This constraint should be satisfied since
 * R.col(i) should be perpendicular to R.col(j). For example, if both
 * R.col(i) and R.col(j) are in the first orthant (+++), their inner product
 * has to be non-negative. If the inner product of two first orthant vectors
 * is exactly zero, then both vectors has to be on the boundaries of the first
 * orthant. But we can then assign the vector to a different orthant. The same
 * proof applies to the opposite orthant case.
 * To impose the constraint that R.col(0) and R.col(1) are not both in the first
 * orthant, we consider the constraint
 * Bpos0.col(0).sum() + Bpos0.col(1).sum() <= 5.
 * Namely, not all 6 entries in Bpos0.col(0) and Bpos0.col(1) can be 1 at the
 * same time, which is another way of saying R.col(0) and R.col(1) cannot be
 * both in the first orthant.
 * Similarly we can impose the constraint on the other orthant.
 * @param prog Add the constraint to this mathematical program.
 * @param Bpos0 Defined in AddRotationMatrixMcCormickEnvelopeMilpConstraints(),
 * Bpos0(i,j) = 1 => R(i, j) >= 0. Bpos0(i, j) = 0 => R(i, j) <= 0.
 */
void AddNotInSameOrOppositeOrthantConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& Bpos0) {
  const std::array<std::pair<int, int>, 3> column_idx = {
      {{0, 1}, {0, 2}, {1, 2}}};
  for (const auto& column_pair : column_idx) {
    const int col_idx0 = column_pair.first;
    const int col_idx1 = column_pair.second;
    for (int o = 0; o < 8; ++o) {
      // To enforce that R.col(i) and R.col(j) are not simultaneously in the
      // o'th orthant, we will impose the constraint
      // vars_same_orthant.sum() < = 5. The variables in vars_same_orthant
      // depend on the orthant number o.
      // To enforce that R.col(i) and R.col(j) are not in the opposite
      // orthants, we will impose the constraint
      // vars_oppo_orthant.sum() <= 5. The variables in vars_oppo_orthant
      // depnd on the orthant number o.
      Vector6<symbolic::Expression> vars_same_orthant;
      Vector6<symbolic::Expression> vars_oppo_orthant;
      for (int axis = 0; axis < 3; ++axis) {
        // axis chooses x, y, or z axis.
        if (o & (1 << axis)) {
          // If the orthant has positive value along the `axis`, then
          // `vars_same_orthant` choose the positive component Bpos0.
          vars_same_orthant(2 * axis) = Bpos0(axis, col_idx0);
          vars_same_orthant(2 * axis + 1) = Bpos0(axis, col_idx1);
          vars_oppo_orthant(2 * axis) = Bpos0(axis, col_idx0);
          vars_oppo_orthant(2 * axis + 1) = 1 - Bpos0(axis, col_idx1);
        } else {
          // If the orthant has negative value along the `axis`, then
          // `vars_same_orthant` choose the negative component 1 - Bpos0.
          vars_same_orthant(2 * axis) = 1 - Bpos0(axis, col_idx0);
          vars_same_orthant(2 * axis + 1) = 1 - Bpos0(axis, col_idx1);
          vars_oppo_orthant(2 * axis) = 1 - Bpos0(axis, col_idx0);
          vars_oppo_orthant(2 * axis + 1) = Bpos0(axis, col_idx1);
        }
      }
      prog->AddLinearConstraint(vars_same_orthant.sum() <= 5);
      prog->AddLinearConstraint(vars_oppo_orthant.sum() <= 5);
    }
  }
}

// For the cross product c = a x b, based on the sign of a and b, we can imply
// the sign of c for some cases. For example, c0 = a1 * b2 - a2 * b1, so if
// (a1, a2, b1, b2) has sign (+, -, +, +), then c0 has to have sign +.
// @param Bpos0. Bpos0(i, j) = 1 => R(i, j) ≥ 0, Bpos0(i, j) = 0 => R(i, j) ≤ 0
void AddCrossProductImpliedOrthantConstraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& Bpos0) {
  // The vertices of the polytope {x | A * x <= b} correspond to valid sign
  // assignment for (a1, b2, a2, b1, c0) in the example above.
  // Since R.col(k) = R.col(i) x R.col(j), we then know that
  // A * [Bpos0(1, i); Bpos0(2, j); Bpos0(1, j); Bpos0(2, i); Bpos0(0, k)] <= b.
  // The matrix A and b is found, by considering the polytope, whose vertices
  // are {0, 1}⁵, excluding the 8 points
  // (a1, b2, a2, b1, c0)
  // ( 0,  0,  1,  0,  0)
  // ( 0,  0,  0,  1,  0)
  // ( 1,  1,  1,  0,  0)
  // ( 1,  1,  0,  1,  0)
  // ( 1,  0,  0,  0,  1)
  // ( 1,  0,  1,  1,  1)
  // ( 0,  1,  0,  0,  1)
  // ( 0,  1,  1,  1,  1)
  // So this polytope has 2⁵ - 8 = 24 vertices in total.
  // The matrix A and b are obtained by converting this polytope from its
  // vertices (V-representation), to its facets (H-representation). We did
  // this conversion through Multi-parametric toolbox. Here is the MATLAB code
  // P = Polyhedron(V);
  // P.computeHRep();
  // A = P.A;
  // b = P.b;
  constexpr int A_rows = 18;
  constexpr int A_cols = 5;
  Eigen::Matrix<double, A_rows, A_cols> A;
  Eigen::Matrix<double, A_rows, 1> b;
  A << 0, 0, 0, 0, -1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 0, 0, -1, 0, 0, -1, 0,
      0, 0, 0, -1, -1, -1, 1, -1, -1, -1, 1, -1, -1, 0, -1, 0, 0, 0, 1, -1, -1,
      -1, 1, -1, 1, -1, -1, 1, 0, 0, 0, -1, 0, 1, -1, 1, 1, 1, -1, 1, 1, 1, 1,
      0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0;
  b << 0, 2, 2, 0, 0, 0, 0, 0, 1, 1, 0, 3, 3, 1, 1, 1, 1, 1;

  for (int col0 = 0; col0 < 3; ++col0) {
    int col1 = (col0 + 1) % 3;
    int col2 = (col1 + 1) % 3;
    // R(k, col2) = R(i, col0) * R(j, col1) - R(j, col0) * R(i, col1)
    // where (i, j, k) = (0, 1, 2), (1, 2, 0) or (2, 0, 1)
    VectorDecisionVariable<A_cols> var;
    for (int i = 0; i < 3; ++i) {
      int j = (i + 1) % 3;
      int k = (j + 1) % 3;
      var << Bpos0(i, col0), Bpos0(j, col1), Bpos0(j, col0), Bpos0(i, col1),
          Bpos0(k, col2);
      prog->AddLinearConstraint(A,
                                Eigen::Matrix<double, A_rows, 1>::Constant(
                                    -std::numeric_limits<double>::infinity()),
                                b, var);
    }
  }
}

void AddRotationMatrixBilinearMcCormickConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R,
    const std::array<std::array<VectorXDecisionVariable, 3>, 3>& B,
    const std::array<std::array<VectorXDecisionVariable, 3>, 3>& lambda,
    const Eigen::Ref<const Eigen::VectorXd>& phi,
    int num_intervals_per_half_axis, IntervalBinning interval_binning) {
  for (int row = 0; row < 3; ++row) {
    AddUnitLengthConstraintWithSos2Lambda(prog, phi, lambda[row][0],
                                          lambda[row][1], lambda[row][2]);
  }
  for (int col = 0; col < 3; ++col) {
    AddUnitLengthConstraintWithSos2Lambda(prog, phi, lambda[0][col],
                                          lambda[1][col], lambda[2][col]);
  }
  AddOrthogonalAndCrossProductConstraintRelaxationReplacingBilinearProduct(
      prog, R, phi, B, lambda, interval_binning);

  // Bpos(i, j) = 1 => R(i, j) >= 0
  // Bpos(i, j) = 0 => R(i, j) <= 0
  MatrixDecisionVariable<3, 3> Bpos;
  switch (interval_binning) {
    case IntervalBinning::kLogarithmic: {
      // If num_intervals_per_half_axis is a power of 2, then B[i][j](0)
      // indicates the sign of R(i, j).
      if (IsPowerOfTwo(num_intervals_per_half_axis)) {
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            Bpos(i, j) = B[i][j](0);
          }
        }
        AddNotInSameOrOppositeOrthantConstraint(prog, Bpos);
        AddNotInSameOrOppositeOrthantConstraint(prog, Bpos.transpose());

        AddCrossProductImpliedOrthantConstraint(prog, Bpos);
        AddCrossProductImpliedOrthantConstraint(prog, Bpos.transpose());

        // If num_intervals_per_half_axis is a power of 2, and it's >= 2, then
        // B[i][j](1) = 1 => -0.5 <= R(i, j) <= 0.5. Furthermore, we know for
        // each row/column of R, it cannot have all three entries in the
        // interval [-0.5, 0.5], since that would imply the norm of the
        // row/column being less than sqrt(3)/2. Thus, we have
        // sum_i B[i][j](1) <= 2 and sum_j B[i][j](1) <= 2
        if (num_intervals_per_half_axis >= 2) {
          for (int i = 0; i < 3; ++i) {
            symbolic::Expression row_sum{0};
            symbolic::Expression col_sum{0};
            for (int j = 0; j < 3; ++j) {
              row_sum += B[i][j](1);
              col_sum += B[j][i](1);
            }
            prog->AddLinearConstraint(row_sum <= 2);
            prog->AddLinearConstraint(col_sum <= 2);
          }
        }
      }
      break;
    }
    case IntervalBinning::kLinear: {
      // Bpos(i, j) = B[i][j](N) ∨ B[i][j](N+1) ∨ ... ∨ B[i][j](2*N-1)
      // where N = num_intervals_per_half_axis_;
      // This "logical or" constraint can be written as
      // Bpos(i, j) ≥ B[i][j](N + k) ∀ k = 0, ..., N-1
      // Bpos(i, j) ≤ ∑ₖ B[i][j](N+k)
      // 0 ≤ Bpos(i, j) ≤ 1
      Bpos = prog->NewContinuousVariables<3, 3>("Bpos");
      prog->AddBoundingBoxConstraint(0, 1, Bpos);
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          prog->AddLinearConstraint(Bpos(i, j) <=
                                    B[i][j].tail(num_intervals_per_half_axis)
                                        .cast<symbolic::Expression>()
                                        .sum());
          for (int k = 0; k < num_intervals_per_half_axis; ++k) {
            prog->AddLinearConstraint(
                Bpos(i, j) >= B[i][j](num_intervals_per_half_axis + k));
          }
        }
      }
      AddNotInSameOrOppositeOrthantConstraint(prog, Bpos);
      AddNotInSameOrOppositeOrthantConstraint(prog, Bpos.transpose());

      AddCrossProductImpliedOrthantConstraint(prog, Bpos);
      AddCrossProductImpliedOrthantConstraint(prog, Bpos.transpose());
      break;
    }
  }
}
}  // namespace

std::string to_string(MixedIntegerRotationConstraintType type) {
  switch (type) {
    case MixedIntegerRotationConstraintType::kBoxSphereIntersection: {
      return "box_sphere_intersection";
    }
    case MixedIntegerRotationConstraintType::kBilinearMcCormick: {
      return "bilinear_mccormick";
    }
    case MixedIntegerRotationConstraintType::kBoth: {
      return "both";
    }
  }
  // The following line should not be reached. We add it due to a compiler
  // defect.
  throw std::runtime_error("Should not reach this part of the code.\n");
}

std::ostream& operator<<(std::ostream& os,
                         const MixedIntegerRotationConstraintType& type) {
  os << to_string(type);
  return os;
}

template <MixedIntegerRotationConstraintType ConstraintType>
MixedIntegerRotationConstraintGenerator<ConstraintType>::
    MixedIntegerRotationConstraintGenerator(int num_intervals_per_half_axis,
                                            IntervalBinning interval_binning)
    : num_intervals_per_half_axis_(num_intervals_per_half_axis),
      interval_binning_(interval_binning),
      phi_nonnegative_{
          Eigen::VectorXd::LinSpaced(num_intervals_per_half_axis_ + 1, 0, 1)} {
  phi_.resize(2 * num_intervals_per_half_axis_ + 1);
  phi_(num_intervals_per_half_axis_) = 0;
  for (int i = 1; i <= num_intervals_per_half_axis_; ++i) {
    phi_(num_intervals_per_half_axis_ - i) = -phi_nonnegative_(i);
    phi_(num_intervals_per_half_axis_ + i) = phi_nonnegative_(i);
  }

  // If we consider the box-sphere intersection, then we need to compute the
  // halfspace nᵀx≥ d, as the tightest halfspace for each intersection region.
  if (ConstraintType ==
          MixedIntegerRotationConstraintType::kBoxSphereIntersection ||
      ConstraintType == MixedIntegerRotationConstraintType::kBoth) {
    const double kEpsilon = std::numeric_limits<double>::epsilon();

    box_sphere_intersection_vertices_.resize(num_intervals_per_half_axis_);
    box_sphere_intersection_halfspace_.resize(num_intervals_per_half_axis_);
    for (int xi = 0; xi < num_intervals_per_half_axis_; ++xi) {
      box_sphere_intersection_vertices_[xi].resize(
          num_intervals_per_half_axis_);
      box_sphere_intersection_halfspace_[xi].resize(
          num_intervals_per_half_axis_);
      for (int yi = 0; yi < num_intervals_per_half_axis_; ++yi) {
        box_sphere_intersection_vertices_[xi][yi].resize(
            num_intervals_per_half_axis_);
        box_sphere_intersection_halfspace_[xi][yi].resize(
            num_intervals_per_half_axis_);
        for (int zi = 0; zi < num_intervals_per_half_axis_; ++zi) {
          const Eigen::Vector3d box_min(
              phi_nonnegative_(xi), phi_nonnegative_(yi), phi_nonnegative_(zi));
          const Eigen::Vector3d box_max(phi_nonnegative_(xi + 1),
                                        phi_nonnegative_(yi + 1),
                                        phi_nonnegative_(zi + 1));
          const double box_min_norm = box_min.lpNorm<2>();
          const double box_max_norm = box_max.lpNorm<2>();
          if (box_min_norm <= 1.0 - kEpsilon &&
              box_max_norm >= 1.0 + kEpsilon) {
            // box_min is strictly inside the sphere, box_max is strictly
            // outside of the sphere.
            // We choose eps here, because if a vector x has unit length, and
            // another vector y is different from x by eps (||x - y||∞ < eps),
            // then max ||y||₂ - 1 is eps.
            box_sphere_intersection_vertices_[xi][yi][zi] =
                internal::ComputeBoxEdgesAndSphereIntersection(box_min,
                                                               box_max);
            DRAKE_DEMAND(box_sphere_intersection_vertices_[xi][yi][zi].size() >=
                         3);

            Eigen::Vector3d normal{};
            internal::ComputeHalfSpaceRelaxationForBoxSphereIntersection(
                box_sphere_intersection_vertices_[xi][yi][zi],
                &(box_sphere_intersection_halfspace_[xi][yi][zi].first),
                &(box_sphere_intersection_halfspace_[xi][yi][zi].second));
          } else if (std::abs(box_min_norm - 1) < kEpsilon) {
            // box_min is on the surface. This is the unique intersection point
            // between the sphere surface and the box.
            box_sphere_intersection_vertices_[xi][yi][zi].push_back(
                box_min / box_min_norm);
          } else if (std::abs(box_max_norm - 1) < kEpsilon) {
            // box_max is on the surface. This is the unique intersection point
            // between the sphere surface and the box.
            box_sphere_intersection_vertices_[xi][yi][zi].push_back(
                box_max / box_max_norm);
          }
        }
      }
    }
  }
}

template <MixedIntegerRotationConstraintType ConstraintType>
typename AddMixedIntegerRotationConstraintReturn<ConstraintType>::Type
MixedIntegerRotationConstraintGenerator<ConstraintType>::AddToProgram(
    MathematicalProgram* prog,
    const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) const {}

template <>
AddMixedIntegerRotationConstraintReturn<
    MixedIntegerRotationConstraintType::kBilinearMcCormick>::Type
MixedIntegerRotationConstraintGenerator<
    MixedIntegerRotationConstraintType::kBilinearMcCormick>::
    AddToProgram(
        MathematicalProgram* prog,
        const Eigen::Ref<const MatrixDecisionVariable<3, 3>>& R) const {
  std::array<std::array<VectorXDecisionVariable, 3>, 3> lambda;
  AddMixedIntegerRotationConstraintReturn<
      MixedIntegerRotationConstraintType::kBilinearMcCormick>::Type B;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const std::string lambda_name =
          "lambda[" + std::to_string(i) + "][" + std::to_string(j) + "]";
      lambda[i][j] = prog->NewContinuousVariables(
          2 * num_intervals_per_half_axis_ + 1, lambda_name);
      // R(i, j) = φᵀ * λ[i][j]
      prog->AddLinearConstraint(
          R(i, j) -
              phi_.dot(lambda[i][j].template cast<symbolic::Expression>()) ==
          0);
      switch (interval_binning_) {
        case IntervalBinning::kLogarithmic: {
          B[i][j] = AddLogarithmicSos2Constraint(
              prog, lambda[i][j].cast<symbolic::Expression>());
          break;
        }
        case IntervalBinning::kLinear: {
          B[i][j] = prog->NewBinaryVariables(
              2 * num_intervals_per_half_axis_,
              "B[" + std::to_string(i) + "][" + std::to_string(j) + "]");
          AddSos2Constraint(prog, lambda[i][j].cast<symbolic::Expression>(),
                            B[i][j].cast<symbolic::Expression>());
          break;
        }
      }
    }
  }

  AddRotationMatrixBilinearMcCormickConstraints(prog, R, B, lambda, phi_,
                                                num_intervals_per_half_axis_,
                                                interval_binning_);
  return B;
}

// Explicit instantiation
template class MixedIntegerRotationConstraintGenerator<
    MixedIntegerRotationConstraintType::kBilinearMcCormick>;
}  // namespace solvers
}  // namespace drake
