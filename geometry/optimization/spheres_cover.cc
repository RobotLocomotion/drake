#include "drake/geometry/optimization/spheres_cover.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "drake/solvers/decision_variable.h"

namespace drake {
namespace geometry {
namespace optimization {
constexpr double kInf = std::numeric_limits<double>::infinity();

SpheresOuterApproximator::SpheresOuterApproximator(
    int num_spheres, std::vector<Eigen::MatrixXd> polytopes,
    const Eigen::Ref<const Eigen::MatrixXd>& outliers,
    std::optional<double> max_sphere_radius)
    : prog_{},
      dim_{static_cast<int>(outliers.rows())},
      polytopes_{std::move(polytopes)},
      outliers_{outliers} {
  spheres_.reserve(num_spheres);
  for (int i = 0; i < num_spheres; ++i) {
    auto a_i = prog_.NewContinuousVariables(dim_, "a");
    auto b_i = prog_.NewContinuousVariables<1>("b")[0];
    spheres_.push_back(SpheresOuterApproximator::Sphere{.a = a_i, .b = b_i});
  }
  const int num_polytopes = static_cast<int>(polytopes_.size());
  const int num_outliers = static_cast<int>(outliers_.cols());
  phi_ = prog_.NewBinaryVariables(num_polytopes, num_spheres, "phi");
  zeta_ = prog_.NewBinaryVariables(num_outliers, num_spheres, "zeta");

  // Each polytope is contained in at least one sphere.
  for (int i = 0; i < num_polytopes; ++i) {
    prog_.AddLinearConstraint(Eigen::VectorXd::Ones(num_spheres), 1, kInf,
                              phi_.row(i));
  }
  outlier_covered_ = prog_.NewBinaryVariables(num_outliers, "outlier_covered");
  // outlier_covered[i] = logical_or(zeta_(i, 0), zeta_(i, 1), ...,
  // zeta_(i, num_spheres-1)). We can write this logical_or operation with
  // linear constraints
  // outlier_covered_[i] >= zeta_(i, j) for all j
  // outlier_covered_[i] <= ∑ⱼ zeta_(i, j)
  Eigen::MatrixXd A_outlier_or =
      Eigen::MatrixXd::Zero(num_spheres + 1, num_spheres + 1);
  A_outlier_or.col(0) = Eigen::VectorXd::Ones(num_spheres + 1);
  A_outlier_or.topRightCorner(num_spheres, num_spheres) =
      -Eigen::MatrixXd::Identity(num_spheres, num_spheres);
  A_outlier_or.bottomRightCorner(1, num_spheres) =
      -Eigen::RowVectorXd::Ones(num_spheres);
  Eigen::VectorXd lb_outlier_or = Eigen::VectorXd::Zero(num_spheres + 1);
  lb_outlier_or(num_spheres) = -kInf;
  Eigen::VectorXd ub_outlier_or =
      Eigen::VectorXd::Constant(num_spheres + 1, kInf);
  ub_outlier_or(num_spheres) = 0;
  for (int i = 0; i < num_outliers; ++i) {
    solvers::VectorXDecisionVariable vars_outlier_or(num_spheres + 1);
    vars_outlier_or(0) = outlier_covered_(i);
    vars_outlier_or.tail(num_spheres) = zeta_.row(i);
    prog_.AddLinearConstraint(A_outlier_or, lb_outlier_or, ub_outlier_or,
                              vars_outlier_or);
  }

  // Add the cost to minimize the summation of outlier_covered_
  prog_.AddLinearCost(Eigen::VectorXd::Ones(num_outliers), 0, outlier_covered_);

  polytope_vertex_square_max_ = -kInf;
  for (const auto& polytope : polytopes_) {
    polytope_vertex_square_max_ =
        std::max(polytope_vertex_square_max_,
                 polytope.array().square().colwise().sum().maxCoeff());
  }
  AddPolytopeInSphereConstraint();

  const double vertex_pairwise_dist_squared_max =
      ComputePolytopePairwiseDistanceSquaredMaximal();
  // When the sphere center is within the convex hull of the polytopes, the
  // best sphere must have its radius smaller than the maximal pairwise
  // distance of the polytope vertices.
  const double max_sphere_radius_squared =
      max_sphere_radius.has_value()
          ? std::min(vertex_pairwise_dist_squared_max,
                     max_sphere_radius.value() * max_sphere_radius.value())
          : vertex_pairwise_dist_squared_max;

  for (int i = 0; i < num_spheres; ++i) {
    for (int j = 0; j < num_outliers; ++j) {
      AddOutlierNotInSphereConstraint(j, i, max_sphere_radius_squared);
    }
  }

  // Add constraint that the sphere radius should be less than
  // max_sphere_radius.
  // For a sphere xᵀx + aᵢᵀx + bᵢ ≤ 0, the radius square is
  // aᵢᵀaᵢ/4 − bᵢ
  // So we impose the second-order cone constraint 4(bᵢ + rₘₐₓ²) ≥ aᵢᵀaᵢ
  if (max_sphere_radius.has_value()) {
    for (int i = 0; i < num_spheres; ++i) {
      Eigen::MatrixXd A_rotated_lorentz =
          Eigen::MatrixXd::Zero(2 + dim_, dim_ + 1);
      A_rotated_lorentz(0, 0) = 1;
      A_rotated_lorentz.bottomRightCorner(dim_, dim_) =
          Eigen::MatrixXd::Identity(dim_, dim_);
      Eigen::VectorXd b_rotated_lorentz = Eigen::VectorXd::Zero(2 + dim_);
      b_rotated_lorentz(0) =
          max_sphere_radius.value() * max_sphere_radius.value();
      b_rotated_lorentz(1) = 4;
      VectorX<symbolic::Variable> vars_rotated_lorentz(dim_ + 1);
      vars_rotated_lorentz(0) = spheres_[i].b;
      vars_rotated_lorentz.tail(dim_) = spheres_[i].a;

      prog_.AddRotatedLorentzConeConstraint(
          A_rotated_lorentz, b_rotated_lorentz, vars_rotated_lorentz);
    }
  }
}

void SpheresOuterApproximator::AddPolytopeInSphereConstraint() {
  const int num_spheres = static_cast<int>(spheres_.size());
  const int num_polytopes = static_cast<int>(polytopes_.size());
  for (int i = 0; i < num_spheres; ++i) {
    for (int j = 0; j < num_polytopes; ++j) {
      for (int vert_idx = 0; vert_idx < polytopes_[j].cols(); ++vert_idx) {
        AddPointInSphereConstraint(polytopes_[j].col(vert_idx), i, phi_(j, i));
      }
    }
  }
  // Add the constraint that at least one of phi_(j, :) is 1, namely the
  // polytope has to be contained in at least one box.
  for (int j = 0; j < num_polytopes; ++j) {
    prog_.AddLinearConstraint(Eigen::RowVectorXd::Ones(num_spheres), 1, kInf,
                              phi_.row(j));
  }
}

void SpheresOuterApproximator::AddOutlierNotInSphereConstraint(
    int outlier_idx, int sphere_idx, double max_sphere_radius_squared) {
  // We add the constraint
  // qⱼᵀqⱼ + aᵢᵀqⱼ + bᵢ ≥ -M₁*ζ(j, i)
  // First we need to compute the big-M constant M₁, as a lower bound of the
  // left hand side. If the sphere is parameterized as (x-c)ᵀ(x-c) ≤ r², then we
  // know that aᵢ=−2c, bᵢ=cᵀc−r² where c is the center of the sphere and r is
  // the radius of the sphere. We can compute min aᵢᵀqⱼ = min -2cᵀqⱼ over c in
  // ConvexHull(polytopes). This minimization is obtained at one of the polytope
  // vertices. bᵢ is lower-bounded by −r².
  double a_dot_q_min = kInf;
  const auto& q = outliers_.col(outlier_idx);

  for (const auto& polytope : polytopes_) {
    a_dot_q_min = std::min(a_dot_q_min,
                           (-2 * q.transpose() * polytope).array().minCoeff());
  }
  const double M1 =
      std::max(-(q.squaredNorm() + a_dot_q_min - max_sphere_radius_squared),

               0.);
  // add the constraint qⱼᵀqⱼ + aᵢᵀqⱼ + bᵢ ≥ −M1 * ζ(j, i)
  Eigen::RowVectorXd coeff(dim_ + 2);
  coeff.head(dim_) = q.transpose();
  coeff(dim_) = 1;
  coeff(dim_ + 1) = M1;
  solvers::VectorXDecisionVariable vars(dim_ + 2);
  vars.head(dim_) = spheres_[sphere_idx].a;
  vars(dim_) = spheres_[sphere_idx].b;
  vars(dim_ + 1) = zeta_(outlier_idx, sphere_idx);
  prog_.AddLinearConstraint(coeff, -q.squaredNorm(), kInf, vars);

  // Add the constraint zeta = 1 => outlier is inside the sphere.
  AddPointInSphereConstraint(q, sphere_idx, zeta_(outlier_idx, sphere_idx));
}

void SpheresOuterApproximator::AddPointInSphereConstraint(
    const Eigen::Ref<const Eigen::VectorXd>& pt, int sphere_idx,
    const symbolic::Variable& binary_var) {
  // We add the constraint pᵀp + aᵀp + b ≤ M * (1-z)
  // First we need to compute the big constant M. To do so, we compute an upper
  // bound of the left hand side. If the sphere is parameterized as (x-c)ᵀ(x-c)
  // ≤ r², then we know that a=−2c, b=cᵀc−r² where c is the center of the
  // sphere and r is the radius of the sphere.
  // We know that the sphere center c should be in the convex hull of all the
  // polytopes, so max aᵀp is upper bounded by max -2cᵀp subject to c in
  // ConvexHull(P), and the maximal occurs at one of vertices of P. The maximal
  // of b is upper bounded by cᵀc, equal to polytope_vertex_square_max_.

  // Compute max aᵀp
  double a_dot_p_max = -kInf;
  for (const auto& polytope : polytopes_) {
    a_dot_p_max = std::max(a_dot_p_max,
                           (-2 * pt.transpose() * polytope).array().maxCoeff());
  }
  const double M = std::max(
      pt.squaredNorm() + a_dot_p_max + polytope_vertex_square_max_, 0.);
  // We add the constraint vᵀv + aᵀv + b ≤ M * (1-z)
  Eigen::RowVectorXd coeff(dim_ + 2);
  coeff.head(dim_) = pt.transpose();
  coeff(dim_) = 1;
  coeff(dim_ + 1) = M;
  solvers::VectorXDecisionVariable vars(dim_ + 2);
  vars.head(dim_) = spheres_[sphere_idx].a;
  vars(dim_) = spheres_[sphere_idx].b;
  vars(dim_ + 1) = binary_var;

  prog_.AddLinearConstraint(coeff, -kInf, M - pt.squaredNorm(), vars);
}

double SpheresOuterApproximator::ComputePolytopePairwiseDistanceSquaredMaximal()
    const {
  double squared_dist = 0;
  for (int i = 0; i < static_cast<int>(polytopes_.size()); ++i) {
    for (int j = 0; j < polytopes_[i].cols(); ++j) {
      if (j != polytopes_[i].cols() - 1) {
        // Compute the squared distance between polytopes_[i].col(j) and
        // polytopes_[i][:, j+1:]
        squared_dist = std::max(
            squared_dist,
            (polytopes_[i].rightCols(polytopes_[i].cols() - j - 1) -
             polytopes_[i].col(j).replicate(1, polytopes_[i].cols() - j - 1))
                .array()
                .square()
                .colwise()
                .sum()
                .maxCoeff());
      }
      for (int k = i + 1; k < static_cast<int>(polytopes_.size()); ++k) {
        // Compute the squared distance between polytopes_[i].col(j) and
        // polytopes_[k]
        squared_dist = std::max(squared_dist,
                                (polytopes_[k] - polytopes_[i].col(j).replicate(
                                                     1, polytopes_[k].cols()))
                                    .array()
                                    .square()
                                    .colwise()
                                    .sum()
                                    .maxCoeff());
      }
    }
  }
  return squared_dist;
}

void SpheresOuterApproximator::AddSphereCenterInPolytopeConvexHull() {
  // TODO(hongkai.dai): fill in this function.
  // First get all the unique vertices of all polytopes.
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
