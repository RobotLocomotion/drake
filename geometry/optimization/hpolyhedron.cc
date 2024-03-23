#include "drake/geometry/optimization/hpolyhedron.h"

#include <algorithm>
#include <bitset>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>

#include <Eigen/Eigenvalues>
#include <drake/solvers/binding.h>
#include <fmt/format.h>
#include <libqhullcpp/Coordinates.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacet.h>
#include <libqhullcpp/QhullFacetList.h>

#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/aggregate_costs_constraints.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/get_program_type.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::LinearConstraint;
using solvers::LinearCost;
using solvers::MathematicalProgram;
using solvers::MatrixXDecisionVariable;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;

namespace {
const double kInf = std::numeric_limits<double>::infinity();

std::tuple<bool, solvers::MathematicalProgramResult> IsInfeasible(
    const MathematicalProgram& prog) {
  // Turn off Gurobi DualReduction to ensure that infeasible problems always
  // return solvers::SolutionResult::kInfeasibleConstraints rather than
  // SolutionResult::kInfeasibleOrUnbounded.
  solvers::SolverOptions solver_options;
  solver_options.SetOption(solvers::GurobiSolver::id(), "DualReductions", 0);
  auto result = solvers::Solve(prog, std::nullopt, solver_options);
  return {result.get_solution_result() ==
              solvers::SolutionResult::kInfeasibleConstraints,
          result};
}

/* Checks whether the constraint cᵀ x ≤ d is already implied by the linear
 constraints in prog. This is done by solving a small linear program
 and modifying the coefficients of  `new_constraint` binding. This method may
 throw a runtime error if the constraints are ill-conditioned.
 @param tol. We check if the prog already implies cᵀ x ≤ d + tol. If yes then we
 think this constraint cᵀ x ≤ d is redundant. Larger tol means that we are less
 strict on the containment.
 */
bool IsRedundant(const Eigen::Ref<const MatrixXd>& c, double d,
                 solvers::MathematicalProgram* prog,
                 Binding<solvers::LinearConstraint>* new_constraint,
                 Binding<solvers::LinearCost>* program_cost_binding,
                 double tol) {
  // Ensures that prog is an LP.
  DRAKE_DEMAND(prog->GetAllConstraints().size() ==
               prog->GetAllLinearConstraints().size());
  DRAKE_DEMAND(prog->GetAllCosts().size() == 1 &&
               prog->linear_costs()[0] == *program_cost_binding);

  // This inequality ensures a bounded objective since the left hand side of
  // the inequality is the same as the cost function on the next line.
  new_constraint->evaluator()->UpdateCoefficients(
      c, VectorXd::Constant(1, -kInf), VectorXd::Constant(1, d + 1));

  program_cost_binding->evaluator()->UpdateCoefficients(-c.transpose(), 0);
  // Constraints define an empty set or the current inequality of other is not
  // redundant. Since we tested whether this polyhedron is empty earlier, the
  // function would already have exited so this is proof that this inequality
  // is irredundant.
  auto [polyhedron_is_empty, result] = IsInfeasible(*prog);

  if (!polyhedron_is_empty && !result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to compute the set difference; it "
        "terminated with SolutionResult {}). This should only happen"
        "if the problem is ill-conditioned",
        result.get_solver_id().name(), result.get_solution_result()));
  }

  // If -result.get_optimal_cost() > other.b()(i) then the inequality is
  // irredundant. Without this constant
  // IrredundantBallIntersectionContainsBothOriginal fails.
  return !(polyhedron_is_empty || -result.get_optimal_cost() > d + tol);
}

/* Returns a trivially-infeasible HPolyhedron of a given dimension. */
HPolyhedron ConstructInfeasibleHPolyhedron(int dimension) {
  MatrixXd A_infeasible(2, dimension);
  A_infeasible.setZero();
  A_infeasible(0, 0) = 1;
  A_infeasible(1, 0) = -1;
  Eigen::Vector2d b_infeasible(-1, 0);
  return HPolyhedron(A_infeasible, b_infeasible);
}

}  // namespace

HPolyhedron::HPolyhedron() : ConvexSet(0, false) {}

HPolyhedron::HPolyhedron(const Eigen::Ref<const MatrixXd>& A,
                         const Eigen::Ref<const VectorXd>& b)
    : ConvexSet(A.cols(), false), A_(A), b_(b) {
  CheckInvariants();
}

HPolyhedron::HPolyhedron(const QueryObject<double>& query_object,
                         GeometryId geometry_id,
                         std::optional<FrameId> reference_frame)
    : ConvexSet(3, false) {
  std::pair<MatrixXd, VectorXd> Ab_G;
  query_object.inspector().GetShape(geometry_id).Reify(this, &Ab_G);

  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.InvertAndCompose(X_WE);
  // A_G*(p_GE + R_GE*p_EE_var) ≤ b_G
  A_ = Ab_G.first * X_GE.rotation().matrix();
  b_ = Ab_G.second - Ab_G.first * X_GE.translation();
}

HPolyhedron::HPolyhedron(const VPolytope& vpoly, double tol)
    : ConvexSet(vpoly.ambient_dimension(), false) {
  // First, handle the case where the VPolytope is empty.
  if (vpoly.IsEmpty()) {
    if (vpoly.ambient_dimension() == 0) {
      throw std::runtime_error(
          "Cannot convert an empty VPolytope with ambient dimension zero into "
          "a HPolyhedron.");
    } else {
      // Just create an infeasible HPolyhedron.
      *this = ConstructInfeasibleHPolyhedron(vpoly.ambient_dimension());
      return;
    }
  }
  if (vpoly.ambient_dimension() == 1) {
    // In 1D, QHull doesn't work. We can simply choose the largest and smallest
    // points, and add a hyperplane there.
    double min_val = vpoly.vertices().minCoeff();
    double max_val = vpoly.vertices().maxCoeff();
    Eigen::MatrixXd A(2, 1);
    Eigen::VectorXd b(2);
    // x <= max_val and x >= min_val (written as -x <= -min_val)
    A << 1, -1;
    b << max_val, -min_val;
    *this = HPolyhedron(A, b);
    return;
  }
  // Next, handle the case where the VPolytope is not full dimensional.
  const AffineSubspace affine_hull(vpoly, tol);
  if (affine_hull.AffineDimension() < affine_hull.ambient_dimension()) {
    // This special case avoids two QHull errors: QH6214 and QH6154.
    // QH6214 is due to the VPolytope not having enough vertices to make a
    // full dimensional simplex, and QH6154 is due to the VPolytope not being
    // full-dimensional, because all the vertices lie along a proper affine
    // subspace. We can handle both of these cases by projecting onto the
    // affine hull and doing the computations there. Then, we lift back to the
    // original space, and add hyperplanes to require the point lie on the
    // affine subspace.
    Eigen::MatrixXd points_local =
        affine_hull.ToLocalCoordinates(vpoly.vertices());

    // Note that QHull will not function in zero or one dimensional spaces, so
    // we handle these separately here.
    Eigen::MatrixXd global_A;
    Eigen::VectorXd global_b;
    if (affine_hull.AffineDimension() == 0) {
      // If it's just a point, then the orthogonal complement constraints
      // will do all the work, and we can set global_A and global_b to empty.
      global_A = Eigen::MatrixXd::Zero(0, ambient_dimension());
      global_b = Eigen::VectorXd::Zero(0);
    } else if (affine_hull.AffineDimension() == 1) {
      // In this case, we have a line, so we just pick the highest and lowest
      // points on that line (w.r.t. the direction of the one vector in the
      // affine basis) and put hyperplanes there.
      int min_idx, max_idx;
      Eigen::VectorXd point_local_vector = points_local.row(0);

      point_local_vector.minCoeff(&min_idx);
      point_local_vector.maxCoeff(&max_idx);
      Eigen::VectorXd lower_point = vpoly.vertices().col(min_idx);
      Eigen::VectorXd upper_point = vpoly.vertices().col(max_idx);
      Eigen::VectorXd direction = affine_hull.basis();

      global_A = Eigen::MatrixXd::Zero(2, ambient_dimension());
      global_A.row(0) = -direction;
      global_A.row(1) = direction;
      global_b = Eigen::VectorXd::Zero(2);
      global_b[0] = -lower_point.dot(direction);
      global_b[1] = upper_point.dot(direction);
    } else {
      // Construct a new VPolytope in the local coordinates, and then try
      // converting it to an HPolyhedron. This VPolytope is full dimensional
      // and has enough points for QHull, so in the subsequent call of the
      // VPolytope -> HPolyhedron constructor, none of this conditional block
      // is considered, and it goes directly to QHull. If QHull has additional
      // errors besides the dimension ones, they will be caught there.
      HPolyhedron hpoly_subspace(VPolytope{points_local});

      // Now, lift the HPolyhedron to the original ambient space. A point x in
      // R^n is projected to P(x-d), where d is the AffineSubspace translation,
      // and P is the projection matrix associated with the basis B (so that PB
      // is the identity matrix, mapping points in the local coordinates of the
      // AffineSubspace to themselves, where B is the matrix whose columns are
      // the basis vectors). Thus, an HPolyhedron in local coordinates
      // {Ay <= b : y in R^m} can be lifted to an HPolyhedron in global
      // coordinates by substituting P(x-d) for y. After simplification, we have
      // {APx <= b + APd : x in R^n}.

      // First, we obtain the matrix P. Because it's a linear operator, we can
      // compute its matrix by passing in the standard basis vectors (centered
      // at the translation of the AffineSubspace).
      Eigen::MatrixXd P_in =
          Eigen::MatrixXd::Identity(ambient_dimension(), ambient_dimension());
      Eigen::MatrixXd P = affine_hull.ToLocalCoordinates(
          P_in.colwise() + affine_hull.translation());

      // Now, we construct the global versions of A and b.
      global_A = hpoly_subspace.A() * P;
      global_b = hpoly_subspace.b() +
                 hpoly_subspace.A() * P * affine_hull.translation();
    }

    // Finally, we add additional constraints from the orthogonal complement
    // basis. This ensures that the points in the new HPolyhedron lie along the
    // affine hull of the VPolytope.
    Eigen::MatrixXd orthogonal_complement_basis =
        affine_hull.OrthogonalComplementBasis();
    Eigen::MatrixXd orth_A(2 * orthogonal_complement_basis.cols(),
                           ambient_dimension());
    orth_A << orthogonal_complement_basis.transpose(),
        -orthogonal_complement_basis.transpose();
    Eigen::VectorXd orth_b = orth_A * affine_hull.translation();

    Eigen::MatrixXd full_A(global_A.rows() + orth_A.rows(),
                           ambient_dimension());
    full_A << global_A, orth_A;
    Eigen::VectorXd full_b(global_b.size() + orth_b.size());
    full_b << global_b, orth_b;
    *this = HPolyhedron(full_A, full_b);
    return;
  }

  // Now that we know that the VPolytope is full dimensional, we can call QHull.
  orgQhull::Qhull qhull;
  qhull.runQhull("", vpoly.ambient_dimension(), vpoly.vertices().cols(),
                 vpoly.vertices().data(), "");
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull exited with status {} and  message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }
  A_.resize(qhull.facetCount(), ambient_dimension());
  b_.resize(qhull.facetCount());
  int facet_count = 0;
  for (const auto& facet : qhull.facetList()) {
    A_.row(facet_count) = Eigen::Map<Eigen::RowVectorXd>(
        facet.outerplane().coordinates(), facet.dimension());
    b_(facet_count) = -facet.outerplane().offset();
    ++facet_count;
  }
}

HPolyhedron::HPolyhedron(const MathematicalProgram& prog)
    : ConvexSet(prog.num_vars(), false) {
  // Preconditions
  DRAKE_THROW_UNLESS(prog.num_vars() > 0);
  DRAKE_THROW_UNLESS(prog.GetAllConstraints().size() > 0);
  DRAKE_THROW_UNLESS(solvers::GetProgramType(prog) ==
                     solvers::ProgramType::kLP);

  // Get linear equality constraints.
  std::vector<Eigen::Triplet<double>> A_triplets;
  std::vector<double> b;
  int A_row_count = 0;
  std::vector<int> unused_linear_eq_y_start_indices;
  int unused_num_rows_added = 0;
  solvers::internal::ParseLinearEqualityConstraints(
      prog, &A_triplets, &b, &A_row_count, &unused_linear_eq_y_start_indices,
      &unused_num_rows_added);

  // Linear equality constraints Ax = b are parsed in the form Ax <= b. So we
  // add in the reverse of the constraint, Ax >= b, encoded as -Ax <= -b. This
  // is implemented by taking each triplet (i, j, v) and adding in
  // (i + N, j, -v), where N is the number of rows of A before we start adding
  // new triplets.
  const int num_equality_triplets = A_triplets.size();
  const int& num_equality_constraints = A_row_count;
  for (int i = 0; i < num_equality_triplets; ++i) {
    const Eigen::Triplet<double>& triplet = A_triplets.at(i);
    A_triplets.emplace_back(triplet.row() + num_equality_constraints,
                            triplet.col(), -triplet.value());
  }
  // We also set b := [b; -b].
  for (int i = 0; i < num_equality_constraints; ++i) {
    b.push_back(-1 * b[i]);
  }
  A_row_count *= 2;

  // Get linear inequality constraints.
  std::vector<std::vector<std::pair<int, int>>>
      unused_linear_constraint_dual_indices;
  solvers::internal::ParseLinearConstraints(
      prog, &A_triplets, &b, &A_row_count,
      &unused_linear_constraint_dual_indices, &unused_num_rows_added);

  // Check that none of the linear inequality constraints are trivially
  // infeasible by requiring a variable be at least ∞ or at most -∞. If this is
  // the case, return a trivially infeasible HPolyhedron. Constraints that
  // violate this condition may not be returned by ParseLinearConstraints.
  for (const auto& binding : prog.linear_constraints()) {
    if (binding.evaluator()->lower_bound().maxCoeff() == kInf ||
        binding.evaluator()->upper_bound().minCoeff() == -kInf) {
      *this = ConstructInfeasibleHPolyhedron(prog.num_vars());
      return;
    }
  }

  // Get bounding box constraints.
  VectorXd lb;
  VectorXd ub;
  solvers::AggregateBoundingBoxConstraints(prog, &lb, &ub);

  for (int i = 0; i < lb.size(); ++i) {
    // If lb[i] == Infinity or ub[i] == -Infinity, we have a trivial
    // infeasibility, so we make a trivially infeasible HPolyhedron by just
    // including the constraints x <= -1, x >= 0, where x is the first decision
    // variable.
    if (lb[i] == kInf || ub[i] == -kInf) {
      *this = ConstructInfeasibleHPolyhedron(prog.num_vars());
      return;
    } else {
      A_triplets.emplace_back(A_row_count, i, 1);
      b.push_back(ub[i]);
      ++A_row_count;
      A_triplets.emplace_back(A_row_count, i, -1);
      b.push_back(-lb[i]);
      ++A_row_count;
    }
  }
  // Form the A and b matrices of the HPolyhedron.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_sparse(A_row_count,
                                                        prog.num_vars());
  A_sparse.setFromTriplets(A_triplets.begin(), A_triplets.end());

  // Identify the rows that do not contain any infinities, as the rows to keep.
  // Since we have filtered out infinities that render the problem infeasible
  // above, these correspond to inequalities that are trivially satisfied by all
  // vectors x, and need not be included in the HPolyhedron.
  std::vector<int> rows_to_keep;
  rows_to_keep.reserve(A_sparse.rows());
  for (int i = 0; i < ssize(b); ++i) {
    if (abs(b[i]) < kInf) {
      rows_to_keep.push_back(i);
    }
  }

  VectorXd b_eigen(b.size());
  b_eigen = VectorXd::Map(b.data(), b.size());

  *this = HPolyhedron(A_sparse.toDense()(rows_to_keep, Eigen::all),
                      b_eigen(rows_to_keep));
}

HPolyhedron::~HPolyhedron() = default;

Hyperellipsoid HPolyhedron::MaximumVolumeInscribedEllipsoid() const {
  MathematicalProgram prog;
  const int N = this->ambient_dimension();
  MatrixXDecisionVariable C = prog.NewSymmetricContinuousVariables(N, "C");
  VectorXDecisionVariable d = prog.NewContinuousVariables(N, "d");

  // Compute rowwise norms for later use in normalization of linear constraints.
  MatrixXd augmented_matrix(A_.rows(), A_.cols() + b_.cols());
  augmented_matrix << A_, b_;
  VectorXd row_norms = augmented_matrix.rowwise().norm();

  // max log det (C).  This method also imposes C ≽ 0.
  prog.AddMaximizeLogDeterminantCost(C.cast<Expression>());
  // |aᵢC|₂ ≤ bᵢ - aᵢd, ∀i
  // Add this as A_lorentz * vars + b_lorentz in the Lorentz cone constraint.
  // vars = [d; C.col(0); C.col(1); ...; C.col(n-1)]
  // A_lorentz = block_diagonal(-A_.row(i), A_.row(i), ..., A_.row(i))
  // b_lorentz = [b_(i); 0; ...; 0]
  // We also normalize each row, by dividing each instance of aᵢ and bᵢ with the
  // corresponding row norm.
  VectorX<symbolic::Variable> vars(C.rows() * C.cols() + d.rows());
  vars.head(d.rows()) = d;
  for (int i = 0; i < C.cols(); ++i) {
    vars.segment(d.rows() + i * C.rows(), C.rows()) = C.col(i);
  }

  Eigen::MatrixXd A_lorentz =
      Eigen::MatrixXd::Zero(1 + C.cols(), (1 + C.cols()) * C.rows());
  Eigen::VectorXd b_lorentz = Eigen::VectorXd::Zero(1 + C.cols());
  for (int i = 0; i < b_.size(); ++i) {
    A_lorentz.setZero();
    A_lorentz.block(0, 0, 1, A_.cols()) = -A_.row(i) / row_norms(i);
    for (int j = 0; j < C.cols(); ++j) {
      A_lorentz.block(j + 1, (j + 1) * C.cols(), 1, A_.cols()) =
          A_.row(i) / row_norms(i);
    }
    b_lorentz(0) = b_(i) / row_norms(i);
    prog.AddLorentzConeConstraint(A_lorentz, b_lorentz, vars);
  }
  auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the maximum inscribed ellipse problem; it "
        "terminated with SolutionResult {}). Make sure that your polyhedron is "
        "bounded and has an interior.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return Hyperellipsoid(result.GetSolution(C).inverse(), result.GetSolution(d));
}

VectorXd HPolyhedron::ChebyshevCenter() const {
  MathematicalProgram prog;
  VectorXDecisionVariable x = prog.NewContinuousVariables(ambient_dimension());
  VectorXDecisionVariable r = prog.NewContinuousVariables<1>("r");

  const double inf = std::numeric_limits<double>::infinity();
  // max r
  prog.AddLinearCost(Vector1d(-1.0), 0, r);

  // r ≥ 0.
  prog.AddBoundingBoxConstraint(0, inf, r);

  // aᵢᵀ x + |aᵢ| r ≤ bᵢ.
  RowVectorXd a(A_.cols() + 1);
  for (int i = 0; i < A_.rows(); ++i) {
    a[0] = A_.row(i).norm();
    a.tail(A_.cols()) = A_.row(i);
    prog.AddLinearConstraint(a, -inf, b_[i], {r, x});
  }

  auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the Chebyshev center problem; it terminated "
        "with SolutionResult {}). Make sure that your polyhedron is bounded "
        "and has an interior.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return result.GetSolution(x);
}

HPolyhedron HPolyhedron::Scale(double scale,
                               std::optional<Eigen::VectorXd> center) const {
  DRAKE_THROW_UNLESS(scale >= 0.0);
  if (center) {
    DRAKE_THROW_UNLESS(center->size() == ambient_dimension());
  } else {
    center = ChebyshevCenter();
  }
  return HPolyhedron(
      A_, std::pow(scale, 1.0 / ambient_dimension()) * (b_ - A_ * *center) +
              A_ * *center);
}

HPolyhedron HPolyhedron::CartesianProduct(const HPolyhedron& other) const {
  MatrixXd A_product = MatrixXd::Zero(A_.rows() + other.A().rows(),
                                      A_.cols() + other.A().cols());
  A_product.topLeftCorner(A_.rows(), A_.cols()) = A_;
  A_product.bottomRightCorner(other.A().rows(), other.A().cols()) = other.A();
  VectorXd b_product(b_.size() + other.b().size());
  b_product << b_, other.b();
  return {A_product, b_product};
}

HPolyhedron HPolyhedron::CartesianPower(int n) const {
  MatrixXd A_power = MatrixXd::Zero(n * A_.rows(), n * A_.cols());
  for (int i{0}; i < n; ++i) {
    A_power.block(i * A_.rows(), i * A_.cols(), A_.rows(), A_.cols()) = A_;
  }
  VectorXd b_power = b_.replicate(n, 1);
  return {A_power, b_power};
}

HPolyhedron HPolyhedron::Intersection(const HPolyhedron& other,
                                      bool check_for_redundancy,
                                      double tol) const {
  DRAKE_THROW_UNLESS(ambient_dimension() == other.ambient_dimension());
  if (check_for_redundancy) {
    return this->DoIntersectionWithChecks(other, tol);
  }
  return this->DoIntersectionNoChecks(other);
}

VectorXd HPolyhedron::UniformSample(
    RandomGenerator* generator,
    const Eigen::Ref<const Eigen::VectorXd>& previous_sample,
    const int mixing_steps) const {
  DRAKE_THROW_UNLESS(mixing_steps >= 1);

  std::normal_distribution<double> gaussian;
  VectorXd direction(ambient_dimension());
  VectorXd current_sample = previous_sample;

  for (int step = 0; step < mixing_steps; ++step) {
    // Choose a random direction.
    for (int i = 0; i < direction.size(); ++i) {
      direction[i] = gaussian(*generator);
    }
    // Find max and min θ subject to
    //   A(previous_sample + θ*direction) ≤ b,
    // aka ∀i, θ * (A * direction)[i] ≤ (b - A * previous_sample)[i].
    VectorXd line_b = b_ - A_ * current_sample;
    VectorXd line_a = A_ * direction;
    double theta_max = std::numeric_limits<double>::infinity();
    double theta_min = -theta_max;
    for (int i = 0; i < line_a.size(); ++i) {
      if (line_a[i] < 0.0) {
        theta_min = std::max(theta_min, line_b[i] / line_a[i]);
      } else if (line_a[i] > 0.0) {
        theta_max = std::min(theta_max, line_b[i] / line_a[i]);
      }
    }
    if (std::isinf(theta_max) || std::isinf(theta_min) ||
        theta_max < theta_min) {
      throw std::invalid_argument(fmt::format(
          "The Hit and Run algorithm failed to find a feasible point in the "
          "set. The `previous_sample` must be in the set.\n"
          "max(A * previous_sample - b) = {}",
          (A_ * current_sample - b_).maxCoeff()));
    }
    // Now pick θ uniformly from [θ_min, θ_max).
    std::uniform_real_distribution<double> uniform_theta(theta_min, theta_max);
    const double theta = uniform_theta(*generator);
    current_sample = current_sample + theta * direction;
  }
  // The new sample is previous_sample + θ * direction.
  return current_sample;
}

// Note: This method only exists to effectively provide ChebyshevCenter(),
// which is a non-static class method, as a default argument for
// previous_sample in the UniformSample method above.
VectorXd HPolyhedron::UniformSample(RandomGenerator* generator,
                                    const int mixing_steps) const {
  VectorXd center = ChebyshevCenter();
  return UniformSample(generator, center, mixing_steps);
}

HPolyhedron HPolyhedron::MakeBox(const Eigen::Ref<const VectorXd>& lb,
                                 const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == ub.size());
  DRAKE_THROW_UNLESS((lb.array() <= ub.array()).all());
  const int N = lb.size();
  MatrixXd A(2 * N, N);
  A << MatrixXd::Identity(N, N), -MatrixXd::Identity(N, N);
  VectorXd b(2 * N);
  b << ub, -lb;
  return HPolyhedron(A, b);
}

HPolyhedron HPolyhedron::MakeUnitBox(int dim) {
  return MakeBox(VectorXd::Constant(dim, -1.0), VectorXd::Constant(dim, 1.0));
}

HPolyhedron HPolyhedron::MakeL1Ball(const int dim) {
  DRAKE_THROW_UNLESS(dim > 0);
  const int size{static_cast<int>(std::pow(2, dim))};
  MatrixXd A = MatrixXd::Ones(size, dim);
  VectorXd b = VectorXd::Ones(size);
  // L1Ball is constructed by iterating over all permutations of {± 1}ᵈⁱᵐ.
  constexpr int bit_set_size = 8 * sizeof(int);
  for (int row = 0; row < A.rows(); ++row) {
    const std::bitset<bit_set_size> row_bits(row);
    for (int col = 0; col < A.cols(); ++col) {
      A(row, col) = row_bits[col] ? -1 : 1;
    }
  }
  return {A, b};
}

std::optional<bool> HPolyhedron::DoIsBoundedShortcut() const {
  if (A_.rows() < A_.cols()) {
    return false;
  }
  Eigen::ColPivHouseholderQR<MatrixXd> qr(A_);
  if (qr.dimensionOfKernel() > 0) {
    return false;
  }
  // Stiemke's theorem of alternatives says that, given A with ker(A) = {0}, we
  // either have existence of x ≠ 0 such that Ax ≥ 0 or we have existence of y
  // > 0 such that y^T A = 0.  Since any y that verifies the second condition
  // can be arbitrarily scaled, and would still pass the second condition,
  // instead of asking y > 0, we can equivalently ask y ≥ 1.  So boundedness
  // corresponds to the following LP being feasible: find y s.t. y ≥ 1, y^T A =
  // 0.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables(A_.rows(), "y");
  prog.AddBoundingBoxConstraint(1.0, std::numeric_limits<double>::infinity(),
                                y);
  prog.AddLinearEqualityConstraint(A_.transpose(), VectorXd::Zero(A_.cols()),
                                   y);
  auto result = solvers::Solve(prog);
  return result.is_success();
}

bool HPolyhedron::DoIsEmpty() const {
  if (ambient_dimension() == 0) {
    return false;
  }
  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(A_.cols(), "x");
  prog.AddLinearConstraint(A_, VectorXd::Constant(b_.rows(), -kInf), b_, x);
  return std::get<0>(IsInfeasible(prog));
}

bool HPolyhedron::ContainedIn(const HPolyhedron& other, double tol) const {
  DRAKE_THROW_UNLESS(other.A().cols() == A_.cols());
  // `this` defines an empty set and therefore is contained in any `other`
  // HPolyhedron.
  if (DoIsEmpty()) {
    return true;
  }

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(A_.cols(), "x");
  prog.AddLinearConstraint(A_, VectorXd::Constant(b_.rows(), -kInf), b_, x);

  Binding<solvers::LinearConstraint> redundant_constraint_binding =
      prog.AddLinearConstraint(other.A().row(0), VectorXd::Constant(1, -kInf),
                               other.b().row(0), x);
  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(-other.A().row(0), 0, x);

  for (int i = 0; i < other.A().rows(); ++i) {
    // If any of the constraints of `other` are irredundant then `this` is
    // not contained in `other`.
    if (!IsRedundant(other.A().row(i), other.b()(i), &prog,
                     &redundant_constraint_binding, &program_cost_binding,
                     tol)) {
      return false;
    }
  }
  return true;
}

HPolyhedron HPolyhedron::DoIntersectionNoChecks(
    const HPolyhedron& other) const {
  MatrixXd A_intersect =
      MatrixXd::Zero(A_.rows() + other.A().rows(), A_.cols());
  A_intersect.topRows(A_.rows()) = A_;
  A_intersect.bottomRows(other.A().rows()) = other.A();
  VectorXd b_intersect(b_.size() + other.b().size());
  b_intersect << b_, other.b();
  return {A_intersect, b_intersect};
}

HPolyhedron HPolyhedron::DoIntersectionWithChecks(const HPolyhedron& other,
                                                  double tol) const {
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      A_.rows() + other.A().rows(), A_.cols());
  VectorXd b(A_.rows() + other.A().rows());
  A.topRows(A_.rows()) = A_;
  b.head(A_.rows()) = b_;

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(A_.cols(), "x");
  prog.AddLinearConstraint(A_, VectorXd::Constant(b_.rows(), -kInf), b_, x);
  auto [infeasible, result] = IsInfeasible(prog);

  // `this` defines an empty set therefore any additional constraint is
  // redundant.
  if (infeasible) {
    return {A_, b_};
  }

  Binding<solvers::LinearConstraint> redundant_constraint_binding =
      prog.AddLinearConstraint(other.A().row(0), VectorXd::Constant(1, -kInf),
                               other.b().row(0), x);
  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(-other.A().row(0), 0, x);

  int num_kept = A_.rows();
  for (int i = 0; i < other.A().rows(); ++i) {
    if (!IsRedundant(other.A().row(i), other.b()(i), &prog,
                     &redundant_constraint_binding, &program_cost_binding,
                     tol)) {
      A.row(num_kept) = other.A().row(i);
      b.row(num_kept) = other.b().row(i);
      ++num_kept;
      HPolyhedron maybe_empty(A.topRows(num_kept), b.topRows(num_kept));
      if (maybe_empty.IsEmpty()) {
        return maybe_empty;
      }
    }
  }
  return {A.topRows(num_kept), b.topRows(num_kept)};
}

HPolyhedron HPolyhedron::ReduceInequalities(double tol) const {
  const std::set<int> redundant_indices = FindRedundant(tol);
  const int num_vars = A_.cols();

  MatrixXd A_new(A_.rows() - redundant_indices.size(), num_vars);
  VectorXd b_new(A_new.rows());
  int i = 0;
  for (int j = 0; j < A_.rows(); ++j) {
    if (!redundant_indices.contains(j)) {
      A_new.row(i) = A_.row(j);
      b_new.row(i) = b_.row(j);
      ++i;
    }
  }
  return {A_new, b_new};
}

std::set<int> HPolyhedron::FindRedundant(double tol) const {
  // This method is based on removing each constraint and solving the resulting
  // LP. If the optimal cost is greater than the constraint's right hand side,
  // then the constraint is redundant.
  // Possible future speed up: solve duals instead of primal in parallel,
  // however this would require building num_threads mathematical programs and
  // may not be worth it.
  std::set<int> redundant_indices;
  MathematicalProgram prog;
  const int num_vars = A_.cols();
  const int num_cons = A_.rows();
  const auto x = prog.NewContinuousVariables(num_vars, "x");
  std::vector<Binding<LinearConstraint>> bindings_vec;
  for (int i = 0; i < num_cons; ++i) {
    bindings_vec.push_back(prog.AddLinearConstraint(
        A_.row(i), -std::numeric_limits<double>::infinity(), b_[i], x));
  }
  auto const_binding = prog.AddLinearCost(-A_.row(0), 0, x);
  for (int i = 0; i < num_cons; ++i) {
    prog.RemoveConstraint(bindings_vec.at(i));
    const_binding.evaluator()->UpdateCoefficients(-A_.row(i), 0);
    const auto result = Solve(prog);
    if ((result.is_success() && -result.get_optimal_cost() > b_[i] + tol) ||
        !result.is_success()) {
      // Bring back the constraint, it is not redundant.
      prog.AddConstraint(bindings_vec.at(i));
    } else {
      redundant_indices.insert(i);
    }
  }
  return redundant_indices;
}

std::unique_ptr<ConvexSet> HPolyhedron::DoClone() const {
  return std::make_unique<HPolyhedron>(*this);
}

bool HPolyhedron::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                               double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  return ((A_ * x).array() <= b_.array() + tol).all();
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
HPolyhedron::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  VectorX<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  new_constraints.push_back(prog->AddLinearConstraint(
      A_,
      VectorXd::Constant(b_.size(), -std::numeric_limits<double>::infinity()),
      b_, vars));
  return {std::move(new_vars), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
HPolyhedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A x ≤ t b, written as [A,-b][x;t] ≤ 0
  const int m = A_.rows();
  const int n = ambient_dimension();
  MatrixXd Abar(m, n + 1);
  Abar.leftCols(n) = A_;
  Abar.col(n) = -b_;
  constraints.emplace_back(prog->AddLinearConstraint(
      Abar, VectorXd::Constant(m, -std::numeric_limits<double>::infinity()),
      VectorXd::Zero(m), {x, Vector1<Variable>(t)}));
  return constraints;
}

std::vector<Binding<Constraint>>
HPolyhedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A_x,
    const Eigen::Ref<const VectorXd>& b_x, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A (A_x x + b_x) ≤ (c' t + d) b, written as [A * A_x, -b * c'][x;t] ≤ d * b
  // - A * b_x
  const int m = A_.rows();
  MatrixXd A_bar(m, x.size() + t.size());
  A_bar.leftCols(x.size()) = A_ * A_x;
  A_bar.rightCols(t.size()) = -b_ * c.transpose();
  VectorXd b_bar = d * b_ - A_ * b_x;
  constraints.emplace_back(prog->AddLinearConstraint(
      A_bar, VectorXd::Constant(m, -std::numeric_limits<double>::infinity()),
      b_bar, {x, t}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
HPolyhedron::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for HPolyhedron.  Implementing "
      "this will likely require additional support from the Convex shape "
      "class (to support in-memory mesh data, or file I/O).");
}

void HPolyhedron::ImplementGeometry(const Box& box, void* data) {
  Eigen::Matrix<double, 6, 3> A;
  A << Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
  Vector6d b;
  // clang-format off
  b << box.width()/2.0, box.depth()/2.0, box.height()/2.0,
        box.width()/2.0, box.depth()/2.0, box.height()/2.0;
  // clang-format on
  auto* Ab = static_cast<std::pair<MatrixXd, VectorXd>*>(data);
  Ab->first = A;
  Ab->second = b;
}

void HPolyhedron::ImplementGeometry(const HalfSpace&, void* data) {
  auto* Ab = static_cast<std::pair<MatrixXd, VectorXd>*>(data);
  // z <= 0.0.
  Ab->first = Eigen::RowVector3d{0.0, 0.0, 1.0};
  Ab->second = Vector1d{0.0};
}

HPolyhedron HPolyhedron::PontryaginDifference(const HPolyhedron& other) const {
  /**
   * The Pontryagin set difference of Polytope P = {x | Ax <= b} and
   * Q = {x | Cx <= d} can be computed P - Q = {x | Ax <= b - h}
   * where hᵢ = max aᵢᵀx subject to x ∈ Q
   */

  DRAKE_THROW_UNLESS(this->ambient_dimension() == other.ambient_dimension());
  DRAKE_THROW_UNLESS(this->IsBounded());
  DRAKE_THROW_UNLESS(other.IsBounded());

  VectorXd b_diff(b_.rows());
  MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(ambient_dimension(), "x");
  // -inf <= Ax <= b
  prog.AddLinearConstraint(
      other.A(), VectorXd::Constant(other.b().rows(), -kInf), other.b(), x);

  auto result = solvers::Solve(prog);
  // other is an empty polyhedron and so Pontryagin difference does nothing
  if (result.get_solution_result() ==
      solvers::SolutionResult::kInfeasibleConstraints) {
    return {A_, b_};
  }

  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(A_.row(0), 0, x);
  for (int i = 0; i < b_.rows(); ++i) {
    program_cost_binding.evaluator()->UpdateCoefficients(-A_.row(i), 0);
    result = solvers::Solve(prog);
    // since constraint set is bounded and non-empty then the program should
    // always have an optimal solution
    if (!result.is_success()) {
      throw std::runtime_error(fmt::format(
          "Solver {} failed to compute the set difference; it "
          "terminated with SolutionResult {}). This should only happen"
          "if the problem is ill-conditioned",
          result.get_solver_id().name(), result.get_solution_result()));
    }
    b_diff(i) = b_(i) + result.get_optimal_cost();
  }
  return {A_, b_diff};
}

void HPolyhedron::CheckInvariants() const {
  DRAKE_THROW_UNLESS(this->ambient_dimension() == A_.cols());
  DRAKE_THROW_UNLESS(A_.rows() == b_.size());
  // Note: If necessary, we could support infinite b, either by removing the
  // corresponding rows of A (since the constraint is vacuous), or checking
  // this explicitly in all relevant computations (like IsBounded).
  DRAKE_THROW_UNLESS(b_.array().isFinite().all());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
