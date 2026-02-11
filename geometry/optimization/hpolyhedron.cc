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

#include "drake/common/overloaded.h"
#include "drake/common/text_logging.h"
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
  const Shape& shape = query_object.inspector().GetShape(geometry_id);
  MatrixXd A_G;
  VectorXd b_G;
  std::tie(A_G, b_G) = shape.Visit<std::pair<MatrixXd, VectorXd>>(overloaded{
      // We only handle certain shape types.
      // TODO(russt): Support [](const Convex& convex); it is already supported
      // by VPolytope.
      [](const Box& box) {
        Eigen::Matrix<double, 6, 3> A;
        A << Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
        Vector6d b;
        // clang-format off
        b << box.width()/2.0, box.depth()/2.0, box.height()/2.0,
             box.width()/2.0, box.depth()/2.0, box.height()/2.0;
        // clang-format on
        return std::make_pair(A, b);
      },
      [](const HalfSpace&) {
        // z <= 0.0.
        Eigen::RowVector3d A{0.0, 0.0, 1.0};
        Vector1d b{0.0};
        return std::make_pair(A, b);
      },
      [&geometry_id](const auto& unsupported) -> std::pair<MatrixXd, VectorXd> {
        throw std::logic_error(fmt::format(
            "{} (geometry_id={}) cannot be converted to a HPolyhedron",
            unsupported, geometry_id));
      }});
  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.InvertAndCompose(X_WE);
  // A_G*(p_GE + R_GE*p_EE_var) ≤ b_G
  A_ = A_G * X_GE.rotation().matrix();
  b_ = b_G - A_G * X_GE.translation();
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
    if (std::abs(b[i]) < kInf) {
      rows_to_keep.push_back(i);
    }
  }

  VectorXd b_eigen(b.size());
  b_eigen = VectorXd::Map(b.data(), b.size());

  *this = HPolyhedron(A_sparse.toDense()(rows_to_keep, eigen_all),
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
  if (A_.rows() == 0) {
    // If there are no hyperplanes, return the origin as the Chebyshev center.
    // Technically any point would work.
    return VectorXd::Zero(ambient_dimension());
  }
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
  if (A_.rows() == 0) {
    // If there are no hyperplanes, scaling does nothing.
    return *this;
  }
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
    const int mixing_steps,
    const std::optional<Eigen::Ref<const Eigen::MatrixXd>>& subspace,
    double tol) const {
  DRAKE_THROW_UNLESS(mixing_steps >= 1);
  if (subspace.has_value()) {
    DRAKE_THROW_UNLESS(subspace->rows() == ambient_dimension());
  }

  std::normal_distribution<double> gaussian;
  VectorXd current_sample = previous_sample;

  const int sampling_dim =
      subspace.has_value() ? subspace->cols() : previous_sample.rows();
  VectorXd gaussian_sample(sampling_dim);
  VectorXd direction(ambient_dimension());

  // If a row of the A matrix is orthogonal to all columns of the basis, then
  // the constraint from that row is enforced by the sample being in the column
  // space of the basis. Thus, we skip that constraint when performing
  // hit-and-run sampling.
  std::vector<bool> skip(A_.rows(), false);
  if (subspace.has_value()) {
    const double squared_tol = tol * tol;
    VectorXd subspace_j_squared_norm(subspace->cols());
    for (int j = 0; j < subspace->cols(); ++j) {
      subspace_j_squared_norm(j) = subspace->col(j).squaredNorm();
    }
    for (int i = 0; i < A_.rows(); ++i) {
      skip[i] = true;
      const double Ai_squared_norm = A_.row(i).squaredNorm();
      for (int j = 0; j < subspace->cols(); ++j) {
        if (std::pow(A_.row(i) * subspace->col(j), 2) >
            squared_tol * Ai_squared_norm * subspace_j_squared_norm(j)) {
          skip[i] = false;
          break;
        }
      }
    }
  }

  for (int step = 0; step < mixing_steps; ++step) {
    // Choose a random direction.
    for (int i = 0; i < gaussian_sample.size(); ++i) {
      gaussian_sample[i] = gaussian(*generator);
    }
    direction =
        subspace.has_value() ? *subspace * gaussian_sample : gaussian_sample;
    // Find max and min θ subject to
    //   A(previous_sample + θ*direction) ≤ b,
    // aka ∀i, θ * (A * direction)[i] ≤ (b - A * previous_sample)[i].
    VectorXd line_b = b_ - A_ * current_sample;
    VectorXd line_a = A_ * direction;
    double theta_max = std::numeric_limits<double>::infinity();
    double theta_min = -theta_max;
    for (int i = 0; i < line_a.size(); ++i) {
      if (skip[i]) {
        continue;
      } else if (line_a[i] < 0.0) {
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

  const double kWarnTolerance = 1e-8;
  if ((current_sample - previous_sample).template lpNorm<Eigen::Infinity>() <
      kWarnTolerance) {
    // If the new sample is extremely close to the previous sample, the user
    // may have a lower-dimensional polytope. We warn them if this happens. Note
    // that we use an absolute tolerance here, since it's unclear how to compute
    // an appropriate relative tolerance.
    drake::log()->warn(
        "The Hit and Run algorithm produced a random guess that is extremely "
        "close to `previous_sample`, which could indicate that the HPolyhedron "
        "being sampled is not full-dimensional. To draw samples from such an "
        "HPolyhedron, please use the `subspace` argument.");
  }
  return current_sample;
}

// Note: This method only exists to effectively provide ChebyshevCenter(),
// which is a non-static class method, as a default argument for
// previous_sample in the UniformSample method above.
VectorXd HPolyhedron::UniformSample(
    RandomGenerator* generator, const int mixing_steps,
    const std::optional<Eigen::Ref<const Eigen::MatrixXd>>& subspace,
    double tol) const {
  VectorXd center = ChebyshevCenter();
  return UniformSample(generator, center, mixing_steps, subspace, tol);
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
  if (IsEmpty()) {
    // We must always test if an HPolyhedron is empty, as all subsequent tests
    // depend on the HPolyhedron being nonempty. See
    // BoundednessCheckEmptyEdgeCases in hpolyhedron_test.cc for examples.
    return true;
  }
  if (A_.rows() < A_.cols()) {
    // If A_ has fewer rows than columns, then either the HPolyhedron is
    // unbounded, or it's empty due to a subset of the inequalities being
    // mutually exclusive (and therefore bounded). Emptiness has already
    // been checked above, so it's unbounded.
    return false;
  }
  Eigen::ColPivHouseholderQR<MatrixXd> qr(A_);
  if (qr.dimensionOfKernel() > 0) {
    // A similar edge case to the above is possible, so once again, the
    // HPolyhedron is bounded if and only if it is empty; because we already
    // checked for emptiness, it's unbounded.
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
  if (other.A().rows() == 0) {
    // If the other polytope is the full space, then we are certainly contained.
    return true;
  }
  if (A_.rows() == 0 && !other.A().isZero()) {
    // The full space can only be contained in another HPolyhedron if the other
    // HPolyhedron is also the full space. The other polytope cannot be the full
    // space if its A matrix is non-zero.
    return false;
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
  if (A_.rows() == 0) {
    // No inequalities so nothing is redundant;
    return redundant_indices;
  }
  if (A_.cols() == 0) {
    // All inequalities are redundant in a 0-dimensional space.
    for (int i = 0; i < A_.rows(); ++i) {
      redundant_indices.insert(i);
    }
    return redundant_indices;
  }
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

namespace {
HPolyhedron MoveFaceAndCull(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                            Eigen::VectorXd* face_center_distance,
                            std::vector<bool>* face_moved_in, int* i,
                            const std::vector<int>& i_cull) {
  DRAKE_DEMAND(ssize(*face_moved_in) >= *i + 1);
  (*face_moved_in)[*i] = true;
  std::vector<int> i_not_cull;
  i_not_cull.reserve(A.rows() - i_cull.size());
  int num_cull_before_i = 0;
  for (int j = 0; j < A.rows(); ++j) {
    if (std::find(i_cull.begin(), i_cull.end(), j) == i_cull.end()) {
      i_not_cull.push_back(j);
    } else if (*std::find(i_cull.begin(), i_cull.end(), j) < *i) {
      // Count how many indices before `i` are to be culled.
      ++num_cull_before_i;
    }
  }
  // Create updated matrices/lists without the elements/rows at indices in
  // `i_cull`.
  MatrixXd A_new(i_not_cull.size(), A.cols());
  VectorXd b_new(i_not_cull.size());
  std::vector<bool> face_moved_in_new;
  face_moved_in_new.reserve(i_not_cull.size());
  VectorXd face_center_distance_new(i_not_cull.size());
  for (int j = 0; j < ssize(i_not_cull); ++j) {
    A_new.row(j) = A.row(i_not_cull[j]);
    b_new(j) = b(i_not_cull[j]);
    face_moved_in_new.push_back((*face_moved_in)[i_not_cull[j]]);
    face_center_distance_new[j] = (*face_center_distance)[i_not_cull[j]];
  }
  // The face index needs to reduce because faces at lower indices have been
  // removed.
  *i = *i - num_cull_before_i;

  HPolyhedron inbody = HPolyhedron(A_new, b_new);
  *face_center_distance = face_center_distance_new;
  *face_moved_in = face_moved_in_new;

  return inbody;
}

bool CheckIntersectionAndPointContainmentConstraints(
    const HPolyhedron& inbody, const HPolyhedron& circumbody,
    const Eigen::Ref<const Eigen::MatrixXd>& points_to_contain,
    const std::vector<HPolyhedron>& intersecting_polytopes,
    bool keep_whole_intersection, double intersection_padding) {
  const double kConstraintTol = 1e-6;
  // Check that all given points are inside this HPolyhedron (the "inbody").
  for (int i_point = 0; i_point < points_to_contain.cols(); ++i_point) {
    if (!inbody.PointInSet(points_to_contain.col(i_point), kConstraintTol)) {
      return false;
    }
  }

  // Check intersection with each intersecting polytope
  if (keep_whole_intersection) {
    for (const HPolyhedron& intersecting_polytope : intersecting_polytopes) {
      const HPolyhedron intersection =
          circumbody.Intersection(intersecting_polytope);
      if (!intersection.ContainedIn(inbody, kConstraintTol)) {
        return false;
      }
    }
  } else {
    MathematicalProgram prog;
    for (const HPolyhedron& intersecting_polytope : intersecting_polytopes) {
      solvers::VectorXDecisionVariable x =
          prog.NewContinuousVariables(inbody.ambient_dimension(), "x");
      intersecting_polytope.AddPointInSetConstraints(&prog, x);
      prog.AddLinearConstraint(
          inbody.A(), VectorXd::Constant(inbody.b().rows(), -kInf),
          inbody.b() - (intersection_padding + kConstraintTol) *
                           inbody.A().rowwise().norm(),
          x);
    }
    solvers::MathematicalProgramResult result = Solve(prog);
    return result.is_success();
  }
  return true;
}
}  // namespace

HPolyhedron HPolyhedron::SimplifyByIncrementalFaceTranslation(
    double min_volume_ratio, bool do_affine_transformation, int max_iterations,
    const Eigen::Ref<const Eigen::MatrixXd>& points_to_contain,
    const std::vector<HPolyhedron>& intersecting_polytopes,
    bool keep_whole_intersection, double intersection_padding,
    int random_seed) const {
  DRAKE_THROW_UNLESS(min_volume_ratio > 0);
  DRAKE_THROW_UNLESS(max_iterations > 0);
  DRAKE_THROW_UNLESS(intersection_padding >= 0);

  const HPolyhedron circumbody = this->ReduceInequalities(0);
  MatrixXd circumbody_A = circumbody.A();
  VectorXd circumbody_b = circumbody.b();
  DRAKE_THROW_UNLESS(!circumbody.IsEmpty());
  DRAKE_THROW_UNLESS(circumbody.IsBounded());

  // Check that circumbody satisfies point containment and intersection
  // constraints.
  DRAKE_THROW_UNLESS(CheckIntersectionAndPointContainmentConstraints(
      circumbody, circumbody, points_to_contain, intersecting_polytopes,
      keep_whole_intersection, intersection_padding));

  // Ensure rows are normalized.
  for (int i = 0; i < circumbody_A.rows(); ++i) {
    const double initial_row_norm = circumbody_A.row(i).norm();
    circumbody_A.row(i) /= initial_row_norm;
    circumbody_b(i) /= initial_row_norm;
  }

  // Create vector of distances from all faces to circumbody center.
  const VectorXd circumbody_ellipsoid_center =
      circumbody.MaximumVolumeInscribedEllipsoid().center();
  VectorXd face_center_distance(circumbody_b.rows());
  for (int i = 0; i < circumbody_b.rows(); ++i) {
    face_center_distance(i) =
        circumbody_b(i) - circumbody_A.row(i) * circumbody_ellipsoid_center;
  }

  // Scaling factor for face distances being translated inward.
  const double face_scale_ratio =
      1 - std::pow(min_volume_ratio, 1.0 / ambient_dimension());

  // If scaled circumbody still meets the intersection constraint with a
  // polytope in `intersecting_polytopes`, then we don't need to worry about
  // losing this intersection in the face translation algorithm because the
  // scaled circumbody will always be a subset of the inbody.
  const HPolyhedron scaled_circumbody =
      circumbody.Scale(min_volume_ratio, circumbody_ellipsoid_center);
  std::vector<drake::geometry::optimization::HPolyhedron>
      reduced_intersecting_polytopes;
  reduced_intersecting_polytopes.reserve(intersecting_polytopes.size());
  for (const HPolyhedron& intersecting_polytope : intersecting_polytopes) {
    if (keep_whole_intersection) {
      reduced_intersecting_polytopes.push_back(intersecting_polytope);
    } else {
      const bool trivially_satisfied =
          CheckIntersectionAndPointContainmentConstraints(
              scaled_circumbody, circumbody,
              Eigen::MatrixXd(ambient_dimension(), 0),
              std::vector<HPolyhedron>{intersecting_polytope},
              keep_whole_intersection, intersection_padding);
      if (!trivially_satisfied) {
        reduced_intersecting_polytopes.push_back(intersecting_polytope);
      }
    }
  }

  // Initialize inbody as circumbody.
  HPolyhedron inbody = circumbody;
  std::vector<bool> face_moved_in(inbody.b().rows(), false);
  int iterations = 0;
  bool any_faces_moved = true;
  RandomGenerator generator(random_seed);
  while (any_faces_moved && iterations < max_iterations) {
    any_faces_moved = false;

    // Shuffle hyperplanes.
    std::vector<int> shuffle_inds(face_center_distance.size());
    std::iota(shuffle_inds.begin(), shuffle_inds.end(), 0);
    std::shuffle(shuffle_inds.begin(), shuffle_inds.end(), generator);
    MatrixXd A_shuffled(inbody.b().size(), ambient_dimension());
    VectorXd b_shuffled(inbody.b().size());
    VectorXd face_center_distance_shuffled(inbody.b().size());
    std::vector<bool> face_moved_in_shuffled(inbody.b().size());
    for (int i_shuffle : shuffle_inds) {
      A_shuffled.row(i_shuffle) = inbody.A().row(shuffle_inds[i_shuffle]);
      b_shuffled(i_shuffle) = inbody.b()(shuffle_inds[i_shuffle]);
      face_center_distance_shuffled(i_shuffle) =
          face_center_distance(shuffle_inds[i_shuffle]);
      face_moved_in_shuffled[i_shuffle] =
          face_moved_in[shuffle_inds[i_shuffle]];
    }
    inbody = HPolyhedron(A_shuffled, b_shuffled);
    face_center_distance = face_center_distance_shuffled;
    face_moved_in = face_moved_in_shuffled;

    int i = 0;
    // Loop through remaining hyperplanes.
    while (i < inbody.b().rows()) {
      if (!face_moved_in[i]) {
        // Lower bound on `b[i]`, to be updated by restrictions posed by
        // intersections.
        double b_i_min_allowed =
            inbody.b()(i) - face_scale_ratio * face_center_distance(i);

        // Loop through intersecting hyperplanes and update `b_i_min_allowed`
        // based on how far each intersection allows the hyperplane to move.
        for (const HPolyhedron& intersecting_polytope :
             reduced_intersecting_polytopes) {
          MathematicalProgram prog;
          solvers::VectorXDecisionVariable x =
              prog.NewContinuousVariables(ambient_dimension(), "x");

          if (keep_whole_intersection) {
            const HPolyhedron intersection =
                inbody.Intersection(intersecting_polytope);

            prog.AddLinearCost(-inbody.A().row(i), 0, x);
            intersection.AddPointInSetConstraints(&prog, x);
          } else {
            solvers::VectorXDecisionVariable b_i =
                prog.NewContinuousVariables(1);

            intersecting_polytope.AddPointInSetConstraints(&prog, x);
            prog.AddLinearConstraint(
                inbody.A(), VectorXd::Constant(inbody.b().rows(), -kInf),
                inbody.b() -
                    VectorXd::Constant(inbody.b().rows(), intersection_padding),
                x);

            // A(i) x <= b(i) - intersection_padding.
            RowVectorXd row = Eigen::RowVectorXd(inbody.A().cols() + 1);
            row << inbody.A().row(i), -1.0;
            solvers::VectorXDecisionVariable xb(inbody.ambient_dimension() + 1);
            xb << x, b_i;
            prog.AddLinearConstraint(row, -kInf, -intersection_padding, xb);
            prog.AddLinearCost(b_i[0]);
          }

          solvers::MathematicalProgramResult result = Solve(prog);
          if (result.is_success()) {
            // A multiplier for cost in LPs that find how far a face can be
            // moved inward before losing an intersection.  Interpretation of
            // the optimal cost varies depending on `keep_whole_intersection`
            // parameter value.
            const int cost_multiplier = keep_whole_intersection ? -1 : 1;
            // A numerical tolerance used to ensure that the intersection LP
            // continues to be feasible throughout the iterations.
            const double kIntersectionFeasibilityPad = 1e-5;
            if (cost_multiplier * result.get_optimal_cost() +
                    kIntersectionFeasibilityPad >
                b_i_min_allowed) {
              b_i_min_allowed = cost_multiplier * result.get_optimal_cost() +
                                kIntersectionFeasibilityPad;
            }
          } else {
            log()->warn(
                "Intersection program did not solve properly.  Will not"
                "  move in hyperplane.");
            b_i_min_allowed = inbody.b()(i);
          }
        }
        // Loop through points to contain and update `b_i_min_allowed`
        // based on how far each point allows the hyperplane to move.
        for (int i_point = 0; i_point < points_to_contain.cols(); ++i_point) {
          b_i_min_allowed =
              std::max(b_i_min_allowed,
                       inbody.A().row(i).dot(points_to_contain.col(i_point)));
        }

        // Ensure `b_min_allowed` does not exceed `b(i)` (hyperplane does not
        // move outward due to adding kIntersectionFeasibilityPad).
        b_i_min_allowed = std::min(b_i_min_allowed, inbody.b()(i));

        // Find which hyperplanes become redundant if we move the hyperplane
        // as far as is allowed.  If any, move face and cull other faces.
        VectorXd b_proposed = inbody.b();
        b_proposed(i) = b_i_min_allowed;
        const HPolyhedron inbody_proposed = HPolyhedron(inbody.A(), b_proposed);
        const std::set<int> i_redundant_set = inbody_proposed.FindRedundant(0);
        const std::vector<int> i_redundant(i_redundant_set.begin(),
                                           i_redundant_set.end());
        if (i_redundant.size() > 0) {
          any_faces_moved = true;
          const MatrixXd A = inbody.A();
          inbody = MoveFaceAndCull(A, b_proposed, &face_center_distance,
                                   &face_moved_in, &i, i_redundant);
        }
      }
      ++i;
    }
    log()->debug("{} faces saved so far",
                 circumbody.b().size() - inbody.b().size());

    ++iterations;
  }

  const HPolyhedron inbody_before_affine_transformation =
      HPolyhedron(inbody.A(), inbody.b());
  // Affine transformation.
  if (do_affine_transformation) {
    inbody = inbody.MaximumVolumeInscribedAffineTransformation(circumbody);
  }

  // Check if intersection and containment constraints are still satisfied after
  // affine transformation, and revert if not.  There is currently no way to
  // constrain that the affine transformation upholds these constraints.
  if (!CheckIntersectionAndPointContainmentConstraints(
          inbody, circumbody, points_to_contain, intersecting_polytopes,
          keep_whole_intersection, intersection_padding)) {
    inbody = inbody_before_affine_transformation;
    log()->debug(
        "Reverting affine transformation due to loss of containment "
        " of point or intersection with other polytope");
  }
  return inbody;
}

HPolyhedron HPolyhedron::MaximumVolumeInscribedAffineTransformation(
    const HPolyhedron& circumbody, bool check_bounded) const {
  DRAKE_THROW_UNLESS(this->ambient_dimension() ==
                     circumbody.ambient_dimension());

  if (check_bounded) {
    DRAKE_THROW_UNLESS(this->IsBounded());
  }

  int n_y = circumbody.A().rows();
  int n_x = this->A().rows();

  // Affine transformation is parameterized by translation `t` and
  // transformation matrix T.
  MathematicalProgram prog;
  solvers::VectorXDecisionVariable t =
      prog.NewContinuousVariables(this->ambient_dimension(), "t");
  solvers::MatrixXDecisionVariable T =
      prog.NewSymmetricContinuousVariables(this->ambient_dimension(), "T");

  // T being PSD is necessary for the cost to be convex, though it is
  // restrictive.
  prog.AddPositiveSemidefiniteConstraint(T);
  // Log determinant is monotonic with volume.
  prog.AddMaximizeLogDeterminantCost(T.cast<Expression>());

  // Containment conditions for affine transformation of HPolyhedron
  // in HPolyhedron.
  solvers::MatrixXDecisionVariable Lambda =
      prog.NewContinuousVariables(n_y, n_x, "Lambda");
  prog.AddBoundingBoxConstraint(0, kInf, Lambda);

  // Loop through and add the elements of the constraints
  // Lambda * `this`.A() = circumbody.A() * T
  // implemented as
  // [this.A().col(i_col).T, -circumbody.A().row(i_row)] * [Lambda.row(i_row).T;
  // T.col(i_col)] = 0 over rows i_row and columns i_col,
  // and
  // Lambda * `this`.b() + circumbody.A() * t <= circumbody.b().
  MatrixXd left_hand_equality_matrix(1, n_x + circumbody.ambient_dimension());
  solvers::VectorXDecisionVariable equality_variables(
      n_x + circumbody.ambient_dimension());
  MatrixXd left_hand_inequality_matrix(1, n_x + circumbody.ambient_dimension());
  solvers::VectorXDecisionVariable inequality_variables(
      n_x + circumbody.ambient_dimension());
  for (int i_row = 0; i_row < n_y; ++i_row) {
    for (int i_col = 0; i_col < circumbody.ambient_dimension(); ++i_col) {
      left_hand_equality_matrix << this->A().col(i_col).transpose(),
          -circumbody.A().row(i_row);
      equality_variables << Lambda.row(i_row).transpose(), T.col(i_col);
      prog.AddLinearEqualityConstraint(left_hand_equality_matrix,
                                       VectorXd::Zero(1), equality_variables);
    }
    left_hand_inequality_matrix << this->b().transpose(),
        circumbody.A().row(i_row);
    inequality_variables << Lambda.row(i_row).transpose(), t;
    prog.AddLinearConstraint(left_hand_inequality_matrix, -kInf,
                             circumbody.b()[i_row], inequality_variables);
  }

  solvers::MathematicalProgramResult result = Solve(prog);

  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the affine transformation problem; it "
        "terminated with SolutionResult {}). Make sure that your polyhedra are "
        "bounded and have interiors.",
        result.get_solver_id().name(), result.get_solution_result()));
  }

  const MatrixXd T_sol = result.GetSolution(T);
  const VectorXd t_sol = result.GetSolution(t);
  const MatrixXd T_inv = T_sol.inverse();

  MatrixXd A_optimized = this->A() * T_inv;
  VectorXd b_optimized = this->b() + this->A() * T_inv * t_sol;

  for (int i = 0; i < n_x; ++i) {
    const double initial_row_norm = A_optimized.row(i).norm();
    A_optimized.row(i) /= initial_row_norm;
    b_optimized(i) /= initial_row_norm;
  }

  return HPolyhedron(A_optimized, b_optimized);
}

std::unique_ptr<ConvexSet> HPolyhedron::DoClone() const {
  return std::make_unique<HPolyhedron>(*this);
}

std::optional<bool> HPolyhedron::DoPointInSetShortcut(
    const Eigen::Ref<const VectorXd>& x, double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  return ((A_ * x).array() <= b_.array() + tol).all();
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
HPolyhedron::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  VectorX<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  if (A_.rows() > 0) {
    new_constraints.push_back(prog->AddLinearConstraint(
        A_,
        VectorXd::Constant(b_.size(), -std::numeric_limits<double>::infinity()),
        b_, vars));
  }
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
  if (m == 0) {
    // No constraints to add.
    return constraints;
  }
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
  if (m == 0) {
    // No constraints to add.
    return constraints;
  }
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
