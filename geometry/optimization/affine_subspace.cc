#include "drake/geometry/optimization/affine_subspace.h"

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/geometry/optimization/spectrahedron.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;
using solvers::LinearCost;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

AffineSubspace::AffineSubspace()
    : AffineSubspace(MatrixXd(0, 0), VectorXd(0)) {}

AffineSubspace::AffineSubspace(const Eigen::Ref<const MatrixXd>& basis,
                               const Eigen::Ref<const VectorXd>& translation)
    : ConvexSet(basis.rows(), true), basis_(basis), translation_(translation) {
  DRAKE_THROW_UNLESS(basis_.rows() == translation_.size());
  DRAKE_THROW_UNLESS(basis_.rows() >= basis_.cols());
  if (basis.rows() > 0 && basis.cols() > 0) {
    basis_decomp_ = Eigen::ColPivHouseholderQR<MatrixXd>(basis_);
    DRAKE_THROW_UNLESS(basis_decomp_.value().rank() == basis_.cols());
  } else {
    basis_decomp_ = std::nullopt;
  }
}

AffineSubspace::AffineSubspace(const ConvexSet& set, double tol)
    : ConvexSet(0, true) {
  // If the set is clearly a singleton, we can easily compute its affine hull.
  const auto singleton_maybe = set.MaybeGetPoint();
  if (singleton_maybe.has_value()) {
    // Fall back to the basis and translation constructor.
    *this = AffineSubspace(Eigen::MatrixXd::Zero(set.ambient_dimension(), 0),
                           singleton_maybe.value());
    return;
  }

  // If the set is not clearly a singleton, we find a feasible point and
  // iteratively compute a basis of the affine hull. If no feasible point
  // exists, the set is empty, so we throw an error.
  const auto translation_maybe = set.MaybeGetFeasiblePoint();
  if (!translation_maybe.has_value()) {
    throw std::runtime_error(
        "AffineSubspace: Cannot take the affine hull of an empty set!");
  }
  const VectorXd translation = translation_maybe.value();
  std::vector<VectorXd> basis_vectors;

  // Create the mathematical program we will use to find basis vectors.
  MathematicalProgram prog;

  // x represents the displacement within a feasible set. We add a bounding box
  // constraint to ensure the problem is not unbounded.
  VectorXDecisionVariable x =
      prog.NewContinuousVariables(set.ambient_dimension(), "x");
  prog.AddBoundingBoxConstraint(-1, 1, x);

  // y and z are two feasible points in the set
  VectorXDecisionVariable y =
      prog.NewContinuousVariables(set.ambient_dimension(), "y");
  VectorXDecisionVariable z =
      prog.NewContinuousVariables(set.ambient_dimension(), "z");
  prog.AddLinearConstraint(x == y - z);
  set.AddPointInSetConstraints(&prog, y);
  set.AddPointInSetConstraints(&prog, z);

  // This is the objective we use. We will iteratively try to minimize the ith
  // component of x for each dimension. Anytime this value is negative, it means
  // we have found a new feasible direction, so we add it to the basis, and also
  // constrain future feasible x vectors to be orthogonal to it (to ensure
  // linear independence of the basis).
  VectorXd new_objective_vector = VectorXd::Zero(set.ambient_dimension());
  Binding<LinearCost> objective = prog.AddLinearCost(new_objective_vector, x);

  for (int i = 0; i < set.ambient_dimension(); ++i) {
    // Update the objective to check the ith dimension
    new_objective_vector.setZero();
    new_objective_vector[i] = 1;
    objective.evaluator()->UpdateCoefficients(new_objective_vector);

    // Minimize x[i]
    auto result = solvers::Solve(prog);
    if (!result.is_success()) {
      throw std::runtime_error(
          fmt::format("AffineSubspace: Failed to compute the affine hull! The "
                      "solution result was {}.",
                      result.get_solution_result()));
    }
    if (result.get_optimal_cost() < -tol) {
      // Get the solution as a new basis vector, and add a constraint that x
      // must now be orthogonal to that new basis vector.
      basis_vectors.push_back(result.GetSolution(x));
      prog.AddLinearConstraint(basis_vectors.back(), 0, 0, x);
    }
  }

  // By construction, basis_vectors is linearly independent. Because we have
  // checked each direction in the ambient space, it will also span the other
  // convex set, so we now have its affine hull.
  MatrixXd basis(set.ambient_dimension(), basis_vectors.size());
  for (size_t i = 0; i < basis_vectors.size(); ++i) {
    basis.col(i) = basis_vectors[i];
  }
  *this = AffineSubspace(basis, translation);
}

AffineSubspace::~AffineSubspace() = default;

std::unique_ptr<ConvexSet> AffineSubspace::DoClone() const {
  return std::make_unique<AffineSubspace>(*this);
}

std::optional<bool> AffineSubspace::DoIsBoundedShortcut() const {
  return basis_.cols() == 0;
}

bool AffineSubspace::DoIsEmpty() const {
  return false;
}

std::optional<VectorXd> AffineSubspace::DoMaybeGetPoint() const {
  if (basis_.cols() == 0) {
    return translation_;
  }
  return std::nullopt;
}

std::optional<VectorXd> AffineSubspace::DoMaybeGetFeasiblePoint() const {
  return translation_;
}

bool AffineSubspace::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                                  double tol) const {
  DRAKE_DEMAND(ambient_dimension() > 0);
  if (basis_.cols() == 0) {
    // In this case, it's just a point, so we directly compare
    return is_approx_equal_abstol(x, translation_, tol);
  }
  // Otherwise, project onto the flat, and compare to the input.
  return is_approx_equal_abstol(x, Project(x), tol);
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
AffineSubspace::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x) const {
  // We require that (x - translation) can be written as a linear
  // combination of the basis vectors
  std::vector<Binding<Constraint>> new_constraints;
  const int n = ambient_dimension();
  const int m = basis_.cols();
  VectorXDecisionVariable alpha = prog->NewContinuousVariables(m, "a");
  // basis_ * α - x = -translation_
  MatrixXd A(n, m + n);
  A.leftCols(m) = basis_;
  A.rightCols(n) = -MatrixXd::Identity(n, n);
  new_constraints.push_back(
      prog->AddLinearEqualityConstraint(A, -translation_, {alpha, x}));
  return {std::move(alpha), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
AffineSubspace::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram*,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>&,
    const Variable&) const {
  // This method of ConvexSet is currently only used in GraphOfConvexSets,
  // and it's bad practice to include unbounded sets in such optimizations.
  // "For GCS the boundedness of the sets is required because you want to
  // be sure that if the perspective coefficient is zero, then the vector
  // variable is constrained to be zero. If the set is unbounded, then the
  // vector variables is only constrained in the recession cone of the set."
  // -Tobia Marcucci
  throw std::runtime_error(
      "AffineSubspace::DoAddPointInNonnegativeScalingConstraints() is not "
      "implemented yet.");
}

std::vector<Binding<Constraint>>
AffineSubspace::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram*, const Eigen::Ref<const MatrixXd>&,
    const Eigen::Ref<const VectorXd>&, const Eigen::Ref<const VectorXd>&,
    double, const Eigen::Ref<const VectorXDecisionVariable>&,
    const Eigen::Ref<const VectorXDecisionVariable>&) const {
  throw std::runtime_error(
      "AffineSubspace::DoAddPointInNonnegativeScalingConstraints() is not "
      "implemented yet.");
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
AffineSubspace::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not supported by AffineSubspace.");
}

double AffineSubspace::DoCalcVolume() const {
  if (AffineDimension() < ambient_dimension()) {
    // An AffineSubspace has zero volume if it has a lower affine dimension than
    // its ambient space. Otherwise, it represents the whole ambient space, and
    // has infinite volume."
    return 0;
  }
  return std::numeric_limits<double>::infinity();
}

Eigen::MatrixXd AffineSubspace::Project(
    const Eigen::Ref<const Eigen::MatrixXd>& x) const {
  DRAKE_THROW_UNLESS(x.rows() == ambient_dimension());
  // If the set is a point, the projection is just that point. This also
  // directly handles the zero-dimensional case.
  const auto maybe_point = DoMaybeGetPoint();
  if (maybe_point) {
    if (x.cols() == 1) {
      return maybe_point.value();
    } else {
      // Outer product, which will return x.cols() copies of the feasible point.
      return maybe_point.value() * Eigen::RowVectorXd::Ones(x.cols());
    }
  }
  const Eigen::MatrixXd least_squares =
      basis_decomp_->solve(x.colwise() - translation_);
  return (basis_ * least_squares).colwise() + translation_;
}

Eigen::MatrixXd AffineSubspace::ToLocalCoordinates(
    const Eigen::Ref<const Eigen::MatrixXd>& x) const {
  DRAKE_THROW_UNLESS(x.rows() == ambient_dimension());
  // If the set is a point, then the basis is empty, so there are no local
  // coordinates. This behavior is handled by returning a length-zero vector.
  // This also directly handles the zero-dimensional case.
  auto maybe_point = DoMaybeGetPoint();
  if (maybe_point) {
    return Eigen::MatrixXd::Zero(0, x.cols());
  }
  return basis_decomp_->solve(x.colwise() - translation_);
}

Eigen::MatrixXd AffineSubspace::ToGlobalCoordinates(
    const Eigen::Ref<const Eigen::MatrixXd>& y) const {
  DRAKE_THROW_UNLESS(y.rows() == AffineDimension());
  return (basis_ * y).colwise() + translation_;
}

bool AffineSubspace::ContainedIn(const AffineSubspace& other,
                                 double tol) const {
  // For this AffineSubspace to be contained in other, their ambient
  // dimensions must be the same, its translation_ must be in other,
  // and its subspace must be contained in the subspace of other.
  if (ambient_dimension() != other.ambient_dimension()) {
    return false;
  }
  if (!other.PointInSet(translation_, tol)) {
    return false;
  }
  // Check that this basis is contained in other. If the basis vectors
  // are all contained in other, than the whole set is.
  for (int i = 0; i < basis_.cols(); ++i) {
    if (!other.PointInSet(basis_.col(i) + translation_, tol)) {
      return false;
    }
  }
  return true;
}

bool AffineSubspace::IsNearlyEqualTo(const AffineSubspace& other,
                                     double tol) const {
  return ContainedIn(other, tol) && other.ContainedIn(*this, tol);
}

Eigen::MatrixXd AffineSubspace::OrthogonalComplementBasis() const {
  // If we have a zero-dimensional AffineSubspace (i.e. a point), the
  // basis_decomp_ isn't constructed, and we just return a basis of the whole
  // space.
  if (!basis_decomp_.has_value()) {
    return MatrixXd::Identity(ambient_dimension(), ambient_dimension());
  }
  // The perpendicular space is equivalent to the kernel of the QR decomposition
  // stored in this class. So we can simply access the rightmost columns of the
  // Q matrix, as described here:
  // https://stackoverflow.com/questions/54766392/eigen-obtain-the-kernel-of-a-sparse-matrix
  int perpendicular_basis_dimension = ambient_dimension() - AffineDimension();
  MatrixXd Q = basis_decomp_.value().householderQ() *
               MatrixXd::Identity(ambient_dimension(), ambient_dimension());
  return Q.rightCols(perpendicular_basis_dimension);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
