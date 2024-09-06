#include "drake/geometry/optimization/affine_subspace.h"

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/geometry/optimization/affine_ball.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/hyperrectangle.h"
#include "drake/geometry/optimization/vpolytope.h"
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

// Internal namespace for better numerical computations of affine hulls.
namespace {
AffineSubspace AffineBallAffineHull(const AffineBall& affine_ball,
                                    std::optional<double> tol) {
  Eigen::FullPivHouseholderQR<MatrixXd> qr(affine_ball.B());
  if (tol) {
    qr.setThreshold(tol.value());
  }
  Eigen::MatrixXd basis =
      qr.matrixQ() *
      MatrixXd::Identity(affine_ball.ambient_dimension(), qr.rank());
  return AffineSubspace(basis, affine_ball.center());
}

AffineSubspace CartesianProductAffineHull(
    const CartesianProduct& cartesian_product, std::optional<double> tol) {
  // TODO(cohnt): Support affine transformations of Cartesian products.
  // This pre-condition is currently checked in the base method.
  DRAKE_DEMAND(cartesian_product.A() == std::nullopt);
  DRAKE_DEMAND(cartesian_product.b() == std::nullopt);
  // Compute the affine hull of each factor of cartesian_product and combine.

  // The basis will be a block diagonal matrix, whose blocks correspond to the
  // bases of the affine subspace of each factor. Not all blocks will be square,
  // and some of the columns on the right will be skipped, since the affine hull
  // may be a proper subspace.
  Eigen::MatrixXd basis =
      Eigen::MatrixXd::Zero(cartesian_product.ambient_dimension(),
                            cartesian_product.ambient_dimension());
  // The translation will be a vector, concatenating all of the translations of
  // each factor. Zero-initialization is not needed, since all entries will be
  // overwritten in the following loop.
  Eigen::VectorXd translation(cartesian_product.ambient_dimension());
  int current_dimension = 0;
  int num_basis_vectors = 0;
  for (int i = 0; i < cartesian_product.num_factors(); ++i) {
    AffineSubspace a(cartesian_product.factor(i), tol);
    basis.block(current_dimension, num_basis_vectors, a.ambient_dimension(),
                a.AffineDimension()) = a.basis();
    translation.segment(current_dimension, a.ambient_dimension()) =
        a.translation();
    current_dimension += a.ambient_dimension();
    num_basis_vectors += a.AffineDimension();
  }
  return AffineSubspace(basis.leftCols(num_basis_vectors), translation);
}

AffineSubspace HyperrectangleAffineHull(const Hyperrectangle& hyperrectangle,
                                        std::optional<double> tol) {
  Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(
      hyperrectangle.ambient_dimension(), hyperrectangle.ambient_dimension());
  int current_dimension = 0;
  int num_basis_vectors = 0;
  for (int i = 0; i < hyperrectangle.ambient_dimension(); ++i) {
    // If the numerical tolerance was not specified, we use a reasonable
    // default.
    if (hyperrectangle.ub()[i] - hyperrectangle.lb()[i] >
        (tol ? tol.value() : 1e-12)) {
      basis(current_dimension, num_basis_vectors) = 1;
      ++num_basis_vectors;
    }
    ++current_dimension;
  }
  return AffineSubspace(basis.leftCols(num_basis_vectors), hyperrectangle.lb());
}

AffineSubspace VPolytopeAffineHull(const VPolytope& vpolytope,
                                   std::optional<double> tol) {
  DRAKE_THROW_UNLESS(vpolytope.vertices().size() > 0);
  // Eigen::JacobiSVD<MatrixXd, Eigen::DecompositionOptions::ComputeThinU>
  // svd(vpolytope.vertices());
  Eigen::JacobiSVD<MatrixXd> svd;
  Eigen::MatrixXd centered_points =
      vpolytope.vertices()
          .rightCols(vpolytope.vertices().cols() - 1)
          .colwise() -
      vpolytope.vertices().col(0);
  svd.compute(centered_points, Eigen::DecompositionOptions::ComputeThinU);
  if (tol) {
    svd.setThreshold(tol.value());
  }
  return AffineSubspace(svd.matrixU().leftCols(svd.rank()),
                        vpolytope.vertices().col(0));
}
}  // namespace

AffineSubspace::AffineSubspace(const ConvexSet& set, std::optional<double> tol)
    : ConvexSet(0, true) {
  if (tol) {
    DRAKE_THROW_UNLESS(tol >= 0);
  }
  // If the set is clearly a singleton, we can easily compute its affine hull.
  const auto singleton_maybe = set.MaybeGetPoint();
  if (singleton_maybe.has_value()) {
    // Fall back to the basis and translation constructor.
    *this = AffineSubspace(Eigen::MatrixXd::Zero(set.ambient_dimension(), 0),
                           singleton_maybe.value());
    return;
  }

  const AffineBall* const maybe_affine_ball =
      dynamic_cast<const AffineBall* const>(&set);
  const AffineSubspace* const maybe_affine_subspace =
      dynamic_cast<const AffineSubspace* const>(&set);
  const CartesianProduct* const maybe_cartesian_product =
      dynamic_cast<const CartesianProduct* const>(&set);
  const Hyperellipsoid* const maybe_hyperellipsoid =
      dynamic_cast<const Hyperellipsoid* const>(&set);
  const Hyperrectangle* const maybe_hyperrectangle =
      dynamic_cast<const Hyperrectangle* const>(&set);
  const VPolytope* const maybe_vpolytope =
      dynamic_cast<const VPolytope* const>(&set);
  if (maybe_affine_ball) {
    *this = AffineBallAffineHull(*maybe_affine_ball, tol);
    return;
  } else if (maybe_affine_subspace) {
    // We can directly copy the object.
    *this = *maybe_affine_subspace;
    return;
  } else if (maybe_cartesian_product) {
    // TODO(cohnt): Handle the case where the CartesianProduct has an associated
    // affine transformation.
    if (maybe_cartesian_product->A() == std::nullopt &&
        maybe_cartesian_product->b() == std::nullopt) {
      *this = CartesianProductAffineHull(*maybe_cartesian_product, tol);
      return;
    }
  } else if (maybe_hyperellipsoid) {
    // Hyperellipsoids are always positive volume, so we can trivially construct
    // their affine hull as the whole vector space.
    *this = AffineSubspace(
        Eigen::MatrixXd::Identity(maybe_hyperellipsoid->ambient_dimension(),
                                  maybe_hyperellipsoid->ambient_dimension()),
        Eigen::VectorXd::Zero(maybe_hyperellipsoid->ambient_dimension()));
    return;
  } else if (maybe_hyperrectangle) {
    *this = HyperrectangleAffineHull(*maybe_hyperrectangle, tol);
    return;
  } else if (maybe_vpolytope) {
    *this = VPolytopeAffineHull(*maybe_vpolytope, tol);
    return;
  }

  // If the set is not clearly a singleton, we find a feasible point and
  // iteratively compute a basis of the affine hull. If the numerical tolerance
  // was not specified, we use a reasonable default.
  if (!tol) {
    tol = 1e-12;
  }
  // If no feasible point exists, the set is empty, so we throw an error.
  const auto translation_maybe = set.MaybeGetFeasiblePoint();
  if (!translation_maybe.has_value()) {
    throw std::runtime_error(
        "AffineSubspace: Cannot take the affine hull of an empty set!");
  }
  const VectorXd translation = translation_maybe.value();
  // The basis of the affine hull. We preallocate the maximum dimension this
  // basis could assume.
  MatrixXd basis(set.ambient_dimension(), set.ambient_dimension());
  // The basis of the orthogonal complement. We preallocate the maximum
  // dimension this basis could assume.
  MatrixXd basis_orth(set.ambient_dimension(), set.ambient_dimension());
  const MatrixXd I =
      MatrixXd::Identity(set.ambient_dimension(), set.ambient_dimension());
  int affine_dimension = 0;
  int complement_dimension = 0;

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

  // This is the objective we use. At each iteration, we will update this cost
  // to search for a new basis vector. Anytime this value is negative, it means
  // we have found a new feasible direction, so we add it to the basis, and also
  // constrain future feasible x vectors to be orthogonal to it (to ensure
  // linear independence of the basis).
  Binding<LinearCost> objective =
      prog.AddLinearCost(VectorXd::Zero(set.ambient_dimension()), x);

  // We incrementally build up a basis of the affine hull and its orthogonal
  // complement. At each iteration, we take in a direction vector v which is
  // orthogonal to all vectors known to be in the affine hull or in the
  // orthogonal complement. We then attempt to minimize <x,v>. We maintain the
  // invariant that at the start of each iteration, any two column vectors taken
  // from distinct matrices {basis.leftCols(affine_dimension),
  // basis_orth.leftCols(complement_dimension)} are orthogonal, and that
  // each of {basis.leftCols(affine_dimension),
  // basis_orth.leftCols(complement_dimension)} are orthonormal bases.
  //
  // If the inner product is less than -tol, we add x to the basis vectors and
  // increment affine_dimension. Otherwise, we add v to the complement basis and
  // increment complement_dimension. The loop terminates when the sum of
  // affine_dimension and complement_dimension is equal to the ambient
  // dimension.
  while (affine_dimension + complement_dimension < set.ambient_dimension()) {
    // Compute a spanning set for the unchecked directions by constructing a
    // matrix whose columns are orthogonal to basis ⊕ basis_orth. Because this
    // is not a basis, some of the columns could be all zero, so we have to find
    // a nonzero column to use as the direction. Because the set spans the
    // orthogonal complement of basis ⊕ basis_orth, which has dimension at least
    // one, at least one of the columns of spanning_unknown must be nonzero.
    const MatrixXd spanning_unknown =
        (I - basis_orth.leftCols(complement_dimension) *
                 basis_orth.leftCols(complement_dimension).transpose()) *
        (I - basis.leftCols(affine_dimension) *
                 basis.leftCols(affine_dimension).transpose());
    int ii = 0;
    VectorXd next_direction = spanning_unknown.col(0);
    // We can use a generous check for 0 here since the spanning_unknown vectors
    // are all approximately unit norm or zero.
    while (next_direction.norm() < 1e-8) {
      ++ii;
      DRAKE_THROW_UNLESS(ii < spanning_unknown.cols());
      next_direction = spanning_unknown.col(ii);
    }
    next_direction.normalize();
    objective.evaluator()->UpdateCoefficients(next_direction);
    // Minimize <x, objective>.
    auto result = solvers::Solve(prog);
    if (!result.is_success()) {
      throw std::runtime_error(
          fmt::format("AffineSubspace: Failed to compute the affine hull! The "
                      "solution result was {}.",
                      result.get_solution_result()));
    }

    if (result.get_optimal_cost() < -tol.value()) {
      // x is in the affine hull, and is added to the basis.
      VectorXd new_basis_vector = result.GetSolution(x);
      basis.col(affine_dimension++) =
          new_basis_vector / new_basis_vector.norm();
      prog.AddLinearEqualityConstraint(basis.col(affine_dimension - 1), 0, x);
    } else {
      // The objective vector is orthogonal to the affine hull. We can then add
      // it to the complement basis. By construction, this vector is already a
      // unit vector and orthogonal to the orthogonal complement basis.
      basis_orth.col(complement_dimension++) = next_direction;
    }
  }

  // By construction, basis_vectors is linearly independent. Because we have
  // checked each direction in the ambient space, it will also span the other
  // convex set, so we now have its affine hull.
  *this = AffineSubspace(basis.leftCols(affine_dimension), translation);
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

std::optional<bool> AffineSubspace::DoPointInSetShortcut(
    const Eigen::Ref<const VectorXd>& x, double tol) const {
  DRAKE_DEMAND(ambient_dimension() > 0);
  if (basis_.cols() == 0) {
    // In this case, it's just a point, so we directly compare
    return is_approx_equal_abstol(x, translation_, tol);
  }
  // Otherwise, project onto the flat, and compare to the input.
  Eigen::VectorXd projected_points(x.rows());
  DoProjectionShortcut(x, &projected_points);
  return is_approx_equal_abstol(x, projected_points, tol);
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

std::vector<std::optional<double>> AffineSubspace::DoProjectionShortcut(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    EigenPtr<Eigen::MatrixXd> projected_points) const {
  // If the set is a point, the projection is just that point. This also
  // directly handles the zero-dimensional case.
  const auto maybe_point = DoMaybeGetPoint();
  if (maybe_point) {
    const Eigen::VectorXd eigen_dists =
        (points - maybe_point.value()).colwise().norm();
    std::vector<std::optional<double>> distances(
        eigen_dists.data(), eigen_dists.data() + eigen_dists.size());
    if (points.cols() == 1) {
      projected_points->col(0) = maybe_point.value();
      return distances;
    } else {
      // Outer product, which will return x.cols() copies of the feasible
      // point.
      *projected_points =
          maybe_point.value() * Eigen::RowVectorXd::Ones(points.cols());
      return distances;
    }
  }
  const Eigen::MatrixXd least_squares =
      basis_decomp_->solve(points.colwise() - translation_);
  *projected_points = (basis_ * least_squares).colwise() + translation_;
  const Eigen::VectorXd eigen_dists =
      (points - *projected_points).colwise().norm();
  std::vector<std::optional<double>> distances(
      eigen_dists.data(), eigen_dists.data() + eigen_dists.size());
  return distances;
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
  // The orthogonal complement is equivalent to the kernel of the QR
  // decomposition stored in this class. So we can simply access the rightmost
  // columns of the Q matrix, as described here:
  // https://stackoverflow.com/questions/54766392/eigen-obtain-the-kernel-of-a-sparse-matrix
  int orthogonal_complement_dimension = ambient_dimension() - AffineDimension();
  MatrixXd Q = basis_decomp_.value().householderQ() *
               MatrixXd::Identity(ambient_dimension(), ambient_dimension());
  return Q.rightCols(orthogonal_complement_dimension);
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
