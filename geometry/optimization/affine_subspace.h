#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** An affine subspace (also known as a "flat", a "linear variety", or a "linear
manifold") is a vector subspace of some Euclidean space, potentially translated
so as to not pass through the origin. Examples include points, lines, and planes
(not necessarily through the origin).

An affine subspace is described by a basis of its corresponding vector subspace,
plus a translation. This description is not unique as any point in the affine
subspace can be used as a translation, and any basis of the corresponding vector
subspace is valid.

An affine subspace can never be empty, because a vector subspace can never be
empty. Thus, the translation will always be contained in the flat. An affine
subspace is bounded if it is a point, which is when the basis has zero columns.

@ingroup geometry_optimization */
class AffineSubspace final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AffineSubspace);

  /** Constructs a default (zero-dimensional, nonempty) affine subspace. */
  AffineSubspace();

  /** Constructs the affine subspace from an n-by-m matrix describing the basis,
  where n is the ambient dimension, and m is the dimension of the subspace, and
  from an n-dimensional vector describing the translation. The set is {x | x =
  translation + basis*y, y ∈ Rᵐ} The columns must be linearly independent.
  @pre basis.rows() == translation.size(). */
  explicit AffineSubspace(const Eigen::Ref<const Eigen::MatrixXd>& basis,
                          const Eigen::Ref<const Eigen::VectorXd>& translation);

  /** Constructs an affine subspace as the affine hull of another convex set.
  The generic approach is to find a feasible point in the set, and then
  iteratively compute feasible vectors until we have a basis that spans the set.
  If you pass in a convex set whose points are matrix-valued (e.g. a
  Spectrahedron), then the affine subspace will work over a flattened
  representation of those coordinates. (So a Spectrahedron with n-by-n matrices
  will output an AffineSubspace with ambient dimension (n * (n+1)) / 2.)

  `tol` sets the numerical precision of the computation. For each dimension, a
  pair of feasible points are constructed, so as to maximize the displacement in
  that dimension. If their displacement along that dimension is larger than tol,
  then the vector connecting the points is added as a basis vector.

  @throws std::exception if `set` is empty.
  @throws std::exception if `tol < 0`.

  For several subclasses of ConvexSet, there is a closed-form computation (or
  more efficient numerical computation) that is preferred.
  - AffineBall: Can be computed via a rank-revealing decomposition; `tol` is
  used as the numerical tolerance for the rank of the matrix. Pass
  `std::nullopt` for `tol` to use Eigen's automatic tolerance computation.
  - AffineSubspace: Equivalent to the copy-constructor; `tol` is ignored.
  - CartesianProduct: Can compute the affine hull of each factor individually;
  `tol` is propagated to the constituent calls. (This is not done if the
  Cartesian product has an associated affine transformation.)
  - Hyperellipsoid: Always equal to the whole ambient space; `tol` is ignored.
  - Hyperrectangle: Can be computed in closed-form; `tol` has the same meaning
  as in the generic affine hull computation.
  - Point: Can be computed in closed-form; `tol` is ignored. This also
  encompasses sets which are obviously a singleton point, as determined via
  MaybeGetPoint.
  - VPolytope: Can be computed via a singular value decomposition; `tol` is
  used as the numerical tolerance for the rank of the matrix. Pass
  `std::nullopt` for `tol` to use Eigen's automatic tolerance computation. */
  explicit AffineSubspace(const ConvexSet& set,
                          std::optional<double> tol = std::nullopt);

  ~AffineSubspace() final;

  /** Returns the basis in an n-by-m matrix, where n is the ambient dimension,
  and m is the number of vectors in the basis. */
  const Eigen::MatrixXd& basis() const { return basis_; }

  /** Returns the translation as a length n vector. */
  const Eigen::VectorXd& translation() const { return translation_; }

  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    ConvexSet::Serialize(a);
    a->Visit(MakeNameValue("basis", &basis_));
    a->Visit(MakeNameValue("translation", &translation_));
  }

  /** Returns the affine dimension of this set. For an affine subspace,
  this is simply the number of columns in the basis_ matrix. A point will
  have affine dimension zero. */
  int AffineDimension() const { return basis_.cols(); }

  /** Given a point x in the standard basis of the ambient space, returns the
  coordinates of x in the basis of the AffineSubspace, with the zero point at
  translation_. The component of x that is orthogonal to the AffineSubspace (if
  it exists) is discarded, so ToGlobalCoordinates(ToLocalCoordinates(x)) is
  equivalent to Project(x). Note that if the AffineSubspace is a point, the
  basis is empty, so the local coordinates will also be empty (and returned as a
  length-zero vector). Each column of the input should be a vector in the
  ambient space, and the corresponding column of the output will be its
  representation in the local coordinates of the affine subspace.
  @pre x.rows() == ambient_dimension() */
  Eigen::MatrixXd ToLocalCoordinates(
      const Eigen::Ref<const Eigen::MatrixXd>& x) const;

  /** Given a point y in the basis of the AffineSubspace, with the zero point
  at translation_, returns the coordinates of y in the standard basis of the
  ambient space. If the AffineSubspace is a point, it has an empty basis, so the
  only possible local coordinates are also empty (and should be passed in as a
  length-zero vector). Each column of the input should be a vector in the
  affine subspace, represented in its local coordinates, and the corresponding
  column of the output will be its representation in the coordinate system of
  the ambient space.
  @pre y.rows() == AffineDimension() */
  Eigen::MatrixXd ToGlobalCoordinates(
      const Eigen::Ref<const Eigen::MatrixXd>& y) const;

  /** Returns `true` if `this` AffineSubspace is contained in `other`. This is
  computed by checking if `translation()` is in `other` and then checking if
  each basis vector is in the span of the basis of `other`. The latter step
  requires finding a least-squares solution, so a nonzero tolerance (`tol`) is
  almost always necessary. (You may have to adjust the default tolerance
  depending on the dimension of your space and the scale of your basis vectors.)
  */
  bool ContainedIn(const AffineSubspace& other, double tol = 1e-15) const;

  /** Returns true if the two AffineSubspaces describe the same set, by checking
  that each set is contained in the other. */
  bool IsNearlyEqualTo(const AffineSubspace& other, double tol = 1e-15) const;

  /** Returns an orthonormal basis of the vector subspace which is orthogonal
  to this AffineSubspace.*/
  Eigen::MatrixXd OrthogonalComplementBasis() const;

  /** An AffineSubspace is bounded if and only if it is zero-dimensional (i.e.,
  a point).
  @param parallelism Ignored -- no parallelization is used.
  @note See @ref ConvexSet::IsBounded "parent class's documentation" for more
  details. */
  using ConvexSet::IsBounded;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  std::optional<bool> DoIsBoundedShortcut() const final;

  bool DoIsEmpty() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final;

  std::optional<bool> DoPointInSetShortcut(
      const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const final;

  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  std::unique_ptr<ConvexSet> DoAffineHullShortcut(
      std::optional<double>) const final;

  double DoCalcVolume() const final;

  std::vector<std::optional<double>> DoProjectionShortcut(
      const Eigen::Ref<const Eigen::MatrixXd>& points,
      EigenPtr<Eigen::MatrixXd> projected_points) const final;

  // Note, we store the original basis as given, plus the QR decomposition, for
  // later use in many of the associated methods. We do not store this if
  // the basis is empty (i.e. the affine subspace is a point).
  Eigen::MatrixXd basis_;
  Eigen::VectorXd translation_;
  std::optional<Eigen::ColPivHouseholderQR<Eigen::MatrixXd>> basis_decomp_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
