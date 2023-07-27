#pragma once

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(AffineSubspace)

  /** Constructs a default (zero-dimensional, nonempty) affine subspace. */
  AffineSubspace();

  /** Constructs the affine subspace from an n-by-m matrix describing the basis,
  where n is the ambient dimension, and m is the dimension of the subspace, and
  from an n-dimensional vector describing the translation. The set is {x | x =
  translation + basis*y, y ∈ Rᵐ} The columns must be linearly independent.
  @pre basis.rows() == translation.size(). */
  explicit AffineSubspace(const Eigen::Ref<const Eigen::MatrixXd>& basis,
                          const Eigen::Ref<const Eigen::VectorXd>& translation);

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

  /** Computes the orthogonal projection of x onto the AffineSubspace. This
  is achieved by finding the least squares solution y for y to x = translation_
  + basis_*y, and returning translation_ + basis_*y.
  @pre x.size() == ambient_dimension() */
  Eigen::MatrixXd Project(const Eigen::Ref<const Eigen::MatrixXd>& x) const;

  /** Given a point x in the standard basis of the ambient space, returns the
  coordinates of x in the basis of the AffineSubspace, with the zero point at
  translation_. The component of x that is orthogonal to the AffineSubspace (if
  it exists) is discarded, so ToGlobalCoordinates(ToLocalCoordinates(x)) is
  equivalent to Project(x). Note that if the AffineSubspace is a point, the
  basis is empty, so the local coordinates will also be empty (and returned as a
  length-zero vector).
  @pre x.size() == ambient_dimension() */
  Eigen::MatrixXd ToLocalCoordinates(
      const Eigen::Ref<const Eigen::MatrixXd>& x) const;

  /** Given a point y in the basis of the AffineSubspace, with the zero point
  at translation_, returns the coordinates of y in the standard basis of the
  ambient space. If the AffineSubspace is a point, it has an empty basis, so the
  only possible local coordinates are also empty (and should be passed in as a
  length-zero vector).
  @pre y.size() == AffineDimension() */
  Eigen::MatrixXd ToGlobalCoordinates(
      const Eigen::Ref<const Eigen::MatrixXd>& y) const;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  bool DoIsBounded() const final;

  bool DoIsEmpty() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetPoint() const final;

  std::optional<Eigen::VectorXd> DoMaybeGetFeasiblePoint() const final;

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

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
