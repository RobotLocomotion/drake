#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A polytope described using the vertex representation.  The set is defined as
the convex hull of the vertices.  The vertices are not guaranteed to be in any
particular order, nor to be minimal (some vertices could be strictly in the
interior of the set).

Note: Unlike the half-space representation, this definition means the set is
always bounded (hence the name polytope, instead of polyhedron).

A VPolytope is empty if and only if it is composed of zero vertices, i.e.,
if vertices_.cols() == 0. This includes the zero-dimensional case. If
vertices_.rows() == 0 but vertices_.cols() > 0, we treat this as having one or
more copies of 0 in the zero-dimensional vector space {0}. If vertices_.rows()
and vertices_.cols() are zero, we treat this as no points in {0}, which is
empty.

@ingroup geometry_optimization */
class VPolytope final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VPolytope);

  /** Constructs a set with no vertices in the zero-dimensional space, which is
  empty (by convention). */
  VPolytope();

  /** Constructs the polytope from a d-by-n matrix, where d is the ambient
  dimension, and n is the number of vertices.  The vertices do not have to be
  ordered, nor minimal (they can contain points inside their convex hull).
  @pydrake_mkdoc_identifier{vertices} */
  explicit VPolytope(const Eigen::Ref<const Eigen::MatrixXd>& vertices);

  /** Constructs the polytope from a bounded polyhedron (using Qhull). If the
  HPolyhedron is not full-dimensional, we perform computations in a coordinate
  system of its affine hull. `tol` specifies the numerical tolerance used in the
  computation of the affine hull. See the documentation of AffineSubspace for
  more details. A loose tolerance is necessary for the built-in solvers, but a
  tighter tolerance can be used with commercial solvers (e.g. Gurobi and Mosek).
  @throws std::runtime_error if H is unbounded or if Qhull terminates with an
  error.
  @pydrake_mkdoc_identifier{hpolyhedron} */
  explicit VPolytope(const HPolyhedron& H, double tol = 1e-9);

  /** Constructs the polytope from a SceneGraph geometry.
  @pydrake_mkdoc_identifier{scenegraph} */
  VPolytope(const QueryObject<double>& query_object, GeometryId geometry_id,
            std::optional<FrameId> reference_frame = std::nullopt);

  ~VPolytope() final;

  /** Creates a new VPolytope whose vertices are guaranteed to be minimal, i.e.,
  if we remove any point from its vertices, then the convex hull of the
  remaining vertices is a strict subset of the polytope. In the 2D case the
  vertices of the new VPolytope are ordered counter-clockwise from the negative
  X axis. For all other cases an order is not guaranteed. If the
  VPolytope is not full-dimensional, we perform computations in a coordinate
  system of its affine hull. `tol` specifies the numerical tolerance used in the
  computation of the affine hull.*/
  VPolytope GetMinimalRepresentation(double tol = 1e-9) const;

  /** Returns true if the point is within `tol` of the set under the L∞-norm.
  Note: This requires the solution of a linear program; the achievable tolerance
  may be dependent on your specific solver and solver parameters.
  @see ConvexSet::set_solver */
  using ConvexSet::PointInSet;

  /** Returns the vertices in a d-by-n matrix, where d is the ambient dimension,
  and n is the number of vertices. */
  const Eigen::MatrixXd& vertices() const { return vertices_; }

  /** Constructs a polyhedron as an axis-aligned box from the lower and upper
  corners. */
  static VPolytope MakeBox(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Constructs the L∞-norm unit box in `dim` dimensions, {x | |x|∞ <= 1 }.
  This is an axis-aligned box, centered at the origin, with edge length 2. */
  static VPolytope MakeUnitBox(int dim);

  /** Uses qhull to compute the Delaunay triangulation and then writes the
  vertices and faces to `filename` in the Wavefront Obj format. Note that the
  extension `.obj` is not automatically added to the `filename`.
  @pre ambient_dimension() == 3. */
  void WriteObj(const std::filesystem::path& filename) const;

  /** Creates a geometry::Convex shape using the vertices of this VPolytope. The
  convex_label is passed as the 'label' of the Convex object.
  @pre ambient_dimension() == 3. */
  Convex ToShapeConvex(
      const std::string& convex_label = "convex_from_vpolytope") const;

  /** Computes the volume of this V-Polytope.
  @note this function calls qhull to compute the volume. */
  using ConvexSet::CalcVolume;

  /** Every VPolytope is bounded by construction.
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

  std::unique_ptr<ConvexSet> DoAffineHullShortcut(
      std::optional<double> tol) const final;

  double DoCalcVolume() const final;

  Eigen::MatrixXd vertices_;
};

/** Obtains all the vertices stored in the convex object.
@retval vertices. Each column of `vertices` is a vertex. We don't impose any
specific order on the vertices. The vertices are expressed in the convex shape's
own frame. */
[[nodiscard]] Eigen::MatrixXd GetVertices(const Convex& convex);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
