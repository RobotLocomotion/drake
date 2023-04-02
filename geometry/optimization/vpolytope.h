#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hpolyhedron.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A polytope described using the vertex representation.  The set is defined as
 the convex hull of the vertices.  The vertices are not guaranteed to be in any
 particular order, nor to be minimal (some vertices could be strictly in the
 interior of the set).

 Note: Unlike the half-space representation, this
 definition means the set is always bounded (hence the name polytope, instead of
 polyhedron).

@ingroup geometry_optimization
*/
class VPolytope final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VPolytope)

  /** Constructs the polytope from a d-by-n matrix, where d is the ambient
  dimension, and n is the number of vertices.  The vertices do not have to be
  ordered, nor minimal (they can contain points inside their convex hull).
  @pydrake_mkdoc_identifier{vertices} */
  explicit VPolytope(const Eigen::Ref<const Eigen::MatrixXd>& vertices);

  /** Constructs the polytope from a bounded polyhedron (using Qhull).
  @throws std::runtime_error if H is unbounded or if Qhull terminates with an
  error.
  @pydrake_mkdoc_identifier{hpolyhedron} */
  explicit VPolytope(const HPolyhedron& H);

  /** Constructs the polytope from a SceneGraph geometry.
  @pydrake_mkdoc_identifier{scenegraph} */
  VPolytope(const QueryObject<double>& query_object, GeometryId geometry_id,
            std::optional<FrameId> reference_frame = std::nullopt);

  ~VPolytope() final;

  /** Creates a new VPolytope whose vertices are guaranteed to be minimal,
  i.e. if we remove any point from its vertices, then the convex hull of the
  remaining vertices is a strict subset of the polytope. In the 2D case
  the vertices of the new VPolytope are ordered counter-clockwise from
  the negative X axis. For all other cases an order is not guaranteed.
  */
  VPolytope GetMinimalRepresentation() const;

  /** Returns true if the point is within @p tol of the set under the L∞-norm.
   Note: This requires the solution of a linear program; the achievable
   tolerance may be dependent on your specific solver and solver parameters.
   @see ConvexSet::set_solver
   */
  using ConvexSet::PointInSet;

  /** Returns the vertices in a d-by-n matrix, where d is the ambient dimension,
  and n is the number of vertices. */
  const Eigen::MatrixXd& vertices() const { return vertices_; }

  /** Constructs a polyhedron as an axis-aligned box from the lower and upper
   * corners. */
  static VPolytope MakeBox(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Constructs the L∞-norm unit box in @p dim dimensions, {x | |x|∞ <= 1 }.
  This is an axis-aligned box, centered at the origin, with edge length 2. */
  static VPolytope MakeUnitBox(int dim);

  /**
   * Computes the volume of this V-Polytope.
   * @note this function calls qhull to compute the volume.
   */
  [[nodiscard]] double CalcVolume() const;

  /**
   * Uses qhull to compute the Delaunay triangulation and then writes the
   * vertices and faces to `filename` in the Wavefront Obj format.
   */
  void WriteObj(const std::string& filename) const;

 private:
  bool DoIsBounded() const { return true; }

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraints(
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

  // Implement support shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box& box, void* data) final;
  void ImplementGeometry(const Convex& convex, void* data) final;

  Eigen::MatrixXd vertices_;
};

/** Obtain all the vertices stored in the convex object.
 * @retval vertices. Each column of `vertices` is a vertex. We don't impose any
 * specific order on the vertices. The vertices are expressed in the convex
 * shape's own frame.
 */
[[nodiscard]] Eigen::MatrixXd GetVertices(const Convex& convex);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
