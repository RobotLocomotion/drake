#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"

namespace drake {
namespace geometry {
namespace optimization {

/** A polytope described using the vertex representation.  The set is defined as
 the convex hull of the vertices.

 Note: Unlike the half-space representation, this
 definition means the set is always bounded (hence the name polytope, instead of
 polyhedron). */
class VPolytope final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(VPolytope)

  explicit VPolytope(const Eigen::Ref<const Eigen::MatrixXd>& vertices);

  VPolytope(const QueryObject<double>& query_object, GeometryId geometry_id,
            std::optional<FrameId> expressed_in = std::nullopt);

  ~VPolytope() final;

  /** Returns true if the point is within @p tol of the set under the L∞-norm.
   Note: This requires the solution of a linear program; the achievable
   tolerance may be dependent on your specific solver and solver parameters.
   @see ConvexSet::set_solver
   */
  using ConvexSet::PointInSet;

  const Eigen::MatrixXd& vertices() const { return vertices_; }

  /** Constructs a polyhedron as an axis-aligned box from the lower and upper
   * corners. */
  static VPolytope MakeBox(const Eigen::Ref<const Eigen::VectorXd>& lb,
                           const Eigen::Ref<const Eigen::VectorXd>& ub);

  /** Constructs the L∞-norm unit box in @p dim dimensions, {x | |x|∞ <= 1 }.
  This is an axis-aligned box, centered at the origin, with edge length 2. */
  static VPolytope MakeUnitBox(int dim);

 private:
  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraint(
      solvers::MathematicalProgram*,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>&) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  // Implement support shapes for the ShapeReifier interface.
  using ShapeReifier::ImplementGeometry;
  void ImplementGeometry(const Box& box, void* data) final;
  // TODO(russt): Support ImplementGeometry(const Convex& convex, ...), but
  // currently it would require e.g. digging ReadObjForConvex out of
  // proximity_engine.cc.

  Eigen::MatrixXd vertices_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
