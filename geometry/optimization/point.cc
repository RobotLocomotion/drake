#include "drake/geometry/optimization/point.h"

#include <limits>
#include <memory>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

#include "drake/common/is_approx_equal_abstol.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using std::sqrt;
using symbolic::Variable;

Point::Point(const QueryObject<double>& query_object, GeometryId geometry_id,
             std::optional<FrameId> expressed_in,
             double maximum_allowable_radius)
    : ConvexSet(&ConvexSetCloner<Point>, 3) {
  double radius = -1.0;
  query_object.inspector().GetShape(geometry_id).Reify(this, &radius);
  DRAKE_DEMAND(radius <= maximum_allowable_radius);

  const RigidTransformd X_WE = expressed_in
                                   ? query_object.GetPoseInWorld(*expressed_in)
                                   : RigidTransformd::Identity();
  const RigidTransformd X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_EG = X_WE.InvertAndCompose(X_WG);
  x_ = X_EG.translation();
}

bool Point::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                         double tol) const {
  return is_approx_equal_abstol(x, x_, tol);
}

void Point::DoAddPointInSetConstraint(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x) const {
  prog->AddBoundingBoxConstraint(x_, x_, x);
}

std::vector<solvers::Binding<solvers::Constraint>>
Point::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  // x == t*x_.
  const int n = ambient_dimension();
  MatrixXd Aeq(n, n + 1);
  Aeq.leftCols(n) = MatrixXd::Identity(n, n);
  Aeq.col(n) = -x_;
  constraints.emplace_back(prog->AddLinearEqualityConstraint(
      Aeq, VectorXd::Zero(n), {x, Vector1<Variable>(t)}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
Point::DoToShapeWithPose() const {
  return std::make_pair(std::make_unique<Sphere>(0.0),
                        math::RigidTransformd(x_));
}

void Point::ImplementGeometry(const Sphere& sphere, void* data) {
  double* radius = reinterpret_cast<double*>(data);
  *radius = sphere.radius();
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
