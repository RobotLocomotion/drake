#include "drake/geometry/optimization/point.h"

#include <memory>

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

Point::Point() : Point(VectorXd(0)) {}

Point::Point(const Eigen::Ref<const VectorXd>& x)
    : ConvexSet(x.size(), true), x_(x) {}

Point::Point(const QueryObject<double>& query_object, GeometryId geometry_id,
             std::optional<FrameId> reference_frame,
             double maximum_allowable_radius)
    : ConvexSet(3, true) {
  const Shape& shape = query_object.inspector().GetShape(geometry_id);
  if (shape.type_name() != "Sphere") {
    throw std::logic_error(fmt::format(
        "Point(geometry_id={}, ...) cannot convert a {}, only a Sphere",
        geometry_id, shape));
  }
  const Sphere& sphere = dynamic_cast<const Sphere&>(shape);
  if (sphere.radius() > maximum_allowable_radius) {
    throw std::runtime_error(
        fmt::format("GeometryID {} has a radius {} is larger than the "
                    "specified maximum_allowable_radius: {}.",
                    geometry_id, sphere.radius(), maximum_allowable_radius));
  }
  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_EG = X_WE.InvertAndCompose(X_WG);
  x_ = X_EG.translation();
}

Point::~Point() = default;

void Point::set_x(const Eigen::Ref<const VectorXd>& x) {
  DRAKE_THROW_UNLESS(x.size() == x_.size());
  x_ = x;
}

std::unique_ptr<ConvexSet> Point::DoClone() const {
  return std::make_unique<Point>(*this);
}

std::optional<bool> Point::DoIsBoundedShortcut() const {
  return true;
}

bool Point::DoIsEmpty() const {
  return false;
}

std::optional<VectorXd> Point::DoMaybeGetPoint() const {
  return x_;
}

std::optional<bool> Point::DoPointInSetShortcut(
    const Eigen::Ref<const VectorXd>& x, double tol) const {
  return is_approx_equal_abstol(x, x_, tol);
}

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
Point::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  VectorX<Variable> new_vars;
  std::vector<Binding<Constraint>> new_constraints;
  new_constraints.push_back(prog->AddBoundingBoxConstraint(x_, x_, x));
  return {std::move(new_vars), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
Point::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  // x == t*x_.
  const int n = ambient_dimension();
  MatrixXd Aeq(n, n + 1);
  Aeq.leftCols(n) = MatrixXd::Identity(n, n);
  Aeq.col(n) = -x_;
  constraints.emplace_back(prog->AddLinearEqualityConstraint(
      Aeq, VectorXd::Zero(n), {x, Vector1<Variable>(t)}));
  return constraints;
}

std::vector<Binding<Constraint>>
Point::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A*x+b == (c'*t+d)*x_.
  const int n = ambient_dimension();
  const int m = x.size();
  const int p = t.size();
  MatrixXd Aeq(n, m + p);
  Aeq.leftCols(m) = A;
  Aeq.rightCols(p) = -x_ * c.transpose();
  VectorXd beq = b - d * x_;
  constraints.emplace_back(prog->AddLinearEqualityConstraint(Aeq, beq, {x, t}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
Point::DoToShapeWithPose() const {
  return std::make_pair(std::make_unique<Sphere>(0.0),
                        math::RigidTransformd(x_));
}

std::vector<std::optional<double>> Point::DoProjectionShortcut(
    const Eigen::Ref<const Eigen::MatrixXd>& points,
    EigenPtr<Eigen::MatrixXd> projected_points) const {
  *projected_points = x_ * Eigen::MatrixXd::Ones(1, points.cols());
  Eigen::VectorXd distances_eigen =
      (points - *projected_points).colwise().norm();
  std::vector<std::optional<double>> distances(
      distances_eigen.data(), distances_eigen.data() + distances_eigen.size());
  return distances;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
