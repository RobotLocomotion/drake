#include "drake/geometry/optimization/cartesian_product.h"

#include <memory>

#include <fmt/format.h>

#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::Solve;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

namespace {

int sum_ambient_dimensions(const ConvexSets& sets) {
  int dim = 0;
  for (const auto& s : sets) {
    dim += s->ambient_dimension();
  }
  return dim;
}

}  // namespace

CartesianProduct::CartesianProduct(const ConvexSets& sets)
    : ConvexSet(&ConvexSetCloner<CartesianProduct>,
                       sum_ambient_dimensions(sets)),
      sets_{sets} {
  for (const auto& s : sets_) {
    DRAKE_DEMAND(s->IsBounded());
  }
}

CartesianProduct::CartesianProduct(const ConvexSet& setA, const ConvexSet& setB)
    : ConvexSet(&ConvexSetCloner<CartesianProduct>,
                setA.ambient_dimension() + setB.ambient_dimension()) {
  DRAKE_DEMAND(setA.IsBounded());
  DRAKE_DEMAND(setB.IsBounded());
  sets_.emplace_back(setA.Clone());
  sets_.emplace_back(setB.Clone());
}

CartesianProduct::CartesianProduct(const ConvexSets& sets,
                                   const Eigen::Ref<const Eigen::MatrixXd>& A,
                                   const Eigen::Ref<const Eigen::VectorXd>& b)
    : ConvexSet(&ConvexSetCloner<CartesianProduct>,
                A.cols()),
      sets_{sets}, A_{A}, b_{b} {
  DRAKE_DEMAND(A_->rows() == b_->rows());
  DRAKE_DEMAND(A_->rows() == sum_ambient_dimensions(sets));
  DRAKE_DEMAND(A_->colPivHouseholderQr().rank() == A_->cols());
  for (const auto& s : sets_) {
    DRAKE_DEMAND(s->IsBounded());
  }
}

CartesianProduct::CartesianProduct(const QueryObject<double>& query_object,
                                   GeometryId geometry_id,
                                   std::optional<FrameId> reference_frame)
    : ConvexSet(&ConvexSetCloner<CartesianProduct>, 3) {
  Cylinder cylinder(1., 1.);
  query_object.inspector()
      .GetShape(geometry_id)
      .Reify(this, &cylinder);

  // Make the cylinder out of a circle (2D sphere) and a line segment (1D box).
  sets_.emplace_back(
      Hyperellipsoid::MakeHypersphere(cylinder.radius(), Vector2d::Zero())
          .Clone());
  sets_.emplace_back(HPolyhedron
                         ::MakeBox(Vector1d{-cylinder.length() / 2.0},
                                  Vector1d{cylinder.length() / 2.0})
                         .Clone());

  const RigidTransformd X_WF =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GF = X_WG.InvertAndCompose(X_WF);

  A_ = X_GF.rotation().matrix();
  b_ = X_GF.translation();
}

CartesianProduct::~CartesianProduct() = default;

const ConvexSet& CartesianProduct::factor(int index) const {
  DRAKE_DEMAND(0 <= index && index < static_cast<int>(sets_.size()));
  return *sets_[index];
}

bool CartesianProduct::DoIsBounded() const {
  // Note: The constructor enforces that A_ is full column rank, and that all
  // sets are bounded.
  return true;
}

bool CartesianProduct::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                                double tol) const {
  int index = 0;
  VectorXd y = A_ ? (*A_)*x + (*b_) : x;
  for (const auto& s : sets_) {
    if (!s->PointInSet(y.segment(index, s->ambient_dimension()),
                       tol)) {
      return false;
    }
    index += s->ambient_dimension();
  }
  return true;
}

void CartesianProduct::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x) const {
  VectorXDecisionVariable y;
  if (A_) {
    // Note: The constructor enforces that A_ is full column rank.
    y = prog->NewContinuousVariables(A_->rows(), "y");
    // y = Ax + b, or [I,-A]*[y;x] = b.
    MatrixXd Aeq = MatrixXd::Identity(A_->rows(), A_->rows() + A_->cols());
    Aeq.rightCols(A_->cols()) = -(*A_);
    prog->AddLinearEqualityConstraint(Aeq, (*b_), {y, x});
  } else {
    y = x;
  }
  int index = 0;
  for (const auto& s : sets_) {
    s->AddPointInSetConstraints(
        prog, y.segment(index, s->ambient_dimension()));
    index += s->ambient_dimension();
  }
}

std::vector<Binding<Constraint>>
CartesianProduct::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  VectorXDecisionVariable y;
  if (A_) {
    y = prog->NewContinuousVariables(A_->rows(), "y");
    // y = Ax + t*b, or [I,-b,-A]*[y;t;x] = 0.
    MatrixXd Aeq = MatrixXd::Identity(A_->rows(), A_->rows() + A_->cols() + 1);
    Aeq.col(A_->rows()) = -(*b_);
    Aeq.rightCols(A_->cols()) = -(*A_);
    constraints.emplace_back(prog->AddLinearEqualityConstraint(
        Aeq, VectorXd::Zero(A_->rows()), {y, Vector1<Variable>{t}, x}));
  } else {
    y = x;
  }
  int index = 0;
  for (const auto& s : sets_) {
    auto new_constraints = s->AddPointInNonnegativeScalingConstraints(
        prog, y.segment(index, s->ambient_dimension()), t);
    index += s->ambient_dimension();
    constraints.insert(constraints.end(),
                       std::make_move_iterator(new_constraints.begin()),
                       std::make_move_iterator(new_constraints.end()));
  }
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
CartesianProduct::DoToShapeWithPose() const {
  // TODO(russt): Consider handling Cylinder as a (very) special case.
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for CartesianProduct.");
}

void CartesianProduct::ImplementGeometry(const Cylinder& cylinder, void* data) {
  Cylinder* c = static_cast<Cylinder*>(data);
  *c = cylinder;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
