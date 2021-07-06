#include "drake/geometry/optimization/vpolytope.h"

#include <limits>
#include <memory>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

VPolytope::VPolytope(const Eigen::Ref<const Eigen::MatrixXd>& vertices)
    : ConvexSet(&ConvexSetCloner<VPolytope>, vertices.rows()),
      vertices_{vertices} {}

VPolytope::VPolytope(const QueryObject<double>& query_object,
                     GeometryId geometry_id,
                     std::optional<FrameId> reference_frame)
    : ConvexSet(&ConvexSetCloner<VPolytope>, 3) {
  Matrix3Xd vertices;
  query_object.inspector().GetShape(geometry_id).Reify(this, &vertices);

  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_EG = X_WE.InvertAndCompose(X_WG);
  vertices_ = X_EG * vertices;
}

VPolytope::~VPolytope() = default;

VPolytope VPolytope::MakeBox(const Eigen::Ref<const VectorXd>& lb,
                             const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == ub.size());
  DRAKE_DEMAND((lb.array() <= ub.array()).all());
  const int n = lb.size();
  DRAKE_DEMAND(n > 0);
  // Make sure that n is small enough to avoid overflow
  DRAKE_DEMAND(n <= static_cast<int>(sizeof(Eigen::Index)) * 8 - 2);
  // Create all 2^n vertices.
  MatrixXd vertices = lb.replicate(1, 1 << n);
  for (int i = 1; i < vertices.cols(); ++i) {
    for (int j = 0; j < n; j++) {
      if (i >> j & 1) {
        vertices(j, i) = ub[j];
      }
    }
  }
  return VPolytope(vertices);
}

VPolytope VPolytope::MakeUnitBox(int dim) {
  return MakeBox(VectorXd::Constant(dim, -1.0), VectorXd::Constant(dim, 1.0));
}

bool VPolytope::DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                             double tol) const {
  const int n = ambient_dimension();
  const int m = vertices_.cols();
  const double inf = std::numeric_limits<double>::infinity();
  // min z s.t. |(v α - x)ᵢ| ≤ z, αᵢ ≥ 0, ∑ᵢ αᵢ = 1.
  MathematicalProgram prog;
  VectorXDecisionVariable z = prog.NewContinuousVariables<1>("z");
  VectorXDecisionVariable alpha = prog.NewContinuousVariables(m, "a");
  // min z
  prog.AddLinearCost(Vector1d(1.0), z);
  // |(v α - x)ᵢ| ≤ z as -z ≤ vᵢ α - xᵢ ≤ z as vᵢ α - z ≤ xᵢ && xᵢ ≤ vᵢ α + z
  MatrixXd A(n, m + 1);
  A.leftCols(m) = vertices_;
  A.col(m) = -VectorXd::Ones(n);
  prog.AddLinearConstraint(A, VectorXd::Constant(n, -inf), x, {alpha, z});
  A.col(m) = VectorXd::Ones(n);
  prog.AddLinearConstraint(A, x, VectorXd::Constant(n, inf), {alpha, z});
  // 0 ≤ αᵢ ≤ 1.  The one is redundant, but may be better than inf for some
  // solvers.
  prog.AddBoundingBoxConstraint(0, 1.0, alpha);
  // ∑ᵢ αᵢ = 1
  prog.AddLinearEqualityConstraint(RowVectorXd::Ones(m), 1.0, alpha);
  auto result = solvers::Solve(prog);
  // The formulation was chosen so that it always has a feasible solution.
  DRAKE_DEMAND(result.is_success());
  // To decouple the solver tolerance from the requested tolerance, we solve the
  // LP, but then evaluate the constraints ourselves.
  // Note: The max(alpha, 0) and normalization were required for Gurobi.
  const VectorXd alpha_sol = result.GetSolution(alpha).cwiseMax(0);
  const VectorXd x_sol = vertices_ * alpha_sol / (alpha_sol.sum());
  return is_approx_equal_abstol(x, x_sol, tol);
}

void VPolytope::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x) const {
  const int n = ambient_dimension();
  const int m = vertices_.cols();
  VectorXDecisionVariable alpha = prog->NewContinuousVariables(m, "a");
  // 0 ≤ αᵢ ≤ 1.  The one is redundant, but may be better than inf for some
  // solvers.
  prog->AddBoundingBoxConstraint(0, 1.0, alpha);
  // v α - x = 0.
  Eigen::MatrixXd A(n, m + n);
  A.leftCols(m) = vertices_;
  A.rightCols(n) = -Eigen::MatrixXd::Identity(n, n);
  prog->AddLinearEqualityConstraint(A, VectorXd::Zero(n), {alpha, x});
  // ∑ αᵢ = 1.
  prog->AddLinearEqualityConstraint(RowVectorXd::Ones(m), 1.0, alpha);
}

std::vector<solvers::Binding<solvers::Constraint>>
VPolytope::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const symbolic::Variable& t) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
  const int n = ambient_dimension();
  const int m = vertices_.cols();
  VectorXDecisionVariable alpha = prog->NewContinuousVariables(m, "a");
  // αᵢ ≥ 0.
  constraints.emplace_back(prog->AddBoundingBoxConstraint(
      0, std::numeric_limits<double>::infinity(), alpha));
  // v α = x.
  MatrixXd A(n, m + n);
  A.leftCols(m) = vertices_;
  A.rightCols(n) = -MatrixXd::Identity(n, n);
  constraints.emplace_back(
      prog->AddLinearEqualityConstraint(A, VectorXd::Zero(n), {alpha, x}));

  // ∑ αᵢ = t.
  RowVectorXd a = RowVectorXd::Ones(m + 1);
  a[m] = -1;
  constraints.emplace_back(
      prog->AddLinearEqualityConstraint(a, 0.0, {alpha, Vector1<Variable>(t)}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
VPolytope::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for VPolytope.  Implementing "
      "this will likely require additional support from the Convex shape "
      "class (to support in-memory mesh data, or file I/O).");
}

void VPolytope::ImplementGeometry(const Box& box, void* data) {
  const double x = box.width() / 2.0;
  const double y = box.depth() / 2.0;
  const double z = box.height() / 2.0;
  Matrix3Xd* vertices = static_cast<Matrix3Xd*>(data);
  vertices->resize(3, 8);
  // clang-format off
  *vertices << -x,  x,  x, -x, -x,  x,  x, -x,
                y,  y, -y, -y, -y, -y,  y,  y,
               -z, -z, -z, -z,  z,  z,  z,  z;
  // clang-format on
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
