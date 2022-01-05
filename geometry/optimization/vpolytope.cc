#include "drake/geometry/optimization/vpolytope.h"

#include <limits>
#include <memory>

#include <fmt/format.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullVertexSet.h>

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

VPolytope::VPolytope(const HPolyhedron& hpoly)
    : ConvexSet(&ConvexSetCloner<VPolytope>, hpoly.ambient_dimension()) {
  DRAKE_THROW_UNLESS(hpoly.IsBounded());

  Eigen::MatrixXd coeffs(hpoly.A().rows(), hpoly.A().cols() + 1);
  coeffs.leftCols(hpoly.A().cols()) = hpoly.A();
  coeffs.col(hpoly.A().cols()) = -hpoly.b();

  Eigen::MatrixXd coeffs_t = coeffs.transpose();
  std::vector<double> flat_coeffs;
  flat_coeffs.resize(coeffs_t.size());
  Eigen::VectorXd::Map(&flat_coeffs[0], coeffs_t.size()) =
      Eigen::VectorXd::Map(coeffs_t.data(), coeffs_t.size());

  Eigen::VectorXd eigen_center = hpoly.ChebyshevCenter();
  std::vector<double> center;
  center.resize(eigen_center.size());
  Eigen::VectorXd::Map(&center[0], eigen_center.size()) = eigen_center;

  orgQhull::Qhull qhull;
  qhull.setFeasiblePoint(orgQhull::Coordinates(center));
  //  By default, Qhull takes in a sequence of vertices and generates a convex
  //  hull from them. Alternatively it can generate the convex hull from a
  //  sequence of halfspaces (requested via the "H" argument). In that case the
  //  inputs are overloaded with the `pointDimension` representing the dimension
  //  the convex hull exists in plus the offset and the `pointCount`
  //  representing the number of faces. Slightly more documentation can be found
  //  here: http://www.qhull.org/html/qhull.htm.
  qhull.runQhull("", hpoly.A().cols() + 1, hpoly.A().rows(), flat_coeffs.data(),
                 "H");
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull terminated with status {} and  message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }

  // Qhull flips some notation when you use the halfspace intersection:
  // http://www.qhull.org/html/qh-code.htm#facet-cpp . Each facet from qhull
  // represents an intersection between halfspaces of the H polyhedron.
  // However, I could not figure out if each QhullFacet stored the exact
  // location of the intersection (i.e. the vertex). Instead, this code takes
  // each intersection of hyperplanes (QhullFacet), pulls out the hyperplanes
  // that are part of the intersection (facet.vertices()) and solves for the
  // vertex that lies at the intersection of these hyperplanes.
  vertices_.resize(hpoly.ambient_dimension(), qhull.facetCount());
  int ii = 0;
  for (const auto& facet : qhull.facetList()) {
    auto incident_hyperplanes = facet.vertices();
    Eigen::MatrixXd vertex_A(incident_hyperplanes.count(),
                             hpoly.ambient_dimension());
    for (int jj = 0; jj < incident_hyperplanes.count(); jj++) {
      std::vector<double> hyperplane =
          incident_hyperplanes.at(jj).point().toStdVector();
      vertex_A.row(jj) = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(
          hyperplane.data(), hyperplane.size());
    }
    vertices_.col(ii) = vertex_A.partialPivLu().solve(Eigen::VectorXd::Ones(
                            incident_hyperplanes.count())) +
                        eigen_center;
    ii++;
  }
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
