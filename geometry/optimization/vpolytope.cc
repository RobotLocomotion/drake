#include "drake/geometry/optimization/vpolytope.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullVertexSet.h>

#include "drake/common/is_approx_equal_abstol.h"
#include "drake/common/overloaded.h"
#include "drake/geometry/optimization/affine_subspace.h"
#include "drake/geometry/read_obj.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::LinearCost;
using solvers::MathematicalProgram;
using solvers::VectorXDecisionVariable;
using symbolic::Variable;

namespace {

/* Given a matrix containing a set of 2D vertices, return a copy
of the matrix where the vertices are ordered counter-clockwise
from the negative X axis. */
MatrixXd OrderCounterClockwise(const MatrixXd& vertices) {
  const size_t dim = vertices.rows();
  const size_t num_vertices = vertices.cols();

  DRAKE_DEMAND(dim == 2);

  std::vector<size_t> indices(num_vertices);
  std::vector<double> angles(num_vertices);

  double center_x = 0;
  double center_y = 0;

  std::iota(indices.begin(), indices.end(), 0);

  for (const auto& i : indices) {
    center_x += vertices.col(i)[0];
    center_y += vertices.col(i)[1];
  }

  center_x /= num_vertices;
  center_y /= num_vertices;

  for (const auto& i : indices) {
    const double x = vertices.col(i)[0] - center_x;
    const double y = vertices.col(i)[1] - center_y;
    angles[i] = std::atan2(y, x);
  }

  std::sort(indices.begin(), indices.end(), [&angles](size_t a, size_t b) {
    return angles[a] > angles[b];
  });

  MatrixXd sorted_vertices(dim, num_vertices);

  for (size_t i = 0; i < num_vertices; ++i) {
    sorted_vertices.col(i) = vertices.col(indices[i]);
  }

  return sorted_vertices;
}

// Extract the vertices from a convex hull. The result is a (3, V) matrix where
// `hull` has V vertices. The hull should come from an invocation of either
// Mesh::GetConvexHull() or Convex::GetConvexHull().
MatrixXd GetConvexHullVertices(const PolygonSurfaceMesh<double>& hull) {
  Matrix3Xd vertices(3, hull.num_vertices());
  for (int vi = 0; vi < hull.num_vertices(); ++vi) {
    vertices.col(vi) = hull.vertex(vi);
  }
  return vertices;
}

}  // namespace

VPolytope::VPolytope() : VPolytope(MatrixXd(0, 0)) {}

VPolytope::VPolytope(const Eigen::Ref<const MatrixXd>& vertices)
    : ConvexSet(vertices.rows(), true), vertices_(vertices) {}

VPolytope::VPolytope(const QueryObject<double>& query_object,
                     GeometryId geometry_id,
                     std::optional<FrameId> reference_frame)
    : ConvexSet(3, true) {
  const Shape& shape = query_object.inspector().GetShape(geometry_id);
  const MatrixXd shape_vertices = shape.Visit(overloaded{
      // We only handle certain shape types.
      [](const Box& box) {
        const double x = box.width() / 2.0;
        const double y = box.depth() / 2.0;
        const double z = box.height() / 2.0;
        MatrixXd result(3, 8);
        // clang-format off
        result << -x,  x,  x, -x, -x,  x,  x, -x,
                   y,  y, -y, -y, -y, -y,  y,  y,
                  -z, -z, -z, -z,  z,  z,  z,  z;
        // clang-format on
        return result;
      },
      [](const Convex& convex) {
        return GetConvexHullVertices(convex.GetConvexHull());
      },
      [](const Mesh& mesh) {
        return GetConvexHullVertices(mesh.GetConvexHull());
      },
      [&geometry_id](const auto& unsupported) -> MatrixXd {
        throw std::logic_error(fmt::format(
            "{} (geometry_id={}) cannot be converted to a VPolytope",
            unsupported, geometry_id));
      }});
  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_EG = X_WE.InvertAndCompose(X_WG);
  vertices_ = X_EG * shape_vertices;
}

VPolytope::VPolytope(const HPolyhedron& hpoly, double tol)
    : ConvexSet(hpoly.ambient_dimension(), true) {
  // First, assert that the HPolyhedron is bounded (since a VPolytope cannot
  // be used to represent an unbounded set).
  DRAKE_THROW_UNLESS(hpoly.IsBounded());

  // Next, handle the case where the HPolyhedron is zero-dimensional.
  if (hpoly.ambient_dimension() == 0) {
    // A zero-dimensional HPolyhedron is always considered nonempty. (See
    // documentation of the HPolyhedron class.) Thus, we construct a matrix with
    // zero rows and one column, denoting a single point in the zero-dimensional
    // vector space.
    Eigen::MatrixXd points = Eigen::MatrixXd::Zero(0, 1);
    *this = VPolytope(points);
    return;
  }

  if (hpoly.ambient_dimension() == 1) {
    // In 1D, QHull doesn't work. We make a list of all hyperplanes that lower
    // bound and upper bound the polytope, and choose the "innermost" points to
    // make the VPolytope.
    const double inf = std::numeric_limits<double>::infinity();
    double supporting_above = inf;
    double supporting_below = -inf;
    for (int i = 0; i < hpoly.b().size(); ++i) {
      if (hpoly.A()(i, 0) > 0) {
        supporting_above = std::min(supporting_above, hpoly.b()(i));
      } else if (hpoly.A()(i, 0) < 0) {
        supporting_below = std::max(supporting_below, -hpoly.b()(i));
      }
    }
    // Because we know the HPolyhedron is bounded, it must have at least one
    // supporting point above and below.
    DRAKE_THROW_UNLESS(supporting_above < inf);
    DRAKE_THROW_UNLESS(supporting_below > -inf);
    if (supporting_below > supporting_above) {
      // In this case, there are a pair of infeasible inequalities, so the
      // HPolyhedron is empty.
      Eigen::Matrix<double, 1, 0> points;
      *this = VPolytope(points);
      return;
    } else {
      Eigen::Matrix<double, 1, 2> points;
      points << supporting_below, supporting_above;
      *this = VPolytope(points);
      return;
    }
  }

  // Next, handle the case where the HPolyhedron is empty.
  if (hpoly.IsEmpty()) {
    // We construct a VPolytope with ambient_dimension() rows and zero columns,
    // denoting zero points in the same dimensional space (i.e. the empty set).
    *this = VPolytope(Eigen::MatrixXd::Zero(hpoly.ambient_dimension(), 0));
    return;
  }

  // Next, handle the case where the HPolyhedron is not full dimensional.
  const AffineSubspace affine_hull(hpoly, tol);
  if (affine_hull.AffineDimension() < affine_hull.ambient_dimension()) {
    // This special case avoids the QHull error QH6023, which occurs when the
    // feasible point given to QHull is not clearly inside the HPolyhedron. If
    // the HPolyhedron is not full dimensional, this error will occur, even if
    // the point is indeed inside the HPolyhedron. To handle this, we project
    // onto the affine hull and do computations there, before lifting the
    // resulting VPolytope to the ambient space.

    // Note that QHull will not function in zero or one dimensional spaces, so
    // we handle these separately here.
    if (affine_hull.AffineDimension() == 0) {
      // If affine_hull is zero dimensional, then hpoly is just a single point.
      const auto maybe_point = hpoly.MaybeGetFeasiblePoint();
      DRAKE_DEMAND(maybe_point.has_value());
      Eigen::MatrixXd points_in(hpoly.ambient_dimension(), 1);
      points_in << maybe_point.value();
      *this = VPolytope(points_in);
      return;
    } else if (affine_hull.AffineDimension() == 1) {
      // If it's one dimensional, then it's a line segment. Then the affine hull
      // has only one basis vector, so to find the endpoints, we minimize and
      // maximize the dot product of this vector with a vector decision
      // variable, constrained to lie in the HPolyhedron.
      DRAKE_DEMAND(affine_hull.basis().cols() == 1);
      MathematicalProgram prog;
      VectorXDecisionVariable x =
          prog.NewContinuousVariables(hpoly.ambient_dimension());
      hpoly.AddPointInSetConstraints(&prog, x);
      Binding<LinearCost> objective =
          prog.AddLinearCost(affine_hull.basis().col(0), x);
      const auto result1 = solvers::Solve(prog);

      // The only reason this solve could fail is if hpoly were unbounded or
      // empty. Both cases are explicitly handled before this.
      DRAKE_DEMAND(result1.is_success());
      const Eigen::VectorXd point1 = result1.GetSolution(x);

      // Now, update the coefficients of the objective, and solve again.
      objective.evaluator()->UpdateCoefficients(-affine_hull.basis().col(0));
      const auto result2 = solvers::Solve(prog);
      DRAKE_DEMAND(result2.is_success());
      const Eigen::VectorXd point2 = result2.GetSolution(x);

      Eigen::MatrixXd vertices(hpoly.ambient_dimension(), 2);
      vertices << point1, point2;
      *this = VPolytope(vertices);
      return;
    }

    // If we have a HPolyhedron defined as {Ax <= b : x in R^n} in the ambient
    // space, and its Affine Hull is the set {Cy + d : y in R^m}, then its
    // representation in the local coordinates of the affine hull will be
    // {A(Cy + d) <= b : y in R^m}, or equivalently, {ACy <= b - Ad : y in R^m}.

    // To prevent numerical issues, we have to remove the halfspaces which are
    // parallel to the affine hull. We check this by taking the dot product of
    // each constraint vector with each basis vector. If the constraint vector
    // is orthogonal to every basis vector (within a prespecified tolerance),
    // then it defines a hyperplane that is parallel to the affine hull, and we
    // must remove it.
    Eigen::MatrixXd product = hpoly.A() * affine_hull.basis();
    const auto mask = product.rowwise().lpNorm<Eigen::Infinity>().array() > tol;
    const int count_nonzero = mask.count();
    Eigen::MatrixXd A_filtered(count_nonzero, hpoly.ambient_dimension());
    Eigen::VectorXd b_filtered(count_nonzero);
    int row = 0;
    for (int i = 0; i < hpoly.A().rows(); ++i) {
      if (mask[i]) {
        A_filtered.row(row) = hpoly.A().row(i);
        b_filtered[row] = hpoly.b()[i];
        ++row;
      }
    }

    HPolyhedron hpoly_subspace(
        A_filtered * affine_hull.basis(),
        b_filtered - A_filtered * affine_hull.translation());

    // Because hpoly_subspace is full-dimensional in the coordinates of the
    // affine subspace, we can directly convert to a VPolytope. If QHull has
    // additional errors besides the dimension one, they will be caught here.
    VPolytope vpoly_subspace(hpoly_subspace);

    // Finally, we extract the vertices of vpoly_subspace and lift them into the
    // ambient space, obtaining the desired VPolytope.
    Eigen::MatrixXd points_local = vpoly_subspace.vertices();
    Eigen::MatrixXd points_global =
        affine_hull.ToGlobalCoordinates(points_local);
    *this = VPolytope(points_global);
    return;
  }

  // Now that we know the HPolyhedron is full dimensional, we can finish the
  // various setup steps and call QHull.
  MatrixXd coeffs(hpoly.A().rows(), hpoly.A().cols() + 1);
  coeffs.leftCols(hpoly.A().cols()) = hpoly.A();
  coeffs.col(hpoly.A().cols()) = -hpoly.b();

  MatrixXd coeffs_t = coeffs.transpose();
  std::vector<double> flat_coeffs;
  flat_coeffs.resize(coeffs_t.size());
  VectorXd::Map(&flat_coeffs[0], coeffs_t.size()) =
      VectorXd::Map(coeffs_t.data(), coeffs_t.size());

  VectorXd eigen_center = hpoly.ChebyshevCenter();
  std::vector<double> center;
  center.resize(eigen_center.size());
  VectorXd::Map(&center[0], eigen_center.size()) = eigen_center;

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
    DRAKE_THROW_UNLESS(incident_hyperplanes.count() >=
                       hpoly.ambient_dimension());
    MatrixXd vertex_A(incident_hyperplanes.count(), hpoly.ambient_dimension());
    for (int jj = 0; jj < incident_hyperplanes.count(); jj++) {
      std::vector<double> hyperplane =
          incident_hyperplanes.at(jj).point().toStdVector();
      vertex_A.row(jj) = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(
          hyperplane.data(), hyperplane.size());
    }
    // The matrix vertex_A is guaranteed to be full column rank and therefore
    // the ColPivHouseholderQR factorization should be stable.
    const Eigen::ColPivHouseholderQR<MatrixXd> QR(vertex_A);
    vertices_.col(ii) =
        QR.solve(VectorXd::Ones(incident_hyperplanes.count())) + eigen_center;
    ii++;
  }
}

VPolytope::~VPolytope() = default;

VPolytope VPolytope::MakeBox(const Eigen::Ref<const VectorXd>& lb,
                             const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_THROW_UNLESS(lb.size() == ub.size());
  DRAKE_THROW_UNLESS((lb.array() <= ub.array()).all());
  const int n = lb.size();
  DRAKE_THROW_UNLESS(n > 0);
  // Make sure that n is small enough to avoid overflow
  DRAKE_THROW_UNLESS(n <= static_cast<int>(sizeof(Eigen::Index)) * 8 - 2);
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

VPolytope VPolytope::GetMinimalRepresentation(double tol) const {
  if (ambient_dimension() == 0) {
    return VPolytope();
  }
  if (vertices_.cols() <= 1) {
    // If there are zero columns, the VPolytope is empty. If there is one
    // column, the VPolytope is a point. Either way, the current representation
    // is already minimal.
    return *this;
  }
  if (ambient_dimension() == 1) {
    // If the ambient dimension is one, we can just take the lowest and highest
    // points.
    const double min = vertices_.row(0).minCoeff();
    const double max = vertices_.row(0).maxCoeff();
    return VPolytope(Eigen::Matrix<double, 1, 2>(min, max));
  }

  // Next, compute the affine hull to see if the VPolytope is not full
  // dimensional.
  const AffineSubspace affine_hull(*this, tol);
  if (affine_hull.AffineDimension() < affine_hull.ambient_dimension()) {
    // Project the points onto the local coordinate system of the affine hull,
    // compute the minimal representation there, and then lift back to the
    // ambient space.
    MatrixXd points_local = affine_hull.ToLocalCoordinates(vertices_);
    VPolytope vpoly_local(points_local);
    MatrixXd minimal_points_local =
        vpoly_local.GetMinimalRepresentation().vertices();
    return VPolytope(affine_hull.ToGlobalCoordinates(minimal_points_local));
  }

  orgQhull::Qhull qhull;
  qhull.runQhull("", vertices_.rows(), vertices_.cols(), vertices_.data(), "");
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull terminated with status {} and  message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }

  MatrixXd minimal_vertices(vertices_.rows(), qhull.vertexCount());
  size_t j = 0;
  for (const auto& qhull_vertex : qhull.vertexList()) {
    size_t i = 0;
    for (const auto& val : qhull_vertex.point()) {
      minimal_vertices(i, j) = val;
      ++i;
    }
    ++j;
  }

  // The qhull C++ interface iterates over the vertices in no specific order.
  // For the 2D case, reorder the vertices according to the counter-clockwise
  // convention.
  if (vertices_.rows() == 2) {
    minimal_vertices = OrderCounterClockwise(minimal_vertices);
  }

  return VPolytope(minimal_vertices);
}

void VPolytope::WriteObj(const std::filesystem::path& filename) const {
  DRAKE_THROW_UNLESS(ambient_dimension() == 3);

  const Vector3d center = vertices_.rowwise().mean();

  orgQhull::Qhull qhull;
  // http://www.qhull.org/html/qh-quick.htm#options
  // Pp avoids complaining about precision (it was used by trimesh).
  // Qt requests a triangulation.
  constexpr char qhull_options[] = "Pp Qt";
  qhull.runQhull("", vertices_.rows(), vertices_.cols(), vertices_.data(),
                 qhull_options);
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull terminated with status {} and message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }

  std::ofstream file;
  file.exceptions(~std::ofstream::goodbit);
  file.open(filename);
  std::vector<int> vertex_id_to_index(qhull.vertexCount() + 1);
  int index = 1;
  for (const auto& vertex : qhull.vertexList()) {
    fmt::print(file, "v {}\n", fmt::join(vertex.point(), " "));
    vertex_id_to_index.at(vertex.id()) = index++;
  }
  for (const auto& facet : qhull.facetList()) {
    DRAKE_DEMAND(facet.vertices().size() == 3);
    // Map the Qhull IDs into the obj file's "v" indices.
    const orgQhull::QhullVertex& v0 = facet.vertices()[0];
    const orgQhull::QhullVertex& v1 = facet.vertices()[1];
    const orgQhull::QhullVertex& v2 = facet.vertices()[2];
    std::array<int, 3> face_indices = {
        vertex_id_to_index.at(v0.id()),
        vertex_id_to_index.at(v1.id()),
        vertex_id_to_index.at(v2.id()),
    };
    // Adjust the normal to point away from the center.
    const Eigen::Map<Vector3d> a(v0.point().coordinates());
    const Eigen::Map<Vector3d> b(v1.point().coordinates());
    const Eigen::Map<Vector3d> c(v2.point().coordinates());
    const Vector3d normal = (b - a).cross(c - a);
    if (normal.dot(a - center) < 0) {
      std::swap(face_indices[0], face_indices[1]);
    }
    fmt::print(file, "f {}\n", fmt::join(face_indices, " "));
  }
  file.close();
}

Convex VPolytope::ToShapeConvex(const std::string& convex_label) const {
  DRAKE_THROW_UNLESS(ambient_dimension() == 3);
  return Convex(vertices_, convex_label);
}

std::unique_ptr<ConvexSet> VPolytope::DoClone() const {
  return std::make_unique<VPolytope>(*this);
}

bool VPolytope::DoIsEmpty() const {
  return vertices_.cols() == 0;
}

std::optional<bool> VPolytope::DoIsBoundedShortcut() const {
  return true;
}

std::optional<VectorXd> VPolytope::DoMaybeGetPoint() const {
  if (vertices_.cols() == 1) {
    return vertices_.col(0);
  }
  return std::nullopt;
}

std::optional<VectorXd> VPolytope::DoMaybeGetFeasiblePoint() const {
  if (IsEmpty()) {
    return std::nullopt;
  } else {
    return vertices_.col(0);
  }
}

bool VPolytope::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                             double tol) const {
  if (vertices_.cols() == 0) {
    return false;
  }

  // Attempt to "fail fast": Checks if a hyperplane through x, with a normal
  // vector colinear to (x - mean(vertices)), separates the point from the
  // VPolytope avoid solving the point containment LP. This is a heuristic,
  // sufficient condition which can falsify that the point is in the set which
  // works better as the point in question gets farther away.

  Eigen::VectorXd vertex_mean = vertices_.rowwise().mean();
  Eigen::VectorXd a = (x - vertex_mean).normalized();
  double b = a.dot(x);
  Eigen::VectorXd vals = a.transpose() * vertices_;
  vals = vals.array() - b;

  // Only allow early return if query point x is sufficiently far away from the
  // vertex mean.
  if ((vals.array() < -tol).all() && (x - vertex_mean).norm() > 1e-13) {
    return false;
  }

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

std::pair<VectorX<Variable>, std::vector<Binding<Constraint>>>
VPolytope::DoAddPointInSetConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x) const {
  std::vector<Binding<Constraint>> new_constraints;
  const int n = ambient_dimension();
  const int m = vertices_.cols();
  VectorXDecisionVariable alpha = prog->NewContinuousVariables(m, "a");
  // 0 ≤ αᵢ ≤ 1.  The one is redundant, but may be better than inf for some
  // solvers.
  new_constraints.push_back(prog->AddBoundingBoxConstraint(0, 1.0, alpha));
  // v α - x = 0.
  MatrixXd A(n, m + n);
  A.leftCols(m) = vertices_;
  A.rightCols(n) = -MatrixXd::Identity(n, n);
  new_constraints.push_back(
      prog->AddLinearEqualityConstraint(A, VectorXd::Zero(n), {alpha, x}));
  // ∑ αᵢ = 1.
  new_constraints.push_back(
      prog->AddLinearEqualityConstraint(RowVectorXd::Ones(m), 1.0, alpha));
  return {std::move(alpha), std::move(new_constraints)};
}

std::vector<Binding<Constraint>>
VPolytope::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
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

std::vector<Binding<Constraint>>
VPolytope::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  const int n = ambient_dimension();
  const int m = vertices_.cols();
  VectorXDecisionVariable alpha = prog->NewContinuousVariables(m, "a");
  // αᵢ ≥ 0.
  constraints.emplace_back(prog->AddBoundingBoxConstraint(
      0, std::numeric_limits<double>::infinity(), alpha));
  // v α = A * x + b.
  MatrixXd A_combination(n, m + x.size());
  A_combination.leftCols(m) = vertices_;
  A_combination.rightCols(x.size()) = -A;
  constraints.emplace_back(
      prog->AddLinearEqualityConstraint(A_combination, b, {alpha, x}));

  // ∑ αᵢ = c' * t + d.
  RowVectorXd a = RowVectorXd::Ones(m + t.size());
  a.tail(t.size()) = -c.transpose();
  constraints.emplace_back(prog->AddLinearEqualityConstraint(a, d, {alpha, t}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
VPolytope::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for VPolytope.  Implementing "
      "this will likely require additional support from the Convex shape "
      "class (to support in-memory mesh data, or file I/O).");
}

std::unique_ptr<ConvexSet> VPolytope::DoAffineHullShortcut(
    std::optional<double> tol) const {
  DRAKE_THROW_UNLESS(vertices_.size() > 0);
  MatrixXd centered_points =
      vertices_.rightCols(vertices_.cols() - 1).colwise() - vertices_.col(0);
#if EIGEN_VERSION_AT_LEAST(5, 0, 0)
  Eigen::JacobiSVD<MatrixXd, Eigen::DecompositionOptions::ComputeThinU> svd;
  svd.compute(centered_points);
#else
  Eigen::JacobiSVD<MatrixXd> svd;
  svd.compute(centered_points, Eigen::DecompositionOptions::ComputeThinU);
#endif
  if (tol) {
    svd.setThreshold(tol.value());
  }
  return std::make_unique<AffineSubspace>(svd.matrixU().leftCols(svd.rank()),
                                          vertices_.col(0));
}

double VPolytope::DoCalcVolume() const {
  orgQhull::Qhull qhull;
  try {
    qhull.runQhull("", ambient_dimension(), vertices_.cols(), vertices_.data(),
                   "");
  } catch (const orgQhull::QhullError& e) {
    if (e.errorCode() == qh_ERRsingular) {
      // The convex hull is singular. It has 0 volume.
      return 0;
    }
  }
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull terminated with status {} and  message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }
  return qhull.volume();
}

MatrixXd GetVertices(const Convex& convex) {
  return GetConvexHullVertices(convex.GetConvexHull());
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
