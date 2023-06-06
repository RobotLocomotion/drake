#include "drake/geometry/optimization/vpolytope.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>

#include <drake_vendor/libqhullcpp/Qhull.h>
#include <drake_vendor/libqhullcpp/QhullVertexSet.h>
#include <fmt/format.h>

#include "drake/common/is_approx_equal_abstol.h"
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

}  // namespace

VPolytope::VPolytope(const Eigen::Ref<const MatrixXd>& vertices)
    : ConvexSet(vertices.rows()), vertices_{vertices} {}

VPolytope::VPolytope(const QueryObject<double>& query_object,
                     GeometryId geometry_id,
                     std::optional<FrameId> reference_frame)
    : ConvexSet(3) {
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
    : ConvexSet(hpoly.ambient_dimension()) {
  DRAKE_THROW_UNLESS(hpoly.IsBounded());

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
    MatrixXd vertex_A(incident_hyperplanes.count(), hpoly.ambient_dimension());
    for (int jj = 0; jj < incident_hyperplanes.count(); jj++) {
      std::vector<double> hyperplane =
          incident_hyperplanes.at(jj).point().toStdVector();
      vertex_A.row(jj) = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(
          hyperplane.data(), hyperplane.size());
    }
    vertices_.col(ii) = vertex_A.partialPivLu().solve(
                            VectorXd::Ones(incident_hyperplanes.count())) +
                        eigen_center;
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

VPolytope VPolytope::GetMinimalRepresentation() const {
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

double VPolytope::CalcVolume() const {
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

std::unique_ptr<ConvexSet> VPolytope::DoClone() const {
  return std::make_unique<VPolytope>(*this);
}

bool VPolytope::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
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
  MatrixXd A(n, m + n);
  A.leftCols(m) = vertices_;
  A.rightCols(n) = -MatrixXd::Identity(n, n);
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

std::vector<solvers::Binding<solvers::Constraint>>
VPolytope::DoAddPointInNonnegativeScalingConstraints(
    solvers::MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A,
    const Eigen::Ref<const VectorXd>& b, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<solvers::Binding<solvers::Constraint>> constraints;
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

void VPolytope::ImplementGeometry(const Box& box, void* data) {
  const double x = box.width() / 2.0;
  const double y = box.depth() / 2.0;
  const double z = box.height() / 2.0;
  DRAKE_ASSERT(data != nullptr);
  Matrix3Xd* vertices = static_cast<Matrix3Xd*>(data);
  vertices->resize(3, 8);
  // clang-format off
  *vertices << -x,  x,  x, -x, -x,  x,  x, -x,
                y,  y, -y, -y, -y, -y,  y,  y,
               -z, -z, -z, -z,  z,  z,  z,  z;
  // clang-format on
}

void VPolytope::ImplementGeometry(const Convex& convex, void* data) {
  DRAKE_ASSERT(data != nullptr);
  Matrix3Xd* vertex_data = static_cast<Matrix3Xd*>(data);
  *vertex_data = GetVertices(convex);
}

MatrixXd GetVertices(const Convex& convex) {
  const auto [tinyobj_vertices, faces, num_faces] = internal::ReadObjFile(
      convex.filename(), convex.scale(), false /* triangulate */);
  unused(faces);
  unused(num_faces);
  orgQhull::Qhull qhull;
  const int dim = 3;
  std::vector<double> tinyobj_vertices_flat(tinyobj_vertices->size() * dim);
  for (int i = 0; i < ssize(*tinyobj_vertices); ++i) {
    for (int j = 0; j < dim; ++j) {
      tinyobj_vertices_flat[dim * i + j] = (*tinyobj_vertices)[i](j);
    }
  }
  qhull.runQhull("", dim, tinyobj_vertices->size(),
                 tinyobj_vertices_flat.data(), "");
  if (qhull.qhullStatus() != 0) {
    throw std::runtime_error(
        fmt::format("Qhull terminated with status {} and  message:\n{}",
                    qhull.qhullStatus(), qhull.qhullMessage()));
  }
  Matrix3Xd vertices(3, qhull.vertexCount());
  int vertex_count = 0;
  for (const auto& qhull_vertex : qhull.vertexList()) {
    vertices.col(vertex_count++) =
        Eigen::Map<Vector3d>(qhull_vertex.point().toStdVector().data());
  }
  return vertices;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
