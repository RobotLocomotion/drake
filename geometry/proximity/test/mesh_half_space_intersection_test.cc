#include "drake/geometry/proximity/mesh_half_space_intersection.h"

#include <functional>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/utilities.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransform;
using math::RigidTransformd;
using math::RotationMatrix;
using math::RotationMatrixd;
using std::make_unique;
using std::move;
using std::pair;
using std::unique_ptr;
using std::vector;

// Creates a TriangleSurfaceMesh of a box. The box is defined in frame B,
// centered on Bo with its dimensions aligned with Bx, By, and Bz. The resultant
// mesh has the vertices expressed in Frame F, courtesy of the transform `X_FB`.
TriangleSurfaceMesh<double> CreateBoxMesh(const RigidTransform<double>& X_FB) {
  // Set the box vertices according to the following diagram. The box is
  // centered at the origin of Frame B. We transform it to some query frame F,
  // making it less likely that all of the underlying functionality succeeds
  // because of some fortuitous initial values. The half space's boundary plane
  // is z = 0 in Frame B. Set the box vertices according to the following
  // diagram.
  //               v₁      +Z         v₃
  //               ●───────┆─────────●
  //              /│       ┆        /│
  //             / │       ┆     v₇/ │
  //         v₅ ●─────────────────●  │
  //            │  │       ┆      │  │
  //            │  │       o┄┄┄┄┄┄│┄┄┄┄┄ +Y
  //            │  │      /       │  │
  //            │  ●─────/────────│──● v₂
  //            │ /v₀   /         │ /
  //            │/     /          │/
  //         v₄ ●─────────────────● v₆
  //                 /
  //                +X

  std::vector<Vector3d> vertices_F{
      {X_FB * Vector3d(-1, -1, -1), X_FB * Vector3d(-1, -1, 1),
       X_FB * Vector3d(-1, 1, -1), X_FB * Vector3d(-1, 1, 1),
       X_FB * Vector3d(1, -1, -1), X_FB * Vector3d(1, -1, 1),
       X_FB * Vector3d(1, 1, -1), X_FB * Vector3d(1, 1, 1)}};

  // Set the twelve box faces using a counter-clockwise winding.
  std::vector<SurfaceTriangle> faces;
  faces.emplace_back(4, 6, 7);  // +X face
  faces.emplace_back(7, 5, 4);  // +X face
  faces.emplace_back(1, 3, 2);  // -X face
  faces.emplace_back(2, 0, 1);  // -X face
  faces.emplace_back(2, 3, 7);  // +Y face
  faces.emplace_back(7, 6, 2);  // +Y face
  faces.emplace_back(4, 5, 1);  // -Y face
  faces.emplace_back(1, 0, 4);  // -Y face
  faces.emplace_back(7, 3, 1);  // +Z face
  faces.emplace_back(1, 5, 7);  // +Z face
  faces.emplace_back(4, 0, 2);  // -Z face
  faces.emplace_back(2, 6, 4);  // -Z face

  // Construct the mesh.
  return {move(faces), move(vertices_F)};
}

// Given a triangle mesh expressed in frame F, creates a "contact surface mesh"
// expressed in frame W. If the mesh representation is polygon, the triangles
// are simply transformed into 3-sided polygons. If the mesh representation is
// triangle, each triangle is tessellated around its centroid.
template <typename MeshType>
MeshType RexpressAsContactMesh(
    const TriangleSurfaceMesh<double>& mesh_F,
    const RigidTransform<typename MeshType::ScalarType>& X_WF) {
  using T = typename MeshType::ScalarType;
  vector<Vector3<T>> vertices_W;
  vertices_W.reserve(mesh_F.num_vertices());
  for (const auto& p_MV : mesh_F.vertices()) {
    vertices_W.emplace_back(X_WF * p_MV.cast<T>());
  }
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    vector<SurfaceTriangle> triangles;
    triangles.reserve(mesh_F.num_triangles() * 3);
    vector<int> polygon(8);
    for (int t_index = 0; t_index < mesh_F.num_triangles(); ++t_index) {
      const auto& tri = mesh_F.element(t_index);
      polygon.clear();
      polygon.push_back(tri.vertex(0));
      polygon.push_back(tri.vertex(1));
      polygon.push_back(tri.vertex(2));
      const Vector3<T>& nhat_W =
          X_WF.rotation() * mesh_F.face_normal(t_index).cast<T>();
      AddPolygonToTriangleMeshData(polygon, nhat_W, &triangles, &vertices_W);
    }
    return {move(triangles), move(vertices_W)};
  } else {
    vector<int> face_data;
    face_data.reserve(mesh_F.num_triangles() * 4);
    for (const auto& tri : mesh_F.triangles()) {
      face_data.push_back(3);
      for (int i = 0; i < 3; ++i) {
        face_data.push_back(tri.vertex(i));
      }
    }
    return {move(face_data), move(vertices_W)};
  }
}


// TODO(SeanCurtis-TRI): All of the tests where we actually examine the mesh
//  use a mesh with a _single_ triangle. In this case, the face-local index
//  values are perfectly aligned with the mesh-local index values. This hides
//  potential errors where the two sets of indices do _not_ align so nicely.
//  One potential solution is to simply insert an additional vertex in the mesh
//  that isn't referenced so the face isn't built on (0, 1, 2), but (1, 2, 3).
//
// Creates a TriangleSurfaceMesh with a single triangle using the given
// vertices.
TriangleSurfaceMesh<double> CreateOneTriangleMesh(const Vector3d& v0,
                                                  const Vector3d& v1,
                                                  const Vector3d& v2) {
  std::vector<Vector3d> vertices{{v0, v1, v2}};
  std::vector<SurfaceTriangle> faces{{0, 1, 2}};
  return TriangleSurfaceMesh<double>(move(faces), move(vertices));
}

// Helper for getting the appropriate mesh builder from an expected mesh type.
template <typename MeshType>
struct BuilderTypeFromMesh {};

template <typename T>
struct BuilderTypeFromMesh<TriangleSurfaceMesh<T>> {
 public:
  using Type = TriMeshBuilder<T>;
};

template <typename T>
struct BuilderTypeFromMesh<PolygonSurfaceMesh<T>> {
 public:
  using Type = PolyMeshBuilder<T>;
};

/* This test evaluates the *values* of the mesh-half space intersection
 algorithm for all supported scalars (double and AutoDiffXd). The derivatives
 for AutoDiffXd are tested below. */
template <typename MeshType>
class MeshHalfSpaceValueTest : public ::testing::Test {
 public:
  // Using the symbol T here wreaks havoc with trying to use that same symbol
  // in the classes in the derived types: shadow warnings. So, we'll simply
  // distinguish names.
  using Scalar = typename MeshType::ScalarType;

  // Accessors for data structures used repeatedly.
  std::unordered_map<int, int>& vertices_to_newly_created_vertices() {
    return vertices_to_newly_created_vertices_;
  }
  std::unordered_map<SortedPair<int>, int>& edges_to_newly_created_vertices() {
    return edges_to_newly_created_vertices_;
  }
  typename BuilderTypeFromMesh<MeshType>::Type& builder_W() {
    return builder_W_;
  }

  // Checks whether two faces (identified by their indices) from two meshes are
  // equivalent, which we define to mean as some permutation of acceptable
  // windings for the vertices of `fb` yields vertices coincident with those of
  // `fa`.
  bool AreFacesEquivalent(int a,
                          const MeshType& mesh_a,
                          int b,
                          const MeshType& mesh_b) {
    const auto& fa = mesh_a.element(a);
    const auto& fb = mesh_b.element(b);
    if (fa.num_vertices() != fb.num_vertices()) return false;

    // Verify that there exists a valid rotation that aligns the vertices in fa
    // and fb. A "rotation" between two ordered lists is a permutation in which
    // the position of every element in one list is a constant, fixed offset
    // from its position in the other (with periodic boundary conditions). For
    // a given rotation, the ith vertex in each face "align" if the vertices
    // are coincident (to within epsilon units of distance).
    const int num_vertices = fa.num_vertices();
    constexpr double kEps = 32 * std::numeric_limits<double>::epsilon();
    constexpr double kEps2 = kEps * kEps;
    // For N vertices, there are N valid rotations.
    for (int rotation = 0; rotation < num_vertices; ++rotation) {
      bool match = true;
      for (int a_i = 0; a_i < num_vertices; ++a_i) {
        const int b_i = (a_i + rotation) % num_vertices;
        if ((mesh_a.vertex(fa.vertex(a_i)) - mesh_b.vertex(fb.vertex(b_i)))
                .squaredNorm() > kEps2) {
          match = false;
          break;
        }
      }
      if (match) return true;
    }

    return false;
  }

  // Calling CallConstructTriangleHalfspaceIntersectionPolygon() produces a mesh
  // and field. This confirms that the resulting test mesh is equivalent to an
  // expected mesh. Specifically:
  //
  //   - The meshes must:
  //     - have the same number of vertices (permuted order is allowed).
  //     - have the same number of faces (again, permuted order is allowed)
  //       - There must be a 1-to-1 correspondence between mesh faces in that
  //         they span the same surface (but possibly with "rotated" vertex
  //         ordering).
  ::testing::AssertionResult MeshesEquivalent(const MeshType& test_mesh_W,
                                              const MeshType& expected_mesh_W) {
    // Simple checks first.
    if (test_mesh_W.num_elements() != expected_mesh_W.num_elements()) {
      return ::testing::AssertionFailure()
             << "Mismatch in face count. Test has "
             << test_mesh_W.num_elements() << " faces; expected "
             << expected_mesh_W.num_elements();
    }
    if (test_mesh_W.num_vertices() != expected_mesh_W.num_vertices()) {
      return ::testing::AssertionFailure()
             << "Mismatch in vertex count. Test has "
             << test_mesh_W.num_vertices() << " vertices; expected "
             << expected_mesh_W.num_vertices();
    }

    // Iterate through each face of the first mesh, looking for a face from the
    // second mesh that is within the given tolerance. This is a quadratic
    // time algorithm (in the number of faces of the meshes) but we expect the
    // number of faces that this algorithm is run on to be small.
    const int face_count = test_mesh_W.num_elements();
    vector<int> unmatched_from_expected(face_count);
    std::iota(unmatched_from_expected.begin(), unmatched_from_expected.end(),
              0);
    for (int i = 0; i < face_count; ++i) {
      bool found_match = false;
      for (int j : unmatched_from_expected) {
        if (AreFacesEquivalent(i, test_mesh_W, j, expected_mesh_W)) {
          found_match = true;
          // Modifying the vector during range iteration is ok, because this
          // modifications triggers dropping out of the iteration.
          unmatched_from_expected[j] = unmatched_from_expected.back();
          unmatched_from_expected.pop_back();
          break;
        }
      }
      // One unmatched element is sufficient to stop the test with failure.
      if (!found_match) {
        return ::testing::AssertionFailure()
               << "At least one face in the test mesh failed to match to a "
                  "face in the expected mesh; there may be more.";
      }
    }

    return ::testing::AssertionSuccess();
  }

  // Calling CallConstructTriangleHalfspaceIntersectionPolygon() produces a mesh
  // and field. This tests the resulting mesh field against this test fixture's
  // pressure field. Specifically:
  //
  //   - The field must:
  //     - report the same per-vertex pressure as when evaluating this fixture's
  //       pressure_in_F_.
  //     - The gradient on each element is equal to this fixture's grad_p_F_.
  //     - For each face, given two vertices on that face, the following must be
  //       true:
  //           (p(V0) + p(V1)) / 2 = p((V0 + V1) / 2)
  //       In other words, we're making sure the field value on the interior is
  //       a linear function consistent with the per-vertex values.
  ::testing::AssertionResult FieldConsistent(const MeshType& test_mesh_W,
                    const MeshFieldLinear<Scalar, MeshType>& test_field_W,
                    const RigidTransform<Scalar>& X_WF) {
    using std::abs;
    const RigidTransform<Scalar> X_FW = X_WF.inverse();

    // This pressure test seems to need a bit more error tolerance. While it's
    // still an acceptable order of magnitude, some investigation as to why
    // this is would be beneficial.
    constexpr double kPressureEps = 3e-14;
    for (int v = 0; v < test_mesh_W.num_vertices(); ++v) {
      const Scalar test_pressure = test_field_W.EvaluateAtVertex(v);
      const Vector3<Scalar> p_FV = X_FW * test_mesh_W.vertex(v);
      const Scalar expected_pressure = pressure_in_F_(p_FV);
      const Scalar error = abs(test_pressure - expected_pressure);
      if (error > kPressureEps) {
        return ::testing::AssertionFailure()
               << "\nBad pressure field value at vertex " << v << "\n"
               << "  Expected: " << expected_pressure << "\n"
               << "  Found: " << test_pressure << "\n"
               << "  tolerance: " << kPressureEps << "\n"
               << "  error: " << error;
      }
    }

    // We don't store/compute the gradients of the field for
    // TriangleSurfaceMesh. Given the algorithm would produce them
    // automatically, if we choose to do so for the triangle mesh, we could
    // remove the compilation guard here.
    if (std::is_same_v<MeshType, PolygonSurfaceMesh<Scalar>>) {
      const Vector3<Scalar> grad_p_W = X_WF.rotation() * grad_p_F_;
      for (int f = 0; f < test_mesh_W.num_elements(); ++f) {
        auto result =
            CompareMatrices(test_field_W.EvaluateGradient(f), grad_p_W);
        if (!result) return result << "\nOn face " << f;
      }
    }

    for (int f = 0; f < test_mesh_W.num_elements(); ++f) {
      const auto& face = test_mesh_W.element(f);
      const Vector3<Scalar>& p_WV0 = test_mesh_W.vertex(face.vertex(0));
      const Vector3<Scalar>& p_WV1 = test_mesh_W.vertex(face.vertex(1));
      const Vector3<Scalar> p_WX = (p_WV0 + p_WV1) / 2;
      const Scalar p_0 = test_field_W.EvaluateAtVertex(face.vertex(0));
      const Scalar p_1 = test_field_W.EvaluateAtVertex(face.vertex(1));
      const Scalar p_X_expected = (p_0 + p_1) / 2;
      const Scalar p_X_test = test_field_W.EvaluateCartesian(f, p_WX);
      const Scalar error = (p_X_test - p_X_expected);
      if (error > kPressureEps) {
        return ::testing::AssertionFailure()
               << "\nMesh face " << f
               << " failed to provide the correct pressure for a point on the "
                  "inside of the face: " << p_WX.transpose() << ".\n"
               << "  Expected: " << p_X_expected << "\n"
               << "  Found: " << p_X_test << "\n"
               << "  tolerance: " << kPressureEps << "\n"
               << "  error: " << error;
      }
    }

    return ::testing::AssertionSuccess();
  }

  // Clears all data structures used for constructing the intersection.
  void ClearConstructionDataStructures() {
    builder_W_ = typename BuilderTypeFromMesh<MeshType>::Type();
    vertices_to_newly_created_vertices_.clear();
    edges_to_newly_created_vertices_.clear();
  }

  // Calls the triangle-half space intersection routine between the test
  // fixture's half space and the provided mesh (measured and expressed in frame
  // F). The resulting builder data are measured and expressed in the world
  // frame (courtesy of the X_WF value).
  //
  // The contact mesh and field are built by iteratively invoking
  // ConstructTriangleHalfSpaceIntersectionPolygon() on *every* triangle of the
  // mesh. It uses this test fixture's caching data structures (which can be
  // inspected after the call).
  //
  // Each call starts with an empty mesh builder and empty cache data
  // structures.
  void CallConstructTriangleHalfspaceIntersectionPolygon(
      const TriangleSurfaceMesh<double>& mesh_F,
      const RigidTransform<Scalar> X_WF) {
    ClearConstructionDataStructures();
    const Vector3<Scalar> grad_p_W = X_WF.rotation() * this->grad_p_F_;
    for (int f_index = 0; f_index < mesh_F.num_elements(); ++f_index) {
      ConstructTriangleHalfspaceIntersectionPolygon(
          mesh_F, f_index, *this->half_space_F_, this->pressure_in_F_, grad_p_W,
          X_WF, &this->builder_W_, &this->vertices_to_newly_created_vertices_,
          &this->edges_to_newly_created_vertices_);
    }
  }

 private:
  void SetUp() {
    // The tests all use half space normal [0 0 1], with point [0 0 2]
    // lying on the half space.
    Vector3<Scalar> normal_F(0, 0, 1);
    const Vector3<Scalar> point_F(0, 0, 2);
    half_space_F_ = std::make_unique<PosedHalfSpace<Scalar>>(normal_F, point_F);
    // Define the half space's pressure field with arbitrary pressure "scale",
    // but make sure it is zero on the surface.
    pressure_scale_ = 17;
    grad_p_F_ = -pressure_scale_ * normal_F;
    pressure_in_F_ = [grad_p_F = grad_p_F_,
                      point_F](const Vector3<Scalar>& p_VF) {
      return grad_p_F.dot(p_VF - point_F);
    };
  }

  typename BuilderTypeFromMesh<MeshType>::Type builder_W_;
  std::unordered_map<int, int> vertices_to_newly_created_vertices_;
  std::unordered_map<SortedPair<int>, int> edges_to_newly_created_vertices_;
  // In this test harness, the half space is always simply defined in the
  // mesh's frame F.
  std::unique_ptr<PosedHalfSpace<Scalar>> half_space_F_;
  double pressure_scale_;
  Vector3<Scalar> grad_p_F_;
  std::function<Scalar(const Vector3<Scalar>&)> pressure_in_F_;
};  // namespace

TYPED_TEST_SUITE_P(MeshHalfSpaceValueTest);

// Verifies that a triangle that lies fully outside of the half space yields an
// empty intersection. This covers Case 4 in the code. This code is agnostic
// of the MeshBuilder type, so we simply execute it with a PolyMeshBuilder.
TYPED_TEST_P(MeshHalfSpaceValueTest, NoIntersection) {
  // Create the mesh, constructing the vertices of the triangle to lie outside
  // the half space.
  const TriangleSurfaceMesh<double> mesh_F = CreateOneTriangleMesh(
      Vector3d(0, 0, 3), Vector3d(1, 0, 3), Vector3d(0, 1, 3));

  // X_WF = I -- because there is no intersection, the frame of the
  // non-existent intersection mesh is irrelevant.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(mesh_F, {});
  EXPECT_EQ(this->builder_W().num_faces(), 0);
  EXPECT_TRUE(this->vertices_to_newly_created_vertices().empty());
  EXPECT_TRUE(this->edges_to_newly_created_vertices().empty());
}

// Verifies that a triangle that lies inside or on the half space yields
// that same triangle. This covers Case 1 in the code.
TYPED_TEST_P(MeshHalfSpaceValueTest, InsideOrOnIntersection) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Case: The triangular mesh lies *completely* inside the half space.
  {
    // Because the mesh is wholly contained within the half space, the contact
    // mesh is the same mesh, just re-expressed in the world frame. Remember,
    // the half space is Fz <= 2. So, putting these triangles at Fz = 1,
    // they'll lie inside.
    const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 1), Vector3d(4, 5, 1), Vector3d(3, 6, 1));

    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    auto expected_mesh_W = RexpressAsContactMesh<MeshType>(tri_mesh_F, X_WF);
    EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, expected_mesh_W));
    EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
  }

  // Case: triangle lies on the boundary plane.
  {
    // This triangle is located at Fz = 2, so it lies exactly on the boundary.
    const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 2), Vector3d(3, 6, 2));

    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    auto expected_mesh_W = RexpressAsContactMesh<MeshType>(tri_mesh_F, X_WF);
    EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, expected_mesh_W));
    EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
  }

  // Case: two triangles with a shared edge, completely enclosed in the half
  // space.
  {
    std::vector<Vector3d> vertices = {Vector3d(4, 5, 2), Vector3d(3, 5, 2),
                                      Vector3d(3, 5, 1), Vector3d(2, 5, 2)};
    std::vector<SurfaceTriangle> faces{{0, 1, 2}, {2, 1, 3}};
    const TriangleSurfaceMesh<double> tri_mesh_F(move(faces), move(vertices));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    auto expected_mesh_W = RexpressAsContactMesh<MeshType>(tri_mesh_F, X_WF);
    EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, expected_mesh_W));
    EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
  }
}

// Verifies that a triangle that has exactly one vertex lying on the boundary
// of the half space produces either (a) a degenerate intersection (if the
// other two vertices lie outside the half space) or (b) the original
// triangle (if the other two vertices lie within the half space). This
// covers Case 3 for (a) and Case 1 for (b).
TYPED_TEST_P(MeshHalfSpaceValueTest, VertexOnHalfspaceIntersection) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Case: (a) one vertex on the boundary, two vertices outside.
  {
    const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 3), Vector3d(3, 6, 3));

    // Verify the degenerate intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
        EXPECT_EQ(mesh_W->num_vertices(), 4);
        EXPECT_EQ(mesh_W->num_elements(), 3);
    } else {
        EXPECT_EQ(mesh_W->num_vertices(), 3);
        EXPECT_EQ(mesh_W->num_elements(), 1);
    }
    // Evidence that when computing the intersecting _polygon_, we copied one
    // vertex and split two edges.
    EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 1);
    EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

    // Confirm degeneracy: areas are close to zero, and vertices are close to
    // vertex 0 in tri_mesh_F.
    for (int f = 0; f < mesh_W->num_elements(); ++f) {
      EXPECT_NEAR(ExtractDoubleOrThrow(mesh_W->area(f)), 0, 1e-15);
    }
    // The reference vertex.
    const Vector3<T> p_WR = X_WF * tri_mesh_F.vertex(0).cast<T>();
    for (int v = 0; v < mesh_W->num_vertices(); ++v) {
      EXPECT_NEAR(ExtractDoubleOrThrow((mesh_W->vertex(v) - p_WR).norm()), 0,
                  1e-15);
      EXPECT_NEAR(ExtractDoubleOrThrow(field_W->EvaluateAtVertex(v)), 0, 1e-15);
    }
  }

  // Case: (b) one vertex on the boundary, two vertices inside.
  {
    const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 2), Vector3d(4, 5, 1), Vector3d(3, 6, 1));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    auto expected_mesh_W = RexpressAsContactMesh<MeshType>(tri_mesh_F, X_WF);
    EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, expected_mesh_W));
    EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
  }
}

// Verifies that a triangle that has exactly two vertices lying on the
// boundary of the half space produces either (a) a degenerate intersection
// (if the remaining vertex lies outside the half space) or (b) the original
// triangle (if the other vertex lies within the half space). This covers
// Case 2 for (a) and Case 1 for (b).
TYPED_TEST_P(MeshHalfSpaceValueTest, EdgeOnHalfspaceIntersection) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Case: (a) two vertices on the boundary, one outside.
  {
    const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 3), Vector3d(2, 5, 2), Vector3d(4, 5, 2));

    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
        EXPECT_EQ(mesh_W->num_vertices(), 5);
        EXPECT_EQ(mesh_W->num_elements(), 4);
    } else {
        EXPECT_EQ(mesh_W->num_vertices(), 4);
        EXPECT_EQ(mesh_W->num_elements(), 1);
    }
    // Evidence that when computing the intersecting _polygon_, we copied two
    // vertices and split two edges.
    EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 2);
    EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

    //                    a
    //                   ╱ ╲
    // ^ z              ╱   ╲
    // |         ______╱__.__╲_____
    // |         ▒▒▒b/e▒▒▒f▒▒▒c/d▒▒
    // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
    // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
    // --------> x
    //
    // The intersecting triangle would consist of vertices a, b, & c. The
    // intersection will end up duplicating vertices b and c into vertices e
    // and d, respectively. The resulting contact polygon is a zero-area quad
    // over vertices b, e, c, & d. The final triangle fan is formed around the
    // centroid f.
    //
    // Confirm degeneracy: areas are close to zero, and vertices are close to
    // vertices 1 and 2 in tri_mesh_F (as documented above).
    for (int f = 0; f < mesh_W->num_elements(); ++f) {
      EXPECT_NEAR(ExtractDoubleOrThrow(mesh_W->area(f)), 0, 1e-15);
    }
    // The reference vertices. For simplicity, we'll only consider the
    // _minimum_ distance each vertex is to one of these two references.
    const Vector3<T> p_WV1 = X_WF * tri_mesh_F.vertex(1).cast<T>();
    const Vector3<T> p_WV2 = X_WF * tri_mesh_F.vertex(2).cast<T>();
    using std::min;
    // The last vertex is the centroid we're ignoring for now.
    for (int v = 0; v < mesh_W->num_vertices() - 1; ++v) {
      const double min_dist =
          ExtractDoubleOrThrow(min((mesh_W->vertex(v) - p_WV1).norm(),
                                   (mesh_W->vertex(v) - p_WV2).norm()));
      EXPECT_NEAR(min_dist, 0, 1e-15);
      EXPECT_NEAR(ExtractDoubleOrThrow(field_W->EvaluateAtVertex(v)), 0, 1e-15);
    }
  }

  // Case: (b) two vertices on the boundary, one vertex inside.
  {
    const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
        Vector3d(3, 5, 1), Vector3d(4, 5, 2), Vector3d(3, 6, 2));

    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    auto expected_mesh_W = RexpressAsContactMesh<MeshType>(tri_mesh_F, X_WF);
    EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, expected_mesh_W));
    EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
  }
}

// Verifies that a triangle that has two vertices within the half space and
// another outside the half space produces a quadrilateral. This covers Case 2.
TYPED_TEST_P(MeshHalfSpaceValueTest, QuadrilateralResults) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Construct one vertex of the triangle to lie outside the half space and the
  // other two to lie inside the half space.
  const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
      Vector3d(3, 5, 3), Vector3d(2, 5, 1), Vector3d(4, 5, 1));

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲d____
  // |         ▒▒▒▒▒╱▒▒▒▒▒▒╲▒▒▒▒
  // |         ▒▒▒▒╱________╲▒▒▒
  // |         ▒▒▒b▒▒▒▒▒▒▒▒▒▒c▒▒
  // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  // --------> x

  {
    // Verify the intersection.
    this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
    auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
      EXPECT_EQ(mesh_W->num_vertices(), 5);
      EXPECT_EQ(mesh_W->num_elements(), 4);
    } else {
      EXPECT_EQ(mesh_W->num_vertices(), 4);
      EXPECT_EQ(mesh_W->num_elements(), 1);
    }
    // Evidence that when computing the intersecting _polygon_, we copied two
    // vertices and split two edges.
    EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 2);
    EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

    // Check that the mesh that results is equivalent to the expected mesh.
    std::vector<Vector3<T>> expected_vertices_W = {
        X_WF * Vector3<T>(2, 5, 1),     // b
        X_WF * Vector3<T>(4, 5, 1),     // c
        X_WF * Vector3<T>(3.5, 5, 2),   // d
        X_WF * Vector3<T>(2.5, 5, 2)};  // e
    const int b{0}, c{1}, d{2}, e{3};
    const std::vector<int> polygon{b, c, d, e};
    // We have to use a unique_ptr because TriangleSurfaceMesh doesn't have a
    // default constructor.
    unique_ptr<MeshType> expected_mesh_W;
    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    // TODO(SeanCurtis-TRI): When we use the known polygon normal in the
    // AddPolygonToPolygonMeshData, move this normal definition out.
    const Vector3<T> nhat_W =
        X_WF.rotation() * tri_mesh_F.face_normal(0).cast<T>();
      std::vector<SurfaceTriangle> expected_faces;
      AddPolygonToTriangleMeshData(polygon, nhat_W, &expected_faces,
                                   &expected_vertices_W);
      expected_mesh_W = make_unique<MeshType>(move(expected_faces),
                                              move(expected_vertices_W));
    } else {
      std::vector<int> face_data;
      AddPolygonToPolygonMeshData(polygon, &face_data);
      expected_mesh_W =
          make_unique<MeshType>(move(face_data), move(expected_vertices_W));
    }

    EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, *expected_mesh_W));
    EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
  }
}

// Verifies that a triangle that has one vertex outside the half space, one
// vertex inside the half space, and one vertex on the boundary of the half
// space produces the intersected polygon that is a degenerated quadrilateral
// with a zero-length edge. This test covers Case 2.
TYPED_TEST_P(MeshHalfSpaceValueTest, OutsideInsideOn) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(RotationMatrix<T>(AngleAxis<T>{
                                   M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
                               Vector3<T>{-0.25, 0.5, 0.75});

  const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
      Vector3d(3, 5, 3), Vector3d(2, 5, 1), Vector3d(3.5, 5, 2));

  // Verify the intersection.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
  auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    EXPECT_EQ(mesh_W->num_vertices(), 5);
    EXPECT_EQ(mesh_W->num_elements(), 4);
  } else {
    EXPECT_EQ(mesh_W->num_vertices(), 4);
    EXPECT_EQ(mesh_W->num_elements(), 1);
  }
  // Evidence that when computing the intersecting _polygon_, we copied two
  // vertices and split two edges.
  EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 2);
  EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  //                   a
  //                   ╱╲
  // ^ z              ╱  ╲
  // |         _____e╱____╲c/d___
  // |         ▒▒▒▒▒╱▒▒▒▒▒▒▒▒▒▒▒▒
  // |         ▒▒▒▒╱▒▒▒▒▒▒▒▒▒▒▒▒▒
  // |         ▒▒▒b▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  // |         ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  // --------> x
  //
  // Note: there is an unvisualized edge between vertices b and c -- ascii art
  // can't draw a line at that angle.

  // Check that the mesh that results is equivalent to the expected mesh.

  // The intersected polygon is a degenerated quadrilateral with the
  // zero-length edge cd.
  std::vector<Vector3<T>> expected_vertices_W = {
      X_WF * Vector3<T>(2, 5, 1),     // b
      X_WF * Vector3<T>(3.5, 5, 2),   // c
      X_WF * Vector3<T>(3.5, 5, 2),   // d
      X_WF * Vector3<T>(2.5, 5, 2)};  // e
  const int b{0}, c{1}, d{2}, e{3};
  const std::vector<int> polygon{b, c, d, e};
  // We have to use a unique_ptr because TriangleSurfaceMesh doesn't have a
  // default constructor.
  unique_ptr<MeshType> expected_mesh_W;
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    // TODO(SeanCurtis-TRI): When we use the known polygon normal in the
    // AddPolygonToPolygonMeshData, move this normal definition out.
    const Vector3<T> nhat_W =
        X_WF.rotation() * tri_mesh_F.face_normal(0).cast<T>();
    std::vector<SurfaceTriangle> expected_faces;
    AddPolygonToTriangleMeshData(polygon, nhat_W, &expected_faces,
                                 &expected_vertices_W);
    expected_mesh_W =
        make_unique<MeshType>(move(expected_faces), move(expected_vertices_W));
  } else {
    std::vector<int> face_data;
    AddPolygonToPolygonMeshData(polygon, &face_data);
    expected_mesh_W =
        make_unique<MeshType>(move(face_data), move(expected_vertices_W));
  }

  EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, *expected_mesh_W));
  EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
}

// Verifies that a triangle with one vertex inside the half space and two
// vertices outside of the half space produces an interesected polygon that
// is a triangle. This test covers Case 3.
TYPED_TEST_P(MeshHalfSpaceValueTest, OneInsideTwoOutside) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(RotationMatrix<T>(AngleAxis<T>{
                                   M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
                               Vector3<T>{-0.25, 0.5, 0.75});

  const TriangleSurfaceMesh<double> tri_mesh_F = CreateOneTriangleMesh(
      Vector3d(3, 5, 3), Vector3d(4, 5, 3), Vector3d(3, 6, 1));

  // ^ z
  // |            b ________ c
  // |              ╲      ╱
  // |         _____e╲____╱d____
  // |         ▒▒▒▒▒▒▒╲▒▒╱▒▒▒▒▒▒
  // |         ▒▒▒▒▒▒▒▒╲╱▒▒▒▒▒▒▒
  // |         ▒▒▒▒▒▒▒▒a▒▒▒▒▒▒▒▒
  // --------> x

  // Verify the intersection.
  this->CallConstructTriangleHalfspaceIntersectionPolygon(tri_mesh_F, X_WF);
  auto [mesh_W, field_W] = this->builder_W().MakeMeshAndField();

  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    EXPECT_EQ(mesh_W->num_vertices(), 4);
    EXPECT_EQ(mesh_W->num_elements(), 3);
  } else {
    EXPECT_EQ(mesh_W->num_vertices(), 3);
    EXPECT_EQ(mesh_W->num_elements(), 1);
  }
  // Evidence that when computing the intersecting _polygon_, we copied one
  // vertex and split two edges.
  EXPECT_EQ(this->vertices_to_newly_created_vertices().size(), 1);
  EXPECT_EQ(this->edges_to_newly_created_vertices().size(), 2);

  std::vector<Vector3<T>> expected_vertices_W = {
      X_WF * Vector3<T>(3, 6, 1), X_WF * Vector3<T>(3, 5.5, 2),
      X_WF * Vector3<T>(3.5, 5.5, 2)};
  const std::vector<int> polygon{0, 1, 2};
  // We have to use a unique_ptr because TriangleSurfaceMesh doesn't have a
  // default constructor.
  unique_ptr<MeshType> expected_mesh_W;
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    // TODO(SeanCurtis-TRI): When we use the known polygon normal in the
    // AddPolygonToPolygonMeshData, move this normal definition out.
    const Vector3<T> nhat_W =
        X_WF.rotation() * tri_mesh_F.face_normal(0).cast<T>();
    std::vector<SurfaceTriangle> expected_faces;
    AddPolygonToTriangleMeshData(polygon, nhat_W, &expected_faces,
                                 &expected_vertices_W);
    expected_mesh_W =
        make_unique<MeshType>(move(expected_faces), move(expected_vertices_W));
  } else {
    std::vector<int> face_data;
    AddPolygonToPolygonMeshData(polygon, &face_data);
    expected_mesh_W =
        make_unique<MeshType>(move(face_data), move(expected_vertices_W));
  }

  EXPECT_TRUE(this->MeshesEquivalent(*mesh_W, *expected_mesh_W));
  EXPECT_TRUE(this->FieldConsistent(*mesh_W, *field_W, X_WF));
}

// Tests ComputeContactSurface() by intersecting a box with a half space. We
// pretend that BVH culling produced a vector consisting of *all* box triangles.
// In this case, we need to confirm the following:
//   - nullptr is returned for empty index list or no *actual* intersections
//     among the triangle indices provided.
//   - Otherwise a valid contact surface is returned:
//     - the mesh is as expected (coarsely), we rely on the unit tests of the
//       underlying infrastructure to get the details correct.
//     - the field is as expected (again, coarsely).
//     - We have gradient samples on geometry M.
TYPED_TEST_P(MeshHalfSpaceValueTest, ComputeContactSurfaceInvocation) {
  using MeshType = TypeParam;
  using T = typename MeshType::ScalarType;

  static_assert(std::is_same_v<MeshType, TriangleSurfaceMesh<T>> ||
                    std::is_same_v<MeshType, PolygonSurfaceMesh<T>>,
                "Mesh type isn't a mesh");

  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<T> X_WF(
      RotationMatrix<T>(
          AngleAxis<T>{M_PI / 7, Vector3<T>{1, 2, 3}.normalized()}),
      Vector3<T>{-0.25, 0.5, 0.75});

  // Set B to an arbitrarily chosen pose relative to F.
  RigidTransform<T> X_FB(math::RollPitchYaw<T>(M_PI_4, M_PI_4, M_PI_4),
                         Vector3<T>(1.0, 2.0, 3.0));
  RigidTransform<double> X_FB_d = convert_to_double(X_FB);


  const TriangleSurfaceMesh<double> tri_mesh_F = CreateBoxMesh(X_FB_d);
  const GeometryId mesh_id = GeometryId::get_new_id();

  // Construct the half-space.
  const Vector3<T> Bz_F = X_FB.rotation().col(2);
  const PosedHalfSpace<T> half_space_F(Bz_F, X_FB.translation());
  const GeometryId half_space_id = GeometryId::get_new_id();
  const Vector3<T> grad_p_F = -17 * Bz_F;
  const Vector3<T> grad_p_W = X_WF.rotation() * grad_p_F;
  auto pressure_in_F = [grad_p_F,
                        p_F0 = X_FB.translation()](const Vector3<T>& p_FV) {
    return grad_p_F.dot(p_FV - p_F0);
  };

  using MeshBuilder = typename BuilderTypeFromMesh<MeshType>::Type;

  // Case no intersections returns nullptr.
  {
    // The triangles indicated don't actually intersect the half space.
    EXPECT_EQ(ComputeContactSurface<MeshBuilder>(
                  mesh_id, tri_mesh_F, half_space_id, half_space_F,
                  pressure_in_F, grad_p_W, {8, 9}, X_WF),
              nullptr);
    // There are no indices provided.
    EXPECT_EQ(ComputeContactSurface<MeshBuilder>(
                  mesh_id, tri_mesh_F, half_space_id, half_space_F,
                  pressure_in_F, grad_p_W, {}, X_WF),
              nullptr);
  }

  // Case: Plane intersects the mesh.
  {
    // We pass in indices of *all* the triangles.
    std::vector<int> tri_indices(tri_mesh_F.num_elements());
    std::iota(tri_indices.begin(), tri_indices.end(), 0);

    const auto& surface = ComputeContactSurface<MeshBuilder>(
        mesh_id, tri_mesh_F, half_space_id, half_space_F, pressure_in_F,
        grad_p_W, tri_indices, X_WF);

    // The half space intersects the mesh with four vertices inside and four
    // outside. Therefore, we expect intersection to produce the following
    // with respect to the intersecting *polygons*.
    //
    // Total number of vertices:            12
    //   -------------------------------------
    //   vertices lying in half space:       4
    //   vertices from split edges:          8
    //
    // Each of the +/- X, Y faces of the box has two triangles. They get
    // clipped into a triangle and quad. Furthermore, the -Z face has two
    // triangles included directly. That leads to 10 total intersection
    // *polygons*.
    //
    // If, however, we are creating a TriangleSurfaceMesh-valued result, each of
    // those faces will be tessellated around its centroid, adding a further 10
    // vertices. Furthermore, there are six triangles and four quads.
    // Tessellating the quads will introduce an *additional* three triangles per
    // quad (4 * 3 = 12 triangles). Tessellating the original triangles will
    // introduce an additional two triangles each (2 * 6 = 12 triangles). So,
    // the TriangleSurfaceMesh will have an addition 24 triangles.
    const MeshType* mesh_W{};
    const MeshFieldLinear<T, MeshType>* field_W{};
    int expected_vert_count = 12;  // Baseline.
    int expected_face_count = 10;
    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
      expected_vert_count += 10;
      expected_face_count += 24;
      mesh_W = &surface->tri_mesh_W();
      field_W = &surface->tri_e_MN();
    } else {
      mesh_W = &surface->poly_mesh_W();
      field_W = &surface->poly_e_MN();
    }

    // We'll assert that the correct number of vertices *and* faces is
    // sufficient evidence to show that ComputeContactSurfaces is correctly
    // calling ConstructTriangleHalfspaceIntersectionPolygon().
    ASSERT_EQ(mesh_W->num_vertices(), expected_vert_count);
    ASSERT_EQ(mesh_W->num_elements(), expected_face_count);
    // The field has been constructed with the same mesh is evidence that the
    // surface has been constructed from the right pieces.
    ASSERT_EQ(&field_W->mesh(), mesh_W);

    // Confirm that we've sampled the half space gradient per face. This test is
    // formulated without assumptions about whether the half space is M or N for
    // the contact surface.
    if (surface->id_M() == half_space_id) {
      ASSERT_TRUE(surface->HasGradE_M());
      for (int f = 0; f < mesh_W->num_elements(); ++f) {
        EXPECT_TRUE(CompareMatrices(surface->EvaluateGradE_M_W(f), grad_p_W));
      }
    } else if (surface->id_N() == half_space_id) {
      ASSERT_TRUE(surface->HasGradE_N());
      for (int f = 0; f < mesh_W->num_elements(); ++f) {
        EXPECT_TRUE(CompareMatrices(surface->EvaluateGradE_N_W(f), grad_p_W));
      }
    } else {
      GTEST_FAIL()
          << "The half space id was not referenced in the contact surface.";
    }
  }
}

REGISTER_TYPED_TEST_SUITE_P(MeshHalfSpaceValueTest, NoIntersection,
                            InsideOrOnIntersection,
                            VertexOnHalfspaceIntersection,
                            EdgeOnHalfspaceIntersection, QuadrilateralResults,
                            OutsideInsideOn, OneInsideTwoOutside,
                            ComputeContactSurfaceInvocation);

// The ComputeContactSurfaceFromSoftHalfSpaceRigidMesh() method has the
// following responsibilities:
//
//    - Dispatch BVH culling
//    - If no candidates, return nullptr.
//    - Otherwise, pass candidates to ComputeContactSurface() and return its
//      return value directly. This function maps the enumeration type to the
//      appropriate mesh representation type.
//
// It is impossible to test correct BVH use (it's all fully contained within
// the implementation). So, we'll focus on intersecting and non-intersecting
// configurations and make sure the data reported is as expected. This function
// does its work independent of scalar type, so we'll only check the
// double-valued instantiation.
GTEST_TEST(ComputeContactSurfaceFromSoftHalfSpaceRigidMeshTest, DoubleValued) {
  // An arbitrary relationship between Frames W and F -- avoiding additive and
  // multiplicative identities.
  const RigidTransform<double> X_WF(
      RotationMatrix<double>(
          AngleAxis<double>{M_PI / 7, Vector3<double>{1, 2, 3}.normalized()}),
      Vector3<double>{-0.25, 0.5, 0.75});

  // Orient and position the mesh arbitrarily in Frame F.
  const RigidTransform<double> X_FB(
      math::RollPitchYaw<double>(M_PI_4, M_PI_4, M_PI_4),
      Vector3<double>(1.0, 2.0, 3.0));
  const TriangleSurfaceMesh<double> mesh_F = CreateBoxMesh(X_FB);
  const GeometryId mesh_id = GeometryId::get_new_id();
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_F(mesh_F);

  // Construct the half-space.
  const GeometryId hs_id = GeometryId::get_new_id();
  const double pressure_scale{1.5};

  {
    // Case: non-intersecting.

    // Put the half space well below the box.
    const RigidTransform<double> X_WH{Vector3<double>{0, 0, -5}};
    EXPECT_EQ(ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
                  hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF,
                  HydroelasticContactRepresentation::kTriangle),
              nullptr);
    EXPECT_EQ(ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
                  hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF,
                  HydroelasticContactRepresentation::kPolygon),
              nullptr);
  }

  // Define an intersecting configuration.
  // The half space is defined in Frame H. The box is defined in Frame B.
  // Have the half space's boundary plane lie _near_ the box's origin and its
  // normal not quite aligned with Bz -- the goal is to get a contact surface
  // that has the same general _topology_ as the
  // (MeshHalfSpaceIntersectionTest, BoxMesh) test (see above), but not
  // actually have the half space perfectly aligned with the mesh.
  // We pick a rotation roughly 5 degrees away from Bz and a small offset
  // from the origin.
  //
  // We will simply confirm that the resulting ContactSurface exists and
  // respects the requested representation.
  const RigidTransform<double> X_BH{
      AngleAxis<double>{M_PI * 0.027, Vector3d{1, 1, 0}.normalized()},
      Vector3<double>{0.1, -0.05, 0.075}};
  const RigidTransform<double> X_WH = X_WF * X_FB * X_BH;

  // Case: Request triangle surface mesh.
  {
    const std::unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
            hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF,
            HydroelasticContactRepresentation::kTriangle);
    ASSERT_NE(contact_surface, nullptr);
    ASSERT_EQ(contact_surface->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }

  // Case: Request polygon surface mesh.
  {
    const std::unique_ptr<ContactSurface<double>> contact_surface =
        ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
            hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF,
            HydroelasticContactRepresentation::kPolygon);
    ASSERT_NE(contact_surface, nullptr);
    ASSERT_EQ(contact_surface->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Confirm that the rigid-soft intersection correctly culls backface geometry.
// Culling is independent of ultimate contact surface mesh representation, so
// we'll simply test it against one mesh and scalar type and call it good. We
// choose the TriangleSurfaceMesh so we can invoke
// IsFaceNormalInNormalDirection() as part of the test.
GTEST_TEST(ComputeContactSurfaceFromSoftHalfSpaceRigidMeshTest, BackfaceCull) {
  // For this test, we're just testing for culling. We presume that for a given
  // configuration of mesh and half space, the right calculations will be done
  // to compute the intersection mesh based on previous tests. So, accordingly:
  //
  //   1. We'll leave the box centered on the world frame origin and aligned
  //      with the world frame basis.  This implies that B = F = W.
  //   2. However, we'll pose the half space in the world frame such that:
  //      a. its normal_W is *not* Wz (so we can confirm that the correct normal
  //         value is used for culling).
  //      b. it is positioned such that the *entire* box is contained in the
  //         half space's volume.

  const RigidTransform<double> X_WF, X_FB;  // Both the identity.

  const TriangleSurfaceMesh<double> mesh_F = CreateBoxMesh(X_FB);
  const GeometryId mesh_id = GeometryId::get_new_id();
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_F(mesh_F);

  // Construct the half-space.
  const GeometryId hs_id = GeometryId::get_new_id();
  // We align the half space normal along a diagonal of the box (to ease the
  // prediction of what the outcome should be -- see below).
  const Vector3<double> normal_W = Vector3<double>{1, -1, -1}.normalized();
  const RigidTransform<double> X_WH =
      HalfSpace::MakePose(normal_W, Vector3<double>{0, 0, -30});
  const double pressure_scale{1.5};

  const std::unique_ptr<ContactSurface<double>> contact_surface =
      ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          hs_id, X_WH, pressure_scale, mesh_id, mesh_F, bvh_F, X_WF,
          HydroelasticContactRepresentation::kTriangle);
  // It definitely produces a contact surface.
  ASSERT_NE(contact_surface, nullptr);

  const TriangleSurfaceMesh<double>& mesh_W = contact_surface->tri_mesh_W();

  // For a wholly enclosed mesh _without_ backface culling, the contact
  // surface would be the full mesh. The input mesh has two triangles per face
  // for a total of 12 triangles and 8 vertices.
  //
  // In fact, because the face normal points along one of the box diagonals.
  // Only three of the box's faces will contribute to the contact surface. So,
  // we'd have 6 faces (2 triangles per face). For vertices, we'll include all
  // but one of the input mesh's vertices (7).
  //
  // For the TriangleSurfaceMesh, those faces would get tessellated around
  // their centroids. So, instead of 6 3-sided polygons, we have 18 triangles.
  // And in addition to the 7-input vertices, we add 6 centroids.
  ASSERT_EQ(mesh_W.num_elements(), 18);
  ASSERT_EQ(mesh_W.num_vertices(), 13);

  // Let's explicitly confirm that every face in the contact surface mesh
  // passes the culling test.
  // There are two tricky bits in setting up this test.
  //
  //   1. The gradient of the half space pressure is in the opposite direction
  //      as its normal. So, in normal circumstances: grad_p_W = -normal_W.
  //   2. We only know that the normals of the contact surface mesh point
  //      *out* of geometry N and *into* geometry M (in the contact
  //      surface). We don't know what the mapping between those ids and the
  //      mesh and half space ids are. If id N is the same as the mesh_id,
  //      then we know the contact surface normals point in the mesh normal
  //      direction. We're done. However, if mesh_id is the id for M, the
  //      contact surface mesh normals point in the opposite direction as the
  //      input mesh. So, to evaluate an _equivalent_ culling test, we should
  //      likewise reverse the gradient direction. That's what the next two
  //      lines of code do to define the gradient direction vector.
  const double grad_scale = contact_surface->id_N() == mesh_id ? 1.0 : -1.0;
  const Vector3<double> grad_p_W = grad_scale * -normal_W;
  for (int tri = 0; tri < mesh_W.num_elements(); ++tri) {
    // Everything is defined in the world frame; so the transform X_WM = I.
    ASSERT_TRUE(IsFaceNormalInNormalDirection(grad_p_W, mesh_W, tri, {}))
        << "Face " << tri;
  }

  // Note: this confirms that the reported faces satisfy the non-culling
  // criteria, but doesn't (can't) confirm that the omitted faces don't. Rethink
  // this test if that proves to be a problem (highly unlikely).
}

typedef ::testing::Types<
    TriangleSurfaceMesh<double>, TriangleSurfaceMesh<AutoDiffXd>,
    PolygonSurfaceMesh<double>, PolygonSurfaceMesh<AutoDiffXd>>
    MyTypes;
INSTANTIATE_TYPED_TEST_SUITE_P(My, MeshHalfSpaceValueTest, MyTypes);

/* This test fixture enables some limited testing of the autodiff-valued contact
 surface. It computes the intersection between a half space and a simple
 triangle mesh (single tri).

 The rigid triangle, defined in the frame R, is simply:

             Rz               The triangle's normal points in the Ry direction.
             ┆   Ry           Vertex v1 is at <1, 0, 0>.
             ┆  ╱             Vertex v2 is at <0, 0, -1>.
             ┆ ╱
          v0 ┆╱    v1
      ┄┄┄┄┄┄┄●┄┄┄┄┄●┄┄┄ Rx
            ╱┆
           ╱ ┆
          ╱  ● v2

  The soft half space, defined in the frame S, has its planar boundary at Sz = 0
  with the normal in the +Sz direction.

  We will create a number of fixed poses of the triangle w.r.t. the half space:

    - horizontal slice: R_RS = I, v2 lies inside the half space and the edge
      connecting vertices 0 and 1 lies parallel with the half space boundary. It
      slices the full triangle into a smaller, similar triangle.
    - skew triangle: v2 lies inside, vertices 0 and 1 lie outside, but no edges
      are parallel with either planar boundary or plane normal. The intersecting
      polygon is a triangle.
    - skew quad: vertices v1 and v2 lie inside the half space, v0 lies outside.
      Again, no triangle edges are aligned with plane or normal. The
      intersecting polygon is a quad.

 The function TestPositionDerivative() will pose the triangle mesh and compute
 a contact surface. It invokes a provided functor to assess the reported
 derivatives of some arbitrary quantity of the contact surface with respect to
 the position of the origin of frame R.

 This test exclusively uses the TriangleSurfaceMesh output type. The reason for
 this is that the introduction of the centroid is a very convenient property to
 use for evaluating correct propagation of derivatives. All other face,
 vertices, and per-feature values would be equivalent for the two mesh
 representation types. */
class MeshHalfSpaceDerivativesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    /* Set up the *soft* half space. */
    id_S_ = GeometryId::get_new_id();
    pressure_scale_ = 1.5e5;
    /* Orient and position the soft half space arbitrarily. */
    X_WS_ = HalfSpace::MakePose(Vector3d{1, 2, 3}.normalized(),
                                Vector3d{0.25, 0.1, -0.2})
                .cast<AutoDiffXd>();

    /* Set up the *rigid* mesh. */
    vector<Vector3d> vertices{Vector3d{0, 0, 0}, Vector3d{1, 0, 0},
                              Vector3d{0, 0, -1}};
    using Face = SurfaceTriangle;
    vector<Face> faces{{Face{0, 1, 2}}};
    id_R_ = GeometryId::get_new_id();
    mesh_R_ =
        make_unique<TriangleSurfaceMesh<double>>(move(faces), move(vertices));
    bvh_R_ = make_unique<Bvh<Obb, TriangleSurfaceMesh<double>>>(*mesh_R_);
  }

  /* Indicator for the relative pose of the mesh (tri) relative to the half
   space. See class documentation for details. */
  enum TriPose { kHorizontalSlice, kSkewTriangle, kSkewQuad };

  /* Tests for an arbitrary quantity of the contact surface against multiple
   relative poses between mesh and half space. We evaluate three different
   configurations (as documented above). For each configuration, we invoke
   evaluate_quantity(). The contact surface is being differentiated with respect
   to the position of the rigid frame's origin in the world frame, p_WRo.

   @param evaluate_quantity  A function that assess some aspect of the contact
                             surface and its derivatives. It must be written
                             to account for the documented relative pose between
                             half space and mesh. The provided function should
                             make use of googletest EXPECT_* macros to perform
                             the assessment. */
  void TestPositionDerivative(
      const std::function<void(const ContactSurface<AutoDiffXd>&,
                               const RigidTransform<AutoDiffXd>&, TriPose)>&
          evaluate_quantity) {
    /* Definition of a test configuration: a name, a relative pose between soft
     and rigid frames, encoded as a double-valued (_d suffix) relative position
     between frame origins and relative orientation, the expected number of
     faces in the resultant contact surface, and the pose enumeration that the
     `evaluate_quantity` method will use to define the expected results. */
    struct Configuration {
      std::string name;
      Vector3d p_SR_d;
      RotationMatrixd R_SR_d;
      int expected_num_faces{};
      TriPose pose;
    };
    vector<Configuration> configurations;

    const RigidTransformd X_WS_d = convert_to_double(X_WS_);
    // We want to make sure that p_SoRo is completely non-zero. So, we pick
    // a point N on the boundary of the half space offset from its origin and
    // position the triangle with respect to that point.
    const Vector3d p_SN{0.25, -0.3, 0};
    {
      /* Leave the triangle unrotated, i.e. R_RS = I. The contact mesh will
       be a "horizontal" slice of the triangle: a right isosceles triangle. */
      const Vector3d n_S{0, 0, 1};
      const Vector3d p_SR_d{p_SN + (1 - kDepth) * n_S};
      configurations.push_back(
          {"Horizontal slice", p_SR_d, RotationMatrixd{}, 3, kHorizontalSlice});
    }

    {
      /* We'll pose the triangle in the soft half space's frame S. If we start
       with bases S and R aligned, we'll rotate the triangle around Sy so that
       edge 01 is no longer parallel with the Sz = 0 plane, and then rotate
       around Sx so that the edge 02 isn't parallel with Sz. Finally, we'll
       position vertex 2 displaced from N and inset into the half space kDepth
       distance.

       The rotation around Sx must be less than pi / 4 to ensure the
       intersection doesn't get culled. */
      const RotationMatrixd R_SR_d = RotationMatrixd::MakeXRotation(-M_PI / 7) *
                                     RotationMatrixd::MakeYRotation(-M_PI / 6);

      const Vector3d p_RoV2_S = R_SR_d * mesh_R_->vertex(2);
      const Vector3d p_SV2 = p_SN + Vector3d{0, 0, -kDepth};
      const Vector3d p_SR_d = p_SV2 - p_RoV2_S;
      configurations.push_back(
          {"Skewed triangle", p_SR_d, R_SR_d, 3, kSkewTriangle});
    }

    {
      /* We'll pose the triangle in the soft half space's frame S. If we start
       with bases S and R aligned, we'll rotate the triangle around Sy so that
       vertices 1 & 2 lie inside the half space. We'll rotate it between zero
       and 45 degrees for two reasons: keep V2 as the deepest vertex and prevent
       the edge connecting V1 and V2 from lying parallel with the plane.
       Finally, we'll tilt the triangle around the Sx axis to break alignment.
       */
      const RotationMatrixd R_SR_d = RotationMatrixd::MakeXRotation(-M_PI / 7) *
                                   RotationMatrixd::MakeYRotation(M_PI / 4.5);

      const Vector3d p_RoV2_S = R_SR_d * mesh_R_->vertex(2);
      const Vector3d p_SV2 = p_SN + Vector3d{0, 0, -kDepth};
      const Vector3d p_SR_d = p_SV2 - p_RoV2_S;
      configurations.push_back({"Skewed quad", p_SR_d, R_SR_d, 4, kSkewQuad});
    }

    for (const auto& config : configurations) {
      const RotationMatrixd R_WR_d = X_WS_d.rotation() * config.R_SR_d;
      const Vector3d p_WR_d = X_WS_d * config.p_SR_d;
      const Vector3<AutoDiffXd> p_WR = math::InitializeAutoDiff(p_WR_d);
      const RigidTransform<AutoDiffXd> X_WR{R_WR_d.cast<AutoDiffXd>(), p_WR};

      auto surface = ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
          id_S_, X_WS_, pressure_scale_, id_R_, *mesh_R_, *bvh_R_, X_WR,
          HydroelasticContactRepresentation::kTriangle);

      SCOPED_TRACE(config.name);
      ASSERT_NE(surface, nullptr);
      ASSERT_EQ(surface->tri_mesh_W().num_triangles(),
                config.expected_num_faces);

      evaluate_quantity(*surface, X_WR, config.pose);
    }
  }

  /* Given the point E which purports to lie on an edge of the triangular mesh,
   finds the edge it lies on (spanning vertices A and B) and returns p_AB_R. */
  Vector3d GetEdgeDirInR(const Vector3d& p_RE) const {
    // We determine the edge that E lies on with this simple metric. If E lies
    // on edge AB, then AB⋅AE = |AB|⋅|AE|, or, to avoid square roots:
    // (AB⋅AE)² = |AB|²⋅|AE|²
    const vector<Vector3d>& verts_R = mesh_R_->vertices();
    const vector<pair<int, int>> edges{{0, 1}, {0, 2}, {1, 2}};
    for (const auto& [a, b] : edges) {
      const Vector3d p_AB_R = verts_R[b] - verts_R[a];
      const Vector3d p_AE_R = p_RE - verts_R[a];
      const double lhs = std::pow(p_AB_R.dot(p_AE_R), 2);
      const double rhs = p_AB_R.squaredNorm() * p_AE_R.squaredNorm();
      if (std::abs(lhs - rhs) < 1e-15) {
        return p_AB_R;
      }
    }
    throw std::logic_error(
        "Querying for point I that isn't actually on a triangle edge");
  }

  /* The soft half space. */
  RigidTransform<AutoDiffXd> X_WS_;
  GeometryId id_S_;
  double pressure_scale_{};

  /* The rigid mesh. */
  GeometryId id_R_;
  unique_ptr<TriangleSurfaceMesh<double>> mesh_R_;
  unique_ptr<Bvh<Obb, TriangleSurfaceMesh<double>>> bvh_R_;

  /* The amount we penetrate the triangle mesh into the half space.  */
  static constexpr double kDepth = 0.25;
};

TEST_F(MeshHalfSpaceDerivativesTest, Area) {
  /* We'll compute the expected contact surface area (and its derivatives) by
   decomposing it into triangles. For a triangle defined by vertices A, B, and
   C, with triangle normal n̂ (and assuming that the triangle winding is
   consistent with the normal direction):

      Area = 0.5 * |(B - A) × (C - A)|₂
           = 0.5 * [(B - A) × (C - A)]ᵀn̂
           = 0.5 * [skew_sym(B - A) (C-A)]ᵀn̂

        where, a x b = skew_sym(a) b and

                          │  0 -a3  a2│
            skew_sym(a) = │ a3   0 -a1│
                          │-a2  a1   0│

    ∂Area          ∂[skew_sym(B - A) (C-A)]ᵀ
    ────── = 0.5 * ─────────────────── n̂
      ∂Ro                ∂Ro

                   │                  ∂(B - A)                    ∂(C - A) │ᵀ
           = 0.5 * │ -skew_sym(C - A) ────────  + skew_sym(B - A) ──────── │ n̂
                   │                    ∂Ro                          ∂Ro   │

   We are *given* the quantities ∂A/∂Ro, ∂B/∂Ro, and ∂C/∂Ro which have been
   independently validated by the VertexPosition test. */
  auto evaluate_area = [X_WS = convert_to_double(this->X_WS_)](
                           const ContactSurface<AutoDiffXd>& surface,
                           const RigidTransform<AutoDiffXd>& X_WR_ad,
                           TriPose pose) {
    const auto& mesh_W = surface.tri_mesh_W();

    // For v × A, this makes a matrix V such that VA = v × A.
    auto skew_matrix = [](const Vector3d& v) {
      Matrix3<double> result;
      // clang-format off
      result <<     0, -v(2),  v(1),
                 v(2),     0, -v(0),
                -v(1),  v(0),   0;
      // clang-format on
      return result;
    };

    /* We don't want to compute the areas of the *individual* triangles in the
     contact surface because it is triangle fan around a centroid. The fan and
     centroid contribute nothing to the area. So, as an independent witness,
     we'll compute the area of the *polygon* based on the vertices on the
     perimeter. The triangles defined below are based on a priori knowledge
     of the behavior of the mesh-half space intersection algorithm. If it
     changes, these hard-coded indices could become invalid. A more robust
     solution would be to *infer* the boundary polygon edges from the contact
     surface, but that's a lot of effort for little present value. */
    vector<vector<int>> triangles;
    switch (pose) {
      case TriPose::kHorizontalSlice:
      case TriPose::kSkewTriangle:
        triangles.emplace_back(vector<int>{0, 1, 2});
        break;
      case TriPose::kSkewQuad:
        /* This is a bit brittle and is predicated on knowledge of how
         the intersection algorithm processes the particular geometry. If that
         proves to be too brittle, we'll need to reconstruct this by looking
         at the provided mesh. */
        triangles.emplace_back(vector<int>{2, 1, 0});
        triangles.emplace_back(vector<int>{2, 3, 1});
        break;
    }

    /* The normal for the contact surface is simply Ry (here, expressed in
     world). */
    const Vector3d n_W =
        convert_to_double(Vector3<AutoDiffXd>(X_WR_ad.rotation().col(1)));

    double area_expected = 0;
    Vector3d dArea_dRo_expected = Vector3d::Zero();
    for (const auto& tri : triangles) {
      const auto& p_WA_ad = mesh_W.vertex(tri[0]);
      const auto& p_WB_ad = mesh_W.vertex(tri[1]);
      const auto& p_WC_ad = mesh_W.vertex(tri[2]);
      const Vector3d p_WA = convert_to_double(p_WA_ad);
      const Vector3d p_WB = convert_to_double(p_WB_ad);
      const Vector3d p_WC = convert_to_double(p_WC_ad);
      const Matrix3<double> dA_dRo = math::ExtractGradient(p_WA_ad);
      const Matrix3<double> dB_dRo = math::ExtractGradient(p_WB_ad);
      const Matrix3<double> dC_dRo = math::ExtractGradient(p_WC_ad);

      const Vector3d p_AB_W = p_WB - p_WA;
      const Vector3d p_AC_W = p_WC - p_WA;
      const double tri_area = 0.5 * p_AB_W.cross(p_AC_W).dot(n_W);
      area_expected += tri_area;

      const Matrix3<double> left = -skew_matrix(p_AC_W) * (dB_dRo - dA_dRo);
      const Matrix3<double> right = skew_matrix(p_AB_W) * (dC_dRo - dA_dRo);
      dArea_dRo_expected += 0.5 * ((left + right).transpose() * n_W);
    }

    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const AutoDiffXd& total_area = mesh_W.total_area();
    EXPECT_NEAR(total_area.value(), area_expected, 4 * kEps);
    EXPECT_TRUE(CompareMatrices(total_area.derivatives(), dArea_dRo_expected,
                                4 * kEps));
  };

  TestPositionDerivative(evaluate_area);
}

TEST_F(MeshHalfSpaceDerivativesTest, VertexPosition) {
  /* In principle, we might expect that the derivatives of contact surface
   vertex position, v, with respect to Ro, ∂v/∂Ro, would simply be the identity.
   After all, the mesh vertices are affixed to the mesh. This is not entirely
   true. In reality, it depends on where the vertex in the contact surface mesh
   comes from:

   - Vertices from mesh_R_ inside the half space
     - These vertices *can* be considered to be rigidly affixed to the mesh,
      so ∂v/∂Ro = I is correct.

   - For vertices at the intersection of plane and tri edge, the derivative of
     the vertex position w.r.t. the position of the mesh origin (Ro) can be
     derived as follows:

     The point V lies on an edge of a triangle. We'll characterize the edge as
     a one of the two end points A, and the *direction* of the edge from A to
     the other vertex, ê. Therefore (with everything measured and expressed
     in the S frame),

         V = A + tê

     And we know that V.z = 0 because it lies on the half space surface. Hence,
     the scalar t (which is equal to the length |AV|) equals -A.z/ê.z and we can
     write V as a function of A:

             | A.x - ê.x * A.z / ê.z |
         V = | A.y - ê.y * A.z / ê.z |
             |            0          |

     So, since A is fixed to the triangle frame R (i.e., ∂A / ∂Ro = I), we see
     that:

                   | 1  0  -ê.x / ê.z |
          ∂V/∂Ro = | 0  1  -ê.y / ê.z |
                   | 0  0       0     |

                   | 1  0  -e.x / e.z |   // We don't actually require ê, e
          ∂V/∂Ro = | 0  1  -e.y / e.z |   // will suffice.
                   | 0  0       0     |

     However, the derivatives reported in the test are ∂p_WV/∂p_WRo, so we have
     to transform the expected result from the soft frame S to world.

          ∂p_WV    ∂(R_WS⋅p_SV + p_WSo)
         ------- = --------------------          // Expand p_WV = X_WS * p_SV.
          ∂p_WRo         ∂p_WRo

                   ∂(R_WS⋅p_SV)
                 = -------------                 // p_WSo doesn't depend on Ro.
                      ∂p_WRo

                   ∂(R_WS⋅p_SV)     ∂p_SRo
                 = ------------- * --------      // Chain rule.
                      ∂p_SRo        ∂p_WRo

                   ∂(R_WS⋅p_SV)                  // Change of position in S is
                 = ------------- * R_SW          // related to change in W by
                      ∂p_SRo                     // R_SW.

                           ∂p_SV
                 = R_WS * -------- * R_SW        // R_WS doesn't depend on Ro.
                           ∂p_SRo

                          | 1 0 -e.x / e.z |
                 = R_WS * | 0 1 -e.y / e.z | * R_SW  // Definition of ∂V/∂Ro.
                          | 0 0      0     |

   - Vertices at the centroid of intersection polygons. Every polygon formed by
     clipping a mesh triangle is tessellated around its centroid. For centroids
     of triangles (where the intersecting polygon between tri and half space is
     a triangle), the centroid position and its derivative are simply:

                 C = (v0 + v1 + v2) / 3, and
            ∂C/∂Ro = (∂v0/∂Ro + ∂v1/∂Ro + ∂v2/∂Ro) / 3

     We skip the centroid for polygons with four or more vertices. Those
     centroids are computed by decomposing the polygon into triangles. For each
     triangle we compute centroid and area and then define the polygon centroid
     as a weighted average of the triangle centroids. There is no simple way to
     validate the derivatives of this vertex, so we'll skip it for now.

   Finally, this test exploits special knowledge that when a polygon with N
   vertices is produced by intersection, N + 1 vertices are added to the contact
   surface mesh: the polygon's N vertices followed by the centroid. In this
   test, we can use that to implicitly recognize which vertices come from edge
   intersections and which are centroids (combined with the fact that there's
   only a single polygon in the contact surface mesh). */
  auto evalute_position = [this](const ContactSurface<AutoDiffXd>& surface,
                                 const RigidTransform<AutoDiffXd>& X_WR_ad,
                                 TriPose pose) {
    constexpr double kEps = 5 * std::numeric_limits<double>::epsilon();

    /* The test is set up so there is only ever a single intersecting polygon.
     So, there is *one* centroid (the last vertex). All other vertices come
     from intersecting a tri edge with the plane or are mesh vertices embedded
     in the half space. We'll evaluate all of those and then handle the centroid
     specially. */
    const Vector3d n_S{0, 0, 1};
    const TriangleSurfaceMesh<AutoDiffXd>& mesh_W = surface.tri_mesh_W();
    const RigidTransformd X_WS = convert_to_double(this->X_WS_);
    const RotationMatrixd& R_WS = X_WS.rotation();
    const RotationMatrixd R_SW = R_WS.inverse();
    const RigidTransformd X_WR = convert_to_double(X_WR_ad);
    const RotationMatrixd& R_WR = X_WR.rotation();

    for (int v = 0; v < mesh_W.num_vertices() - 1; ++v) {
      const Vector3<AutoDiffXd>& p_WV_ad = mesh_W.vertex(v);
      const Vector3d p_WV = convert_to_double(p_WV_ad);
      const Vector3d p_SV = X_WS.inverse() * p_WV;
      if (p_SV.z() < -1e-10) {
        /* Because this point lies *below* the half space surface (and we know
         it is not the centroid), this must be one of mesh_R's vertices inside
         the half space. It is rigidly affixed to R so the derivatives are just
         the identity. */
        const Matrix3<double> J_W = math::ExtractGradient(p_WV_ad);
        ASSERT_TRUE(CompareMatrices(J_W, Matrix3<double>::Identity(), kEps));
        continue;
      }
      const Vector3d e_S = R_SW * R_WR * GetEdgeDirInR(X_WR.inverse() * p_WV);
      Matrix3<double> expected_J_S;
      // clang-format off
      expected_J_S << 1,  0, -e_S.x() / e_S.z(),
                      0,  1, -e_S.y() / e_S.z(),
                      0,  0,  0;
      // clang-format on
      const Matrix3<double> expected_J_W =
          R_WS * (expected_J_S * R_SW.matrix());
      const Matrix3<double> J_W = math::ExtractGradient(p_WV_ad);
      ASSERT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
    }

    /* Now handle the centroid. */
    switch (pose) {
      case kHorizontalSlice:
      case kSkewTriangle: {
        /* The derivative of the centroid should simply be the mean of the first
         three vertices' derivatives. */
        Matrix3<double> expected_J_W = Matrix3<double>::Zero();
        for (int v = 0; v < 3; ++v) {
          expected_J_W += math::ExtractGradient(mesh_W.vertex(v));
        }
        expected_J_W /= 3;
        const Vector3<AutoDiffXd>& p_WC = mesh_W.vertex(3);
        const Matrix3<double> J_W = math::ExtractGradient(p_WC);
        EXPECT_TRUE(CompareMatrices(J_W, expected_J_W, kEps));
        break;
      }
      case kSkewQuad:
        /* We skip the centroid for the quad case. */
        break;
    }
  };

  TestPositionDerivative(evalute_position);
}

TEST_F(MeshHalfSpaceDerivativesTest, FaceNormalsWrtPosition) {
  /* None of the face normals depend on Ro. They should all report the zero
   matrix. */
  auto evaluate_normals = [](const ContactSurface<AutoDiffXd>& surface,
                             const RigidTransform<AutoDiffXd>&, TriPose) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const Matrix3<double> zero_matrix = Matrix3<double>::Zero();
    for (int t = 0; t < surface.tri_mesh_W().num_elements(); ++t) {
      const Vector3<AutoDiffXd> n_W = surface.tri_mesh_W().face_normal(t);
      EXPECT_TRUE(CompareMatrices(math::ExtractGradient(n_W),
                                  zero_matrix, 32 * kEps));
    }
  };

  TestPositionDerivative(evaluate_normals);
}

TEST_F(MeshHalfSpaceDerivativesTest, FaceNormalsWrtOrientation) {
  /* The normals of the contact surface are always parallel with the normals on
   the rigid mesh. Therefore, as the mesh normals change, so should the contact
   surface normals.

   So, we'll define the rotation matrix R_WR as an angle of θ radians around an
   arbitrary unit vector v̂ and differentiate with respect to θ. The mesh normals
   n̂ point in the Ry direction. We can show that the expected derivative is
   dn̂/dθ = v̂ × n̂ = v̂ × Ry as follows:

       R_WR = exp(skew_sym(v̂)*θ)
        n̂_W = R_WR * Ry_R

    dn̂_W/dθ = d(R_WR * Ry_R)/dθ
            = dR_WR/dθ * Ry_R
            = skew_sym(v̂) * exp(skew_sym(v̂)*θ) * Ry_R
            = v̂ × n̂

   This test does *not* use the TestPositionDerivative() API because that
   differentiates with respect to Ro and makes assumptions about the resulting
   mesh. For this test, those assumptions would be invalid, so we'll simply
   duplicate that portion of TestPositionDerivative() that is relevant for this
   test. */

  /* Arbitrary rotation axis. It is important that the vector be defined such
   that rotations don't cause all intersecting triangles to be "backface
   culled".  */
  const Vector3d v_S = Vector3d{-1, 2, -3}.normalized();
  for (const double theta : {0.0, M_PI / 6, M_PI / 2 * 0.9, M_PI / 2 * 0.99}) {
    /* We'll push the origin down into the half space to guarantee
     intersection without doing otherwise unnecessary analysis). We're only
     testing normal direction; any intersection is sufficient. The x- and y-
     elements are arbitrary, non-zero values. */
    const Vector3<AutoDiffXd> p_WR =
        this->X_WS_ * Vector3<AutoDiffXd>{0.25, -0.7, 0.1};
    AutoDiffXd theta_ad = theta;
    theta_ad.derivatives().resize(1);
    theta_ad.derivatives() << 1;
    RotationMatrix<AutoDiffXd> R_SR(AngleAxis<AutoDiffXd>(theta_ad, v_S));
    RigidTransform<AutoDiffXd> X_WR{this->X_WS_.rotation() * R_SR, p_WR};
    const Vector3d v_W = convert_to_double(this->X_WS_).rotation() * v_S;

    auto surface = ComputeContactSurfaceFromSoftHalfSpaceRigidMesh(
        id_S_, X_WS_, pressure_scale_, id_R_, *mesh_R_, *bvh_R_, X_WR,
        HydroelasticContactRepresentation::kTriangle);

    SCOPED_TRACE(fmt::format("theta = {:.5f} radians", theta));
    /* surface != nullptr --> num_elements() > 0 as documented in the API but
     we'll explicitly confirm to make sure the test doesn't pass by accident. */
    ASSERT_NE(surface, nullptr);
    ASSERT_GT(surface->tri_mesh_W().num_elements(), 0);

    /* Test dn̂/dθ = v̂ × n̂  = v̂ × Ry. */
    const Vector3d Ry_W = math::ExtractValue(X_WR.rotation().col(1));
    const Vector3d expected_deriv = v_W.cross(Ry_W);
    for (int t = 0; t < surface->tri_mesh_W().num_elements(); ++t) {
      const auto& n = surface->tri_mesh_W().face_normal(t);
      /* Precision decreases as the mesh gets closer to lying parallel to the
       half space surface. This simple switch accounts for the observed
       behavior in this test. */
      EXPECT_TRUE(CompareMatrices(math::ExtractGradient(n),
                                  expected_deriv, theta > 1.3 ? 1e-13 : 5e-15));
    }
  }
}

TEST_F(MeshHalfSpaceDerivativesTest, Pressure) {
  /* The pressure at a point v embedded in a half space with normal n̂ and point
   on its surface o is:

          p = En̂ᵀ(o − v)

     ∂p/∂Ro = ∂[En̂ᵀ(o − v)]/∂Ro
            = En̂ᵀ∂[o − v]/∂Ro
            = En̂ᵀ[∂o/∂Ro − ∂v/∂Ro]
            = -En̂ᵀ∂v/∂Ro

   We've already tested the derivative of vertex position w.r.t. Ro, so we can
   make use of that to confirm that the pressure derivatives are consistent. */
  auto evaluate_pressures = [E = this->pressure_scale_,
                             X_WS = convert_to_double(this->X_WS_)](
                                const ContactSurface<AutoDiffXd>& surface,
                                const RigidTransform<AutoDiffXd>&,
                                double theta) {
    constexpr double kEps = std::numeric_limits<double>::epsilon();
    const Vector3d& n_W = X_WS.rotation().col(2);
    for (int v = 0; v < surface.tri_mesh_W().num_vertices(); ++v) {
      const auto& p_WV = surface.tri_mesh_W().vertex(v);
      const Matrix3<double> dV_dRo = math::ExtractGradient(p_WV);
      const Vector3d expected_dp_dRo = -E * n_W.transpose() * dV_dRo;
      const AutoDiffXd& p = surface.tri_e_MN().EvaluateAtVertex(v);
      EXPECT_TRUE(
          CompareMatrices(p.derivatives(), expected_dp_dRo, 3 * E * kEps));
    }
  };

  TestPositionDerivative(evaluate_pressures);
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
