#include "drake/geometry/proximity/volume_mesh.h"

#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/volume_mesh_field.h"
#include "drake/math/autodiff.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Remove this helper class if we change
//  CalcGradBarycentric() from private to public.
template <typename T>
class VolumeMeshTester {
 public:
  explicit VolumeMeshTester(const VolumeMesh<T>& mesh) : mesh_(mesh) {}
  Vector3<T> CalcGradBarycentric(int e, int i) const {
    return mesh_.CalcGradBarycentric(e, i);
  }

 private:
  const VolumeMesh<T>& mesh_;
};

namespace {

using math::RigidTransform;
using math::RollPitchYaw;

template <typename T>
VectorX<T> ToVectorX(const std::vector<Vector3<T>>& input) {
  VectorX<T> result(input.size() * 3);
  for (int v = 0, i = 0; v < ssize(input); ++v, i += 3) {
    result.template segment<3>(i) = input[v];
  }
  return result;
}

// Test instantiation of VolumeMesh of a geometry M and inspecting its
// components. By default, the vertex positions are expressed in M's frame.
// The optional parameter X_WM will change the vertex positions to W's frame.
template <typename T>
std::unique_ptr<VolumeMesh<T>> TestVolumeMesh(
    const RigidTransform<T> X_WM = RigidTransform<T>::Identity(),
    double scale = 1.0) {
  // A  trivial volume mesh comprises of two tetrahedral elements with
  // vertices on the coordinate axes and the origin like this:
  //
  //      +Z
  //       |
  //       v3
  //       |
  //       |
  //     v0+------v2---+Y
  //      /|
  //     / |
  //   v1  v4
  //   /   |
  // +X    |
  //      -Z
  //
  // In the picture above, the positions are expressed in M's frame. The
  // optional parameter X_WM will change the vertex positions to W's frame.
  //
  const int element_data[2][4] = {{0, 1, 2, 3}, {0, 2, 1, 4}};
  std::vector<VolumeElement> elements;
  for (int e = 0; e < 2; ++e) elements.emplace_back(element_data[e]);
  const Vector3<T> vertex_data[5] = {Vector3<T>::Zero(), Vector3<T>::UnitX(),
                                     Vector3<T>::UnitY(), Vector3<T>::UnitZ(),
                                     -Vector3<T>::UnitZ()};
  std::vector<Vector3<T>> vertices_W;
  for (int v = 0; v < 5; ++v) {
    vertices_W.emplace_back(X_WM * (vertex_data[v] * scale));
  }
  auto volume_mesh_W = std::make_unique<VolumeMesh<T>>(std::move(elements),
                                                       std::move(vertices_W));
  EXPECT_EQ(2, volume_mesh_W->num_elements());
  EXPECT_EQ(5, volume_mesh_W->num_vertices());
  for (int v = 0; v < 5; ++v)
    EXPECT_EQ(X_WM * (vertex_data[v] * scale), volume_mesh_W->vertex(v));
  for (int e = 0; e < 2; ++e)
    for (int v = 0; v < 4; ++v)
      EXPECT_EQ(element_data[e][v], volume_mesh_W->element(e).vertex(v));
  return volume_mesh_W;
}

// Test instantiation of VolumeMesh using `double` as the underlying scalar
// type.
GTEST_TEST(VolumeMeshTest, TestVolumeMeshDouble) {
  auto volume_mesh = TestVolumeMesh<double>();
}

GTEST_TEST(VolumeMeshTest, TestDegenerateVolumeMeshDouble) {
  auto volume_mesh =
      TestVolumeMesh<double>(RigidTransform<double>::Identity(), 1e-8);
  std::array<double, 4> dummy_values{1, 2, 3, 4};
  DRAKE_EXPECT_THROWS_MESSAGE(
      volume_mesh->CalcGradientVectorOfLinearField(dummy_values, 0),
      ".*not calculate gradient.*");
  auto result =
      volume_mesh->MaybeCalcGradientVectorOfLinearField(dummy_values, 0);
  EXPECT_FALSE(result.has_value());
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no test of
// differentiation.
GTEST_TEST(VolumeMeshTest, TestVolumeMeshAutoDiffXd) {
  auto volume_mesh = TestVolumeMesh<AutoDiffXd>();
}

template <typename T>
void TestVolumeMeshEqual() {
  const auto mesh = TestVolumeMesh<T>();

  // Mesh equals itself.
  EXPECT_TRUE(mesh->Equal(*mesh));

  // Mesh equals its copy.
  {
    const VolumeMesh<T> mesh_copy = *mesh;
    EXPECT_TRUE(mesh->Equal(mesh_copy));
  }

  // Different positions of vertices.
  {
    const auto mesh_translate =
        TestVolumeMesh<T>(RigidTransform<T>(Vector3<T>(1., 1., 1.)));
    EXPECT_FALSE(mesh->Equal(*mesh_translate));
  }

  // Different tetrahedral connectivity.
  {
    std::vector<Vector3<T>> vertices_copy = mesh->vertices();
    std::vector<VolumeElement> tetrahedra = mesh->tetrahedra();
    // clang-format off
    // Re-order vertices of the first tetrahedron.
    tetrahedra[0] = VolumeElement(tetrahedra[0].vertex(1),
                                  tetrahedra[0].vertex(2),
                                  tetrahedra[0].vertex(0),
                                  tetrahedra[0].vertex(3));
    // clang-format on
    VolumeMesh<T> mesh_different_tetrahedra(std::move(tetrahedra),
                                            std::move(vertices_copy));
    EXPECT_FALSE(mesh->Equal(mesh_different_tetrahedra));
  }

  // Different number of vertices.
  {
    std::vector<Vector3<T>> vertices = mesh->vertices();
    vertices.emplace_back(1.2, 3.7, 0.15);
    std::vector<VolumeElement> tetrahedra = mesh->tetrahedra();
    VolumeMesh<T> another_mesh(std::move(tetrahedra), std::move(vertices));
    EXPECT_FALSE(mesh->Equal(another_mesh));
  }

  // Different number of tetrahedra.
  {
    std::vector<Vector3<T>> vertices = mesh->vertices();
    std::vector<VolumeElement> tetrahedra = mesh->tetrahedra();
    tetrahedra.pop_back();
    VolumeMesh<T> another_mesh(std::move(tetrahedra), std::move(vertices));
    EXPECT_FALSE(mesh->Equal(another_mesh));
  }
}

GTEST_TEST(VolumeMeshTest, TestEqualDouble) {
  TestVolumeMeshEqual<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles.
GTEST_TEST(VolumeMeshTest, TestEqualAutoDiffXd) {
  TestVolumeMeshEqual<AutoDiffXd>();
}

template <typename T>
void TestCalcTetrahedronVolume() {
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh = TestVolumeMesh<T>(X_WM);
  // Estimate 4 multiply+add, each introduces 2 epsilons.
  const double kTolerance(8.0 * std::numeric_limits<double>::epsilon());

  const double expect_tetrahedron_volume(1. / 6.);
  for (int e = 0; e < 2; ++e) {
    const double tetrahedron_volume =
        ExtractDoubleOrThrow(volume_mesh->CalcTetrahedronVolume(e));
    EXPECT_NEAR(expect_tetrahedron_volume, tetrahedron_volume, kTolerance);
  }
}

GTEST_TEST(VolumeMeshTest, TestCalcTetrahedronVolumeDouble) {
  TestCalcTetrahedronVolume<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcTetrahedronVolumeAutoDiffXd) {
  TestCalcTetrahedronVolume<AutoDiffXd>();
}

template <typename T>
void TestCalcVolume() {
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh = TestVolumeMesh<T>(X_WM);
  // Estimate 4 multiply+add, each introduces 2 epsilons.
  const double kTolerance(8.0 * std::numeric_limits<double>::epsilon());

  const double expected_volume(1. / 3.);
  const double volume = ExtractDoubleOrThrow(volume_mesh->CalcVolume());
  EXPECT_NEAR(expected_volume, volume, kTolerance);
}

GTEST_TEST(VolumeMeshTest, TestCalcVolumeDouble) {
  TestCalcVolume<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcVolumeAutoDiffXd) {
  TestCalcVolume<AutoDiffXd>();
}

template <typename T>
void TestCalcBarycentric() {
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh = TestVolumeMesh<T>(X_WM);
  // Empirically the std::numeric_limits<double>::epsilon() 2.2e-16 is too
  // small to account for the pose.
  const T kTolerance(1e-14);
  const int element{0};

  using Barycentric = typename VolumeMesh<T>::template Barycentric<T>;
  // At the centroid of the tetrahedral element v0v1v2v3.
  {
    Vector3<T> p_M(0.25, 0.25, 0.25);
    const Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    const Barycentric expect_barycentric(0.25, 0.25, 0.25, 0.25);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the centroid of the face v0v1v2.
  {
    Vector3<T> p_M(1. / 3., 1. / 3., 0.);
    const Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    const Barycentric expect_barycentric(1. / 3., 1. / 3., 1. / 3., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the middle of the edge v0v1.
  {
    Vector3<T> p_M(0.5, 0., 0.);
    const Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    const Barycentric expect_barycentric(0.5, 0.5, 0., 0.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
  // At the vertex v3.
  {
    Vector3<T> p_M(0., 0., 1.);
    const Barycentric barycentric =
        volume_mesh->CalcBarycentric(X_WM * p_M, element);
    const Barycentric expect_barycentric(0., 0., 0., 1.);
    EXPECT_LE((barycentric - expect_barycentric).norm(), kTolerance);
  }
}

GTEST_TEST(VolumeMeshTest, TestCalcBarycentricDouble) {
  TestCalcBarycentric<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcBarycentricAutoDiffXd) {
  TestCalcBarycentric<AutoDiffXd>();
}

template <typename T>
void TestCalcGradBarycentric() {
  // The mesh M consists of one tetrahedral element whose vertices are at the
  // origin and on the coordinate axes of M's frame. The chosen vertex
  // coordinates avoid symmetry but are easy to calculate ∇bᵢ manually.
  //
  //                                 Z
  //                                 |
  //                                 v3
  //                                 |
  //                                 |
  //     v0_M = (0,0,0)              |
  //     v1_M = (1,0,0)              |   M's frame
  //     v2_M = (0,2,0)              |
  //     v3_M = (0,0,3)            v0+---+---v2-----Y
  //                                /
  //                              v1
  //                              /
  //                             /
  //                            X
  //
  const int tetrahedron[4] = {0, 1, 2, 3};
  const Vector3<T> v0_M(0., 0., 0.);
  const Vector3<T> v1_M(1., 0., 0.);
  const Vector3<T> v2_M(0., 2., 0.);
  const Vector3<T> v3_M(0., 0., 3.);

  // We use an arbitrary pose of the volume mesh M in World frame W to make the
  // test more realistic.
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 5.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));

  // Create the mesh with vertex coordinates expressed in World frame to make
  // the test more realistic.
  const VolumeMesh<T> mesh_W(
      {VolumeElement(tetrahedron)},
      {Vector3<T>(X_WM * v0_M), Vector3<T>(X_WM * v1_M),
       Vector3<T>(X_WM * v2_M), Vector3<T>(X_WM * v3_M)});

  const VolumeMeshTester<T> tester(mesh_W);
  const auto gradb0_W = tester.CalcGradBarycentric(0, 0);
  const auto gradb1_W = tester.CalcGradBarycentric(0, 1);
  const auto gradb2_W = tester.CalcGradBarycentric(0, 2);
  const auto gradb3_W = tester.CalcGradBarycentric(0, 3);

  // In this example, we have these equations, expressed in M's frame:
  //      b₀(x,y,z) = -x - y/2 - z/3 + 1,
  //      b₁(x,y,z) = x,
  //      b₂(x,y,z) = y/2,
  //      b₃(x,y,z) = z/3.
  // We can manually verify the equations by checking that bᵢ(vi) = 1 for all i,
  // bⱼ(vj) = 0 for all i ≠ j, and ∑bⱼ = 1.
  //     We can read off ∇bᵢ from the coefficients of x,y,z in the above
  // equations.
  const Vector3<T> expect_gradb0_M(-1., -1. / 2., -1. / 3.);
  const Vector3<T> expect_gradb1_M = Vector3<T>::UnitX();
  const Vector3<T> expect_gradb2_M = Vector3<T>::UnitY() / 2.;
  const Vector3<T> expect_gradb3_M = Vector3<T>::UnitZ() / 3.;

  const auto R_WM = X_WM.rotation();
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb0_M, gradb0_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb1_M, gradb1_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb2_M, gradb2_W, 1e-14));
  EXPECT_TRUE(CompareMatrices(R_WM * expect_gradb3_M, gradb3_W, 1e-14));

  // Since ∑bᵢ = 1, the gradients have zero sum: ∑(∇bᵢ) = 0.
  EXPECT_TRUE(CompareMatrices(gradb0_W + gradb1_W + gradb2_W + gradb3_W,
                              Vector3<T>::Zero(), 1e-14));
}

GTEST_TEST(VolumeMeshTest, TestCalcGradBarycentricDouble) {
  TestCalcGradBarycentric<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcGradBarycentricAutoDiffXd) {
  TestCalcGradBarycentric<AutoDiffXd>();
}

template <typename T>
void TestCalcGradientVectorOfLinearField() {
  // CalcGradientVectorOfLinearField() is a weighted sum of
  // CalcGradBarycentric() which is already tested with non-symmetric vertex
  // coordinates and an arbitrary pose. Therefore, it suffices to test
  // CalcGradientVectorOfLinearField() in the mesh's frame M with symmetric
  // vertex coordinates but non-symmetric field values.
  //
  // The four vertices v0,v1,v2,v3 of the tetrahedral element e0 have
  // coordinates expressed in M's frame as:
  //
  //                                +Z
  //                                 |
  //     v0_M = (0,0,0), f₀ = 2      v3
  //     v1_M = (1,0,0), f₁ = 3      |   M's frame
  //     v2_M = (0,1,0), f₂ = 4      |
  //     v3_M = (0,0,1), f₃ = 5    v0+------v2---+Y
  //                                /
  //                               /
  //                             v1
  //                             /
  //                           +X
  //
  auto mesh_M = TestVolumeMesh<T>();
  const std::array<T, 4> f{2., 3., 4., 5.};

  const Vector3<T> gradf_M = mesh_M->CalcGradientVectorOfLinearField(f, 0);

  // The field f on the tetrahedral element e0 satisfies this equation with
  // coordinates expressed in M's frame:
  //       f(x,y,z) = x + 2y + 3z + 2.
  // We can verify the equation manually by checking that f(vi_M) = fᵢ like
  // these:
  //       f(0,0,0) = 2
  //       f(1,0,0) = 3
  //       f(0,1,0) = 4
  //       f(0,0,1) = 5
  // Therefore, we can read off ∇f, expressed in M's frame, from the
  // coefficients of x,y,z in the equation.
  const Vector3<T> expect_gradf_M{1., 2., 3.};

  EXPECT_TRUE(CompareMatrices(expect_gradf_M, gradf_M, 1e-14));
}

GTEST_TEST(VolumeMeshTest, TestCalcGradientVectorOfLinearFieldDouble) {
  TestCalcGradientVectorOfLinearField<double>();
}

GTEST_TEST(VolumeMeshTest, TestCalcGradientVectorOfLinearFieldAutoDiffXd) {
  TestCalcGradientVectorOfLinearField<AutoDiffXd>();
}

template <typename T>
void TestTransformVertices() {
  const double tolerance = 4.0 * std::numeric_limits<T>::epsilon();
  const double scale = 1e-6;
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh_W = TestVolumeMesh<T>(RigidTransform<T>(), scale);
  volume_mesh_W->TransformVertices(X_WM);
  auto volume_mesh_W_expected = TestVolumeMesh<T>(X_WM, scale);
  EXPECT_TRUE(volume_mesh_W->Equal(*volume_mesh_W_expected, tolerance));

  // Check inward face normals and edge vectors. If the vertices have an
  // error of O(tolerance) edge vectors likewise have an error of
  // O(tolerance). Normals will have an error of order O(tolerance / L) where L
  // is a characteristic length of the triangle. We use an exaggerated scale so
  // that each triangular face has a characteristic length of 1e-6.
  for (int i = 0; i < volume_mesh_W->num_elements(); ++i) {
    // Test normals
    for (int j = 0; j < 4; ++j) {
      EXPECT_TRUE(CompareMatrices(volume_mesh_W->inward_normal(i, j),
                                  volume_mesh_W_expected->inward_normal(i, j),
                                  tolerance / scale,
                                  MatrixCompareType::relative));
    }

    // Test edges
    for (int j = 0; j < 4; ++j) {
      for (int k = j + 1; k < 4; ++k) {
        EXPECT_TRUE(
            CompareMatrices(volume_mesh_W->edge_vector(i, j, k),
                            volume_mesh_W_expected->edge_vector(i, j, k),
                            tolerance, MatrixCompareType::relative));
      }
    }
  }
}

GTEST_TEST(VolumeMeshTest, TestTransformVerticesDouble) {
  TestTransformVertices<double>();
}

GTEST_TEST(VolumeMeshTest, TestTransformVerticesAutoDiffXd) {
  TestTransformVertices<AutoDiffXd>();
}

template <typename T>
void TestSetAllPositions() {
  const RigidTransform<T> X_WM(
      RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
      Vector3<T>(1.0, 2.0, 3.0));
  auto volume_mesh_W = TestVolumeMesh<T>(RigidTransform<T>());
  auto volume_mesh_W_expected = TestVolumeMesh<T>(X_WM);

  volume_mesh_W->SetAllPositions(ToVectorX(volume_mesh_W_expected->vertices()));

  // Positions and derived quantities should match exactly.
  EXPECT_TRUE(volume_mesh_W->Equal(*volume_mesh_W_expected, 0));

  // Check inward face normals and edge vectors.
  for (int i = 0; i < volume_mesh_W->num_elements(); ++i) {
    // Test normals
    for (int j = 0; j < 4; ++j) {
      EXPECT_TRUE(CompareMatrices(volume_mesh_W->inward_normal(i, j),
                                  volume_mesh_W_expected->inward_normal(i, j),
                                  0));
    }

    // Test edges
    for (int j = 0; j < 4; ++j) {
      for (int k = j + 1; k < 4; ++k) {
        EXPECT_TRUE(
            CompareMatrices(volume_mesh_W->edge_vector(i, j, k),
                            volume_mesh_W_expected->edge_vector(i, j, k), 0));
      }
    }
  }
}

GTEST_TEST(VolumeMeshTest, TestSetAllPositionsDouble) {
  TestSetAllPositions<double>();
}

GTEST_TEST(VolumeMeshTest, TestSetAllPositionsAutoDiffXd) {
  TestSetAllPositions<AutoDiffXd>();
}

template <typename T>
void TestPositionDependentQuantities() {
  // Test a simple single tetrahedron mesh.
  {
    const VolumeMesh<T> mesh_M({VolumeElement(0, 1, 2, 3)},
                               {Vector3<T>::Zero(), Vector3<T>::UnitX(),
                                Vector3<T>::UnitY(), Vector3<T>::UnitZ()});
    // There is only one tetrahedral element.
    const int e0 = 0;
    EXPECT_EQ(mesh_M.inward_normal(e0, 0), -Vector3<T>::Ones().normalized());
    EXPECT_EQ(mesh_M.inward_normal(e0, 1), Vector3<T>::UnitX());
    EXPECT_EQ(mesh_M.inward_normal(e0, 2), Vector3<T>::UnitY());
    EXPECT_EQ(mesh_M.inward_normal(e0, 3), Vector3<T>::UnitZ());

    EXPECT_EQ(mesh_M.edge_vector(e0, 0, 1), Vector3<T>::UnitX());
    EXPECT_EQ(mesh_M.edge_vector(e0, 0, 2), Vector3<T>::UnitY());
    EXPECT_EQ(mesh_M.edge_vector(e0, 0, 3), Vector3<T>::UnitZ());
    EXPECT_EQ(mesh_M.edge_vector(e0, 1, 2), Vector3<T>(-1, 1, 0));
    EXPECT_EQ(mesh_M.edge_vector(e0, 1, 3), Vector3<T>(-1, 0, 1));
    EXPECT_EQ(mesh_M.edge_vector(e0, 2, 3), Vector3<T>(0, -1, 1));
  }

  // Test a more complicated mesh with an arbitrary transformation.
  {
    const RigidTransform<T> X_WM(
        RollPitchYaw<T>(M_PI / 6.0, 2.0 * M_PI / 3.0, 7.0 * M_PI / 4.0),
        Vector3<T>(1.0, 2.0, 3.0));
    auto volume_mesh_W = TestVolumeMesh<T>(X_WM);

    // Edge (i, j), regardless of the lexicographical order of i and j.
    auto signed_edge = [&volume_mesh_W](int e, int i, int j) -> Vector3<T> {
      if (i > j) return -volume_mesh_W->edge_vector(e, j, i);
      return volume_mesh_W->edge_vector(e, i, j);
    };

    // Test algebraic identities on computed quantities.
    for (int i = 0; i < volume_mesh_W->num_elements(); ++i) {
      const VolumeElement& e = volume_mesh_W->element(i);
      for (int j = 0; j < 4; ++j) {
        // Local indices of neighbor vertices.
        const int bj = (j + 1) % 4;
        const int cj = (j + 2) % 4;
        const int dj = (j + 3) % 4;

        const Vector3<T>& a = volume_mesh_W->vertex(e.vertex(j));
        const Vector3<T>& b = volume_mesh_W->vertex(e.vertex(bj));
        const Vector3<T>& c = volume_mesh_W->vertex(e.vertex(cj));
        const Vector3<T>& d = volume_mesh_W->vertex(e.vertex(dj));

        // Inward normal of face (b, c, d) pointing towards a.
        const Vector3<T>& n = volume_mesh_W->inward_normal(i, j);

        // Edges from a to {b, c, d} point into the face.
        EXPECT_LT(n.dot(b - a), 0);
        EXPECT_LT(n.dot(c - a), 0);
        EXPECT_LT(n.dot(d - a), 0);

        // Edges of the face are perpendicular to the face.
        EXPECT_NEAR(ExtractDoubleOrThrow(n.dot(b - c)), 0,
                    std::numeric_limits<double>::epsilon());
        EXPECT_NEAR(ExtractDoubleOrThrow(n.dot(b - d)), 0,
                    std::numeric_limits<double>::epsilon());
        EXPECT_NEAR(ExtractDoubleOrThrow(n.dot(c - d)), 0,
                    std::numeric_limits<double>::epsilon());

        // Edge arithmetic (e.g. a + (a, b) ≅ b)
        EXPECT_TRUE(CompareMatrices(b, a + signed_edge(i, j, bj),
                                    std::numeric_limits<double>::epsilon(),
                                    MatrixCompareType::relative));
        EXPECT_TRUE(CompareMatrices(c, a + signed_edge(i, j, cj),
                                    std::numeric_limits<double>::epsilon(),
                                    MatrixCompareType::relative));
        EXPECT_TRUE(CompareMatrices(d, a + signed_edge(i, j, dj),
                                    std::numeric_limits<double>::epsilon(),
                                    MatrixCompareType::relative));
      }
    }
  }
}

GTEST_TEST(VolumeMeshTest, TestPositionDependentQuantitiesDouble) {
  TestPositionDependentQuantities<double>();
}

GTEST_TEST(VolumeMeshTest, TestPositionDependentQuantitiesAutoDiffXd) {
  TestPositionDependentQuantities<AutoDiffXd>();
}

template <typename T>
std::unique_ptr<VolumeMeshFieldLinear<T, T>> TestVolumeMeshFieldLinear() {
  auto volume_mesh = TestVolumeMesh<T>();

  // We give names to the values at vertices for testing later.
  const T f0{1.};
  const T f1{2.};
  const T f2{3.};
  const T f3{4.};
  const T f4{5.};
  std::vector<T> f_values = {f0, f1, f2, f3, f4};

  auto volume_mesh_field = std::make_unique<VolumeMeshFieldLinear<T, T>>(
      std::move(f_values), volume_mesh.get());

  // Tests evaluation of the field on the element e0 {v0, v1, v2, v3}.
  const int e0{0};
  const typename VolumeMesh<T>::template Barycentric<T> b{0.4, 0.3, 0.2, 0.1};
  const T expect_p = b(0) * f0 + b(1) * f1 + b(2) * f2 + b(3) * f3;
  EXPECT_EQ(expect_p, volume_mesh_field->Evaluate(e0, b));

  // Tests the gradient vector ∇p on the element e0 {v0, v1, v2, v3}.
  // VolumeMeshFieldLinear stores ∇p for each element. We only check that
  // ∇p stored in VolumeMeshFieldLinear is the same as the gradient
  // calculated by VolumeMesh. The vectors are expressed in frame M of the mesh.
  Vector3<T> gradp_M = volume_mesh_field->EvaluateGradient(e0);
  Vector3<T> expect_gradp_M = volume_mesh->CalcGradientVectorOfLinearField(
      std::array<T, 4>{f0, f1, f2, f3}, e0);
  EXPECT_TRUE(CompareMatrices(expect_gradp_M, gradp_M, 1e-14));

  return volume_mesh_field;
}

// Test instantiation of VolumeMeshFieldLinear using `double` as the underlying
// scalar type.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldLinearDouble) {
  auto volume_mesh_field = TestVolumeMeshFieldLinear<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There are no tests of
// differentiation.
GTEST_TEST(VolumeMeshFieldTest, TestVolumeMeshFieldLinearAutoDiffXd) {
  auto volume_mesh_field = TestVolumeMeshFieldLinear<AutoDiffXd>();
}

/* A double-valued mesh can produce AutoDiffXd-valued results for
 CalcBarycentric() and CalcGradientVectorOfLinearField() based on the scalar
 type of the query point. This confirms that mixing behavior. The tests work by
 instantiating meshes on both scalar types and various query parameters on
 both types. Then we mix and match mesh and parameter scalars and verify the
 outputs. By "verify", we don't worry too much about the correctness of the
 values and derivatives; we're just checking that, mechanically speaking,
 derivatives are propagating as expected. We make sure that AutoDiffXd-valued
 parameters have at least *one* value with non-zero derivatives. */
class ScalarMixingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    mesh_d_ = TestVolumeMesh<double>();

    // We construct an AutoDiffXd-valued mesh from the double-valued mesh. We
    // only set the derivatives for vertex 3. That means, operations on
    // test 0 *must* have derivatives, but tet 1 may not have them.
    std::vector<Vector3<AutoDiffXd>> vertices;
    vertices.emplace_back(mesh_d_->vertex(0));
    vertices.emplace_back(mesh_d_->vertex(1));
    vertices.emplace_back(mesh_d_->vertex(2));
    vertices.emplace_back(math::InitializeAutoDiff(mesh_d_->vertex(3)));
    vertices.emplace_back(mesh_d_->vertex(4));
    std::vector<VolumeElement> elements(mesh_d_->tetrahedra());

    mesh_ad_ = std::make_unique<VolumeMesh<AutoDiffXd>>(std::move(elements),
                                                        std::move(vertices));
  }

  std::unique_ptr<VolumeMesh<double>> mesh_d_;
  std::unique_ptr<VolumeMesh<AutoDiffXd>> mesh_ad_;

  int e0_{0};
  int e1_{1};
};

TEST_F(ScalarMixingTest, CalcBarycentric) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  Vector3<double> p_WQ_d = Vector3<double>::Zero();
  for (int v = 0; v < 4; ++v) {
    p_WQ_d += mesh_d_->vertex(v);
  }
  p_WQ_d /= 4;
  const Vector3<AutoDiffXd> p_WQ_ad = math::InitializeAutoDiff(p_WQ_d);

  const Vector4<double> b_expected = Vector4<double>{1, 1, 1, 1} / 4;

  {
    // Double-valued mesh and double-valued point: double-valued result.
    const Vector4<double> b = mesh_d_->CalcBarycentric(p_WQ_d, e0_);
    EXPECT_TRUE(CompareMatrices(b, b_expected, kEps));
  }

  {
    // Double-valued mesh with AutoDiffXd-valued point: AutodDiffXd-valued
    // result.
    const Vector4<AutoDiffXd>& b = mesh_d_->CalcBarycentric(p_WQ_ad, e0_);
    EXPECT_EQ(b(0).derivatives().size(), 3);
    EXPECT_EQ(b(1).derivatives().size(), 3);
    EXPECT_EQ(b(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(b), b_expected, kEps));
  }

  {
    // AutoDiffXd-valued mesh with double-valued point on triangle *with*
    // derivatives: AutodDiffXd-valued result *with* derivatives.
    const Vector4<AutoDiffXd>& b1 = mesh_ad_->CalcBarycentric(p_WQ_d, e0_);
    EXPECT_EQ(b1(0).derivatives().size(), 3);
    EXPECT_EQ(b1(1).derivatives().size(), 3);
    EXPECT_EQ(b1(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(b1), b_expected, kEps));

    // AutoDiffXd-valued mesh with double-valued point on triangle *without*
    // derivatives: AutodDiffXd-valued result *without* derivatives.
    const Vector4<AutoDiffXd>& b2 = mesh_ad_->CalcBarycentric(p_WQ_d, e1_);
    EXPECT_EQ(b2(0).derivatives().size(), 0);
    EXPECT_EQ(b2(1).derivatives().size(), 0);
    EXPECT_EQ(b2(2).derivatives().size(), 0);
    // We'll assume the *value* on this one is correct.
  }

  {
    // AutoDiffXd-valued mesh with AutoDiffXd-valued point on triangle:
    // AutodDiffXd-valued.
    const Vector4<AutoDiffXd>& b = mesh_ad_->CalcBarycentric(p_WQ_ad, e0_);
    EXPECT_EQ(b(0).derivatives().size(), 3);
    EXPECT_EQ(b(1).derivatives().size(), 3);
    EXPECT_EQ(b(2).derivatives().size(), 3);
    EXPECT_TRUE(CompareMatrices(math::ExtractValue(b), b_expected, kEps));
  }
}

TEST_F(ScalarMixingTest, CalcGradientVectorOfLinearField) {
  constexpr double kEps = std::numeric_limits<double>::epsilon();
  const std::array<double, 4> field_d{0, 0, 0, 1};
  AutoDiffXd f0{0};
  // Because one vertex of the AutoDiffXd-valued mesh has 3 derivatives, this
  // must likewise have three derivatives to be compatible.
  f0.derivatives().resize(3);
  f0.derivatives() << 1, 2, 3;
  const std::array<AutoDiffXd, 4> field_ad = {f0, 0, 0, 1};
  const Vector3<double> grad_W_expected{0, 0, 1};

  {
    // Double-valued mesh and double-valued field: double-valued result.
    const Vector3<double> grad_W =
        mesh_d_->CalcGradientVectorOfLinearField(field_d, e0_);
    EXPECT_TRUE(CompareMatrices(grad_W, grad_W_expected, kEps));
  }

  {
    // Double-valued mesh with AutoDiffXd-valued field: AutodDiffXd-valued
    // result.
    const Vector3<AutoDiffXd> grad_W =
        mesh_d_->CalcGradientVectorOfLinearField(field_ad, e0_);
    EXPECT_EQ(grad_W(0).derivatives().size(), 3);
    EXPECT_EQ(grad_W(1).derivatives().size(), 3);
    EXPECT_EQ(grad_W(2).derivatives().size(), 3);
    EXPECT_TRUE(
        CompareMatrices(math::ExtractValue(grad_W), grad_W_expected, kEps));
  }

  {
    // AutoDiffXd-valued mesh with double-valued field on triangle *with*
    // derivatives: AutodDiffXd-valued result *with* derivatives.
    const Vector3<AutoDiffXd> grad_W1 =
        mesh_ad_->CalcGradientVectorOfLinearField(field_d, e0_);
    EXPECT_EQ(grad_W1(0).derivatives().size(), 3);
    EXPECT_EQ(grad_W1(1).derivatives().size(), 3);
    EXPECT_EQ(grad_W1(2).derivatives().size(), 3);
    EXPECT_TRUE(
        CompareMatrices(math::ExtractValue(grad_W1), grad_W_expected, kEps));

    // AutoDiffXd-valued mesh with double-valued field on triangle *without*
    // derivatives: AutodDiffXd-valued result *without* derivatives.
    const Vector3<AutoDiffXd> grad_W2 =
        mesh_ad_->CalcGradientVectorOfLinearField(field_d, e1_);
    EXPECT_EQ(grad_W2(0).derivatives().size(), 0);
    EXPECT_EQ(grad_W2(1).derivatives().size(), 0);
    EXPECT_EQ(grad_W2(2).derivatives().size(), 0);
    // We'll assume the *value* on this one is correct.
  }

  {
    // AutoDiffXd-valued mesh with AutoDiffXd-valued field on triangle:
    // AutodDiffXd-valued.
    const Vector3<AutoDiffXd> grad_W =
        mesh_ad_->CalcGradientVectorOfLinearField(field_ad, e0_);
    EXPECT_EQ(grad_W(0).derivatives().size(), 3);
    EXPECT_EQ(grad_W(1).derivatives().size(), 3);
    EXPECT_EQ(grad_W(2).derivatives().size(), 3);
    EXPECT_TRUE(
        CompareMatrices(math::ExtractValue(grad_W), grad_W_expected, kEps));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
