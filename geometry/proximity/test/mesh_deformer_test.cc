/* @file
 Test SetAllPositions() functions for VolumeMesh, TriangleSurfaceMesh, and
 PolygonSurfaceMesh. */

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

// N.B. We can't use partial specialization on functions, so we create a factory
// struct to serve the purpose.
template <typename T, template <typename> typename MeshType>
struct BoxMaker {
  static MeshType<T> make(const Box& box) {
    throw std::logic_error("Unimplemented call to MakeBox");
  }
};

template <typename T>
struct BoxMaker<T, VolumeMesh> {
  static VolumeMesh<T> make(const Box& box) {
    return MakeBoxVolumeMeshWithMa<T>(box);
  }
};

template <typename T>
struct BoxMaker<T, TriangleSurfaceMesh> {
  static TriangleSurfaceMesh<T> make(const Box& box) {
    return ConvertVolumeToSurfaceMesh(MakeBoxVolumeMeshWithMa<T>(box));
  }
};

template <typename T>
struct BoxMaker<T, PolygonSurfaceMesh> {
  static TriangleSurfaceMesh<T> make(const Box& box) {
    // Volume mesh to tri mesh.
    TriangleSurfaceMesh<T> tri_mesh =
        ConvertVolumeToSurfaceMesh(MakeBoxVolumeMeshWithMa<T>(box));
    // Tri mesh to polygon mesh.
    std::vector<int> face_data;
    for (int f = 0; f < tri_mesh.num_triangles(); ++f) {
      face_data.emplace_back(3);
      for (int i = 0; i < 3; ++i) {
        face_data.emplace_back(tri_mesh.triangles()[f].vertex(i));
      }
    }
    return PolygonSurfaceMesh<T>(std::move(face_data), tri_mesh.vertices());
  }
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;

/* There are two independent axes to test: the type of mesh (volume vs triangle
 vs polygon) and the scalar type for the mesh (double or AutoDiffXd). This test
 provides the basis for testing the vertex position updating
 for all six combinations.

 See below where we instantiate this test fixture explicitly on each type of
 mesh and then use gtest to iterate over the scalar types. */
template <typename T, template <typename> typename MeshType>
class MeshDeformerTest : public ::testing::Test {
 public:
  MeshDeformerTest()
      : ::testing::Test(), mesh_(BoxMaker<T, MeshType>().make(Box(1, 2, 3))) {
    // Create some nonsensical vertex positions -- these monotonically
    // increasing values will *not* match the original box.
    q_.resize(mesh_.num_vertices() * 3);
    for (int i = 0; i < q_.size(); ++i) {
      q_[i] = i;
    }
  }

  void TestSetAllPositions() {
    // Quick reality check that we don't start in a deformed configuration.
    ASSERT_FALSE(MatchesQ());
    mesh_.SetAllPositions(q_);
    EXPECT_TRUE(MatchesQ());

    const int q_size = q_.rows();

    const VectorX<T> too_small(q_size - 1);
    DRAKE_EXPECT_THROWS_MESSAGE(
        mesh_.SetAllPositions(too_small),
        ".*SetAllPositions.+ \\d+ vertices with data for \\d+ DoFs");

    const VectorX<T> too_large(q_size + 1);
    DRAKE_EXPECT_THROWS_MESSAGE(
        mesh_.SetAllPositions(too_large),
        ".*SetAllPositions.+ \\d+ vertices with data for \\d+ DoFs");
  }

  /* Reports if the mesh coordinates match the arbitrary q-values to which we
   will deform the mesh. */
  ::testing::AssertionResult MatchesQ() const {
    const VectorX<T> values = Eigen::Map<const VectorX<T>>(
        reinterpret_cast<const T*>(mesh_.vertices().data()), q_.size());
    return CompareMatrices(values, q_);
  }

 protected:
  MeshType<T> mesh_;
  VectorX<T> q_;
};

template <typename T>
using VolumeMeshDeformerTest = MeshDeformerTest<T, VolumeMesh>;
TYPED_TEST_SUITE(VolumeMeshDeformerTest, ScalarTypes);

TYPED_TEST(VolumeMeshDeformerTest, SetAllPositions) {
  this->TestSetAllPositions();
}

template <typename T>
using TriangleSurfaceMeshDeformerTest =
    MeshDeformerTest<T, TriangleSurfaceMesh>;
TYPED_TEST_SUITE(TriangleSurfaceMeshDeformerTest, ScalarTypes);

TYPED_TEST(TriangleSurfaceMeshDeformerTest, SetAllPositions) {
  this->TestSetAllPositions();
}

template <typename T>
using PolygonSurfaceMeshDeformerTest = MeshDeformerTest<T, TriangleSurfaceMesh>;
TYPED_TEST_SUITE(PolygonSurfaceMeshDeformerTest, ScalarTypes);

TYPED_TEST(PolygonSurfaceMeshDeformerTest, SetAllPositions) {
  this->TestSetAllPositions();
}

// Makes a mesh consisting of a single triangle whose area is *not* 0.5.
template <typename MeshType>
MeshType MakeSingleTriangle() {
  using T = typename MeshType::ScalarType;
  std::vector<Vector3<T>> vertices;
  vertices.emplace_back(0, 0, 0);
  vertices.emplace_back(0, 2, 0);
  vertices.emplace_back(0, 0, 2);
  // The mesh types only differ in how they describe faces.
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    std::vector<SurfaceTriangle> triangles;
    triangles.emplace_back(0, 1, 2);
    return TriangleSurfaceMesh(std::move(triangles), std::move(vertices));
  } else if constexpr (std::is_same_v<MeshType, PolygonSurfaceMesh<T>>) {
    std::vector<int> face_data{3, 0, 1, 2};
    return PolygonSurfaceMesh(std::move(face_data), std::move(vertices));
  } else {
    throw std::logic_error("Unimplemented");
  }
}

// Tests that for surface meshes, auxiliary quantities (face normal, face area,
// and mesh centroid) are updated along with vertex position changes.
template <typename T, template <typename> typename MeshType>
class MeshDeformerDataTest : public ::testing::Test {
 public:
  MeshDeformerDataTest()
      : ::testing::Test(), mesh_(MakeSingleTriangle<MeshType<T>>()) {
    // Create vertex positions that all lie in the xy plane.
    DRAKE_DEMAND(mesh_.num_vertices() == 3);
    q_.resize(mesh_.num_vertices() * 3);
    q_.template segment<3>(0) = Vector3<T>::Zero();
    q_.template segment<3>(3) = Vector3<T>::UnitX();
    q_.template segment<3>(6) = Vector3<T>::UnitY();
  }

  void TestData() {
    // Quick reality check that we don't start with the expected data.
    EXPECT_NE(mesh_.total_area(), 0.5);

    mesh_.SetAllPositions(q_);

    // The deformer's only responsibility is to tell the mesh to update itself
    // based on the fact that vertex positions have changed. Therefore, we only
    // need evidence that it got invoked. Simply checking one dependent quantity
    // would be enough
    EXPECT_EQ(mesh_.total_area(), 0.5);
  }

 protected:
  MeshType<T> mesh_;
  VectorX<T> q_;
};

template <typename T>
using TriangleMeshDataTest = MeshDeformerDataTest<T, TriangleSurfaceMesh>;
TYPED_TEST_SUITE(TriangleMeshDataTest, ScalarTypes);
TYPED_TEST(TriangleMeshDataTest, TestData) {
  this->TestData();
}

template <typename T>
using PolygonMeshDataTest = MeshDeformerDataTest<T, PolygonSurfaceMesh>;
TYPED_TEST_SUITE(PolygonMeshDataTest, ScalarTypes);
TYPED_TEST(PolygonMeshDataTest, TestData) {
  this->TestData();
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
