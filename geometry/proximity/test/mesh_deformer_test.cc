#include "drake/geometry/proximity/mesh_deformer.h"

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

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;

/* The mesh deformer has two independent axes to test: the type of mesh (volume
 vs surface) and the scalar type for the mesh (double or AutoDiffXd). This test
 provides the basis for testing the constructor and the vertex position
 updating for all four combinations.

 See below where we instantiate this test fixture explicitly on each type of
 mesh and then use gtest to iterate over the scalar types. */
template <typename T, template <typename> typename MeshType>
class MeshDeformerTest : public ::testing::Test {
 public:
  MeshDeformerTest()
      : ::testing::Test(),
        mesh_(BoxMaker<T, MeshType>().make(Box(1, 2, 3))),
        deformer_(&mesh_) {
    // Create some nonsensical vertex positions -- these monotonically
    // increasing values will *not* match the original box.
    q_.resize(mesh_.num_vertices() * 3);
    for (int i = 0; i < q_.size(); ++i) {
      q_[i] = i;
    }
  }

  /* Tests the constructor. */
  void TestConstructor() {
    // The test class constructs it, so, we'll simply confirm that the deformer
    // has the mesh.
    EXPECT_EQ(&deformer_.mesh(), &mesh_);
  }

  void TestSetAllPositions() {
    // Quick reality check that we don't start in a deformed configuration.
    ASSERT_FALSE(MatchesQ());
    deformer_.SetAllPositions(q_);
    EXPECT_TRUE(MatchesQ());

    const int q_size = q_.rows();

    const VectorX<T> too_small(q_size - 1);
    DRAKE_EXPECT_THROWS_MESSAGE(
        deformer_.SetAllPositions(too_small),
        ".+SetAllPositions.+ \\d+ vertices with data for \\d+ vertices");

    const VectorX<T> too_large(q_size + 1);
    DRAKE_EXPECT_THROWS_MESSAGE(
        deformer_.SetAllPositions(too_large),
        ".+SetAllPositions.+ \\d+ vertices with data for \\d+ vertices");
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
  MeshDeformer<MeshType<T>> deformer_;
  VectorX<T> q_;
};

template <typename T>
using VolumeMeshDeformerTest = MeshDeformerTest<T, VolumeMesh>;
TYPED_TEST_SUITE(VolumeMeshDeformerTest, ScalarTypes);

TYPED_TEST(VolumeMeshDeformerTest, Construction) {
  this->TestConstructor();
}

TYPED_TEST(VolumeMeshDeformerTest, SetAllPositions) {
  this->TestSetAllPositions();
}

template <typename T>
using SurfaceMeshDeformerTest = MeshDeformerTest<T, TriangleSurfaceMesh>;
TYPED_TEST_SUITE(SurfaceMeshDeformerTest, ScalarTypes);

TYPED_TEST(SurfaceMeshDeformerTest, Construction) {
  this->TestConstructor();
}

TYPED_TEST(SurfaceMeshDeformerTest, SetAllPositions) {
  this->TestSetAllPositions();
}

// N.B. We can't use partial specialization on functions, so we create a factory
// struct to serve the purpose.
template <typename T, template <typename> typename MeshType>
struct SingleTriangleMaker {
  static MeshType<T> make() { throw std::logic_error("Unimplemented call"); }
};

// Makes a TriangleSurfaceMesh with a single triangle whose normal is _not_
// along the z-axis.
template <typename T>
struct SingleTriangleMaker<T, TriangleSurfaceMesh> {
  static TriangleSurfaceMesh<T> make() {
    std::vector<Vector3<T>> vertices;
    vertices.emplace_back(0, 0, 0);
    vertices.emplace_back(0, 2, 0);
    vertices.emplace_back(0, 0, 2);
    std::vector<SurfaceTriangle> triangles;
    triangles.emplace_back(0, 1, 2);
    return TriangleSurfaceMesh(std::move(triangles), std::move(vertices));
  }
};

// Makes a PolygonSurfaceMesh with a single triangle whose normal is _not_ along
// the z-axis.
template <typename T>
struct SingleTriangleMaker<T, PolygonSurfaceMesh> {
  static PolygonSurfaceMesh<T> make() {
    std::vector<int> face_data{3, 0, 1, 2};
    std::vector<Vector3<T>> vertices;
    vertices.emplace_back(0, 0, 0);
    vertices.emplace_back(0, 2, 0);
    vertices.emplace_back(0, 0, 2);
    return PolygonSurfaceMesh(std::move(face_data), std::move(vertices));
  }
};

// Tests that for surface meshes, auxiliary quantities (face normal, face area,
// and mesh centroid) are updated along with vertex position changes.
template <typename T, template <typename> typename MeshType>
class MeshDeformerDataTest : public ::testing::Test {
 public:
  MeshDeformerDataTest()
      : ::testing::Test(),
        mesh_(SingleTriangleMaker<T, MeshType>().make()),
        deformer_(&mesh_) {
    // Create vertex positions that all lie in the xy plane.
    DRAKE_DEMAND(mesh_.num_vertices() == 3);
    q_.resize(mesh_.num_vertices() * 3);
    q_.template segment<3>(0) = Vector3<T>::Zero();
    q_.template segment<3>(3) = Vector3<T>::UnitX();
    q_.template segment<3>(6) = Vector3<T>::UnitY();
  }

  void TestData() {
    // Quick reality check that we don't start with the expected data.
    EXPECT_FALSE(CompareMatrices(mesh_.face_normal(0), Vector3<T>::UnitZ()));
    EXPECT_NE(mesh_.area(0), 0.5);
    EXPECT_NE(mesh_.total_area(), 0.5);
    const Vector3<T> expected_centroid(1.0 / 3.0, 1.0 / 3.0, 0.0);
    EXPECT_FALSE(CompareMatrices(mesh_.element_centroid(0), expected_centroid));
    EXPECT_FALSE(CompareMatrices(mesh_.centroid(), expected_centroid));

    deformer_.SetAllPositions(q_);

    EXPECT_TRUE(CompareMatrices(mesh_.face_normal(0), Vector3<T>::UnitZ()));
    EXPECT_EQ(mesh_.area(0), 0.5);
    EXPECT_EQ(mesh_.total_area(), 0.5);
    EXPECT_TRUE(CompareMatrices(mesh_.element_centroid(0), expected_centroid));
    EXPECT_TRUE(CompareMatrices(mesh_.centroid(), expected_centroid));
  }

 protected:
  MeshType<T> mesh_;
  MeshDeformer<MeshType<T>> deformer_;
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
