#include "drake/geometry/proximity/mesh_deformer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_box_mesh.h"

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

    const VectorX<T> too_small(q_size- 1);
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

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake


