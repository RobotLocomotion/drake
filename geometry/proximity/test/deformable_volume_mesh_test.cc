#include "drake/geometry/proximity/deformable_volume_mesh.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

template <typename T>
class DeformableVolumeMeshTest : public ::testing::Test {
 public:
  DeformableVolumeMeshTest() : ::testing::Test(), mesh_(MakeBox()) {}

  /* Reports if the mesh coordinates match the arbitrary q-values to which we
   will deform the mesh. We're *assuming* that q is correctly sized (i.e.,
   q.size() == 3 * V, where V is the number of vertices). */
  ::testing::AssertionResult MatchesQ(const VectorX<T>& q) const {
    const VectorX<T> values = Eigen::Map<const VectorX<T>>(
        reinterpret_cast<const T*>(mesh_.mesh().vertices().data()), q.size());
    return CompareMatrices(values, q);
  }

  /* The box specification we're meshing. */
  static Box box(double scale = 1) {
    return Box(1 * scale, 2 * scale, 3 * scale);
  }

  /* Creates an instance of the box we'll use for the test. */
  static VolumeMesh<T> MakeBox(double scale = 1) {
    return MakeBoxVolumeMeshWithMa<T>(box(scale));
  }

 protected:
  DeformableVolumeMesh<T> mesh_;
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(DeformableVolumeMeshTest, ScalarTypes);

/* Generically poke around the mesh to make sure things are wired up correctly.
   - the mesh is a copy of the input mesh. */
TYPED_TEST(DeformableVolumeMeshTest, Construction) {
  /* The test class constructs the deformable mesh; invocation is implicit in
   test fixture. We'll simply confirm the listed properties. */

  /* The contained mesh is a copy of the expected mesh. */
  EXPECT_TRUE(this->mesh_.mesh().Equal(this->MakeBox()));
}

TYPED_TEST(DeformableVolumeMeshTest, CopyConstructor) {
  using T = TypeParam;

  DeformableVolumeMesh<T> copy(this->mesh_);

  /* The copy's members are different *instances*, but equal values to the
   source. */
  EXPECT_NE(&copy.mesh(), &this->mesh_.mesh());
  EXPECT_TRUE(copy.mesh().Equal(this->mesh_.mesh()));
}

TYPED_TEST(DeformableVolumeMeshTest, MoveConstructor) {
  using T = TypeParam;
  /* Construct a reference copy -- assumes the copy constructor works (see the
   CopyConstructor test). */
  const DeformableVolumeMesh<T> ref(this->mesh_);

  DeformableVolumeMesh<T> moved(std::move(this->mesh_));

  /* The targets's members are different *instances*, but equal values to the
   reference. */
  EXPECT_NE(&moved.mesh(), &this->mesh_.mesh());
  EXPECT_TRUE(moved.mesh().Equal(ref.mesh()));
}

TYPED_TEST(DeformableVolumeMeshTest, CopyAssignment) {
  using T = TypeParam;
  DeformableVolumeMesh<T> big(this->MakeBox(2.0));
  const DeformableVolumeMesh<T>& small = this->mesh_;

  EXPECT_FALSE(big.mesh().Equal(small.mesh()));

  big = small;

  /* The copy's members are different *instances*, but equal values to the
   source. */
  EXPECT_NE(&big.mesh(), &small.mesh());
  EXPECT_TRUE(big.mesh().Equal(small.mesh()));
}

TYPED_TEST(DeformableVolumeMeshTest, MoveAssignment) {
  using T = TypeParam;
  /* With move *assignment*, we need a deformable mesh with an initial value
   such that we can recognize successful reassignment. So, we create a mesh
   that is "big" (compared to the "small" source mesh) and then move the small
   mesh onto the big mesh. */
  DeformableVolumeMesh<T> big(this->MakeBox(2.0));

  /* Construct a reference copy -- assumes the copy constructor works (see the
   CopyConstructor test). */
  const DeformableVolumeMesh<T> small_ref(this->mesh_);

  /* An alias for the "small" source deformable mesh. */
  DeformableVolumeMesh<T>& small = this->mesh_;

  EXPECT_FALSE(big.mesh().Equal(small.mesh()));

  big = std::move(small);

  /* The targets's members are different *instances*, but equal values to the
   reference. */
  EXPECT_NE(&big.mesh(), &small.mesh());
  EXPECT_TRUE(big.mesh().Equal(small_ref.mesh()));
}

/* Given the mesh, pass in new vertex positions.
   - Confirm vertices have moved. */
TYPED_TEST(DeformableVolumeMeshTest, Update) {
  using T = TypeParam;
  /* Create a set of coordinates where all of the vertices are moved toward the
   origin by scaling. */
  const double kScale = 0.33333;
  const int num_vertices = this->mesh_.mesh().num_vertices();
  const auto& vertices = this->mesh_.mesh().vertices();
  VectorX<T> q(num_vertices * 3);
  for (int v = 0, i = 0; v < num_vertices; ++v, i += 3) {
    q.segment(i, 3) << vertices[v].r_MV() * kScale;
  }

  /* Make sure we don't start in a pre-deformed state. */
  ASSERT_FALSE(this->MatchesQ(q));
  this->mesh_.UpdateVertexPositions(q);
  ASSERT_TRUE(this->MatchesQ(q));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
