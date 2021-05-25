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

/* In testing the move/copy semantics:

1. For copy semantics, we want to confirm that the deformation mechanism is
   wired up correctly. So, after copyping, we'll confirm the meshes match, but
   when we update vertex positions for *one* of the meshes, we'll confirm they
   no longer match.
2. For move semantics, knowing something about move semantics on meshes we'll
   confirm that the target of the move semantics doesn't match the source
   mesh.
For both, we'll make sure moving the vertices matches a reference mesh with
the same vertex positions.

For the assignment operators, we need to have *constructed* meshes that change
values. So, we'll construct two meshes with identical topology but different
scale (big and small), assigning the small to the big so that we can detect the
efficacy of the assignment.

This implicitly tests the UpdateVertexPositions() method over and over again; so
there is no dedicated test for that method. */
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

  /* Given a mesh with N vertices, this returns a vector of 3N values which
   represents a scaling of the vertex positions. */
  static VectorX<T> ScaleVertexPositions(const VolumeMesh<T>& mesh,
                                         double scale) {
    const int num_vertices = mesh.num_vertices();
    const auto& vertices = mesh.vertices();
    VectorX<T> q(num_vertices * 3);
    for (int v = 0, i = 0; v < num_vertices; ++v, i += 3) {
      q.segment(i, 3) << vertices[v].r_MV() * scale;
    }
    return q;
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

  /* Moving copy's vertex positions leaves mesh_'s vertex positions in place. */
  constexpr double kScale = 0.3333;
  const VectorX<T> q = this->ScaleVertexPositions(copy.mesh(), kScale);
  copy.UpdateVertexPositions(q);
  EXPECT_FALSE(copy.mesh().Equal(this->mesh_.mesh()));
  /* However, if we construct a box with the same scaled factor, those meshes
   will be equal. */
  const VolumeMesh<T> ref = this->MakeBox(kScale);
  EXPECT_TRUE(copy.mesh().Equal(ref, std::numeric_limits<double>::epsilon()));
}

TYPED_TEST(DeformableVolumeMeshTest, MoveConstructor) {
  using T = TypeParam;
  /* Construct a reference copy -- assumes the copy constructor works (see the
   CopyConstructor test). */
  const DeformableVolumeMesh<T> ref(this->mesh_);

  DeformableVolumeMesh<T> moved(std::move(this->mesh_));

  /* The targets's members are different *instances*, but equal values to the
   reference. The source mesh is left in a state such that the moved mesh is
   no longer equal to the source mesh. */
  EXPECT_FALSE(moved.mesh().Equal(this->mesh_.mesh()));
  EXPECT_TRUE(moved.mesh().Equal(ref.mesh()));

  /* Now we confirm that such move-constructed mesh properly deforms (based on
   comparing it with a reference mesh). */
  constexpr double kScale = 0.3333;
  const VectorX<T> q = this->ScaleVertexPositions(moved.mesh(), kScale);
  moved.UpdateVertexPositions(q);
  EXPECT_TRUE(moved.mesh().Equal(this->MakeBox(kScale),
                                 std::numeric_limits<double>::epsilon()));
}

TYPED_TEST(DeformableVolumeMeshTest, CopyAssignment) {
  using T = TypeParam;
  DeformableVolumeMesh<T> big(this->MakeBox(2.0));
  const DeformableVolumeMesh<T>& small = this->mesh_;

  /* Confirm that the big mesh doesn't start equal to the small mesh. */
  ASSERT_FALSE(big.mesh().Equal(small.mesh()));

  big = small;

  /* The copy's members are different *instances*, but equal values to the
   source. */
  EXPECT_NE(&big.mesh(), &small.mesh());
  EXPECT_TRUE(big.mesh().Equal(small.mesh()));

  /* Moving copy's vertex positions leaves mesh_'s vertex positions in place. */
  constexpr double kScale = 0.3333;
  const VectorX<T> q = this->ScaleVertexPositions(big.mesh(), kScale);
  big.UpdateVertexPositions(q);
  EXPECT_FALSE(big.mesh().Equal(small.mesh()));
  /* However, if we construct a box with the same scaled factor, those meshes
   will be equal. */
  const VolumeMesh<T> ref = this->MakeBox(kScale);
  EXPECT_TRUE(big.mesh().Equal(ref, std::numeric_limits<double>::epsilon()));
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

  /* Confirm that the big mesh doesn't start equal to the small mesh. */
  ASSERT_FALSE(big.mesh().Equal(small.mesh()));

  big = std::move(small);

  /* The targets's members are different *instances*, but equal values to the
   reference. */
  EXPECT_FALSE(big.mesh().Equal(small.mesh()));
  EXPECT_TRUE(big.mesh().Equal(small_ref.mesh()));

  /* Now we confirm that such move-assigned mesh properly deforms (based on
   comparing it with a reference mesh). */
  constexpr double kScale = 0.3333;
  const VectorX<T> q = this->ScaleVertexPositions(big.mesh(), kScale);
  big.UpdateVertexPositions(q);
  EXPECT_TRUE(big.mesh().Equal(this->MakeBox(kScale),
                               std::numeric_limits<double>::epsilon()));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
