#include "drake/geometry/deformable_mesh.h"

#include <limits>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/make_box_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

/* We test a particular instantiation of the DeformableMesh on VolumeMesh. In
testing the move/copy semantics:

1. For copy semantics, we want to confirm that the deformation mechanism is
   wired up correctly. So, after copying, we'll confirm the meshes match, but
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
class DeformableMeshTest : public ::testing::Test {
 public:
  DeformableMeshTest() : ::testing::Test(), mesh_(MakeBox()) {}

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

  /* Extracts the vertex positions of the given `mesh` into a vector of values
   appropriate for calling UpdateVertexPositions(). */
  static VectorX<T> ExtractVertexPositions(const VolumeMesh<T>& mesh) {
    const int num_vertices = mesh.num_vertices();
    const auto& vertices = mesh.vertices();
    VectorX<T> q(num_vertices * 3);
    for (int v = 0, i = 0; v < num_vertices; ++v, i += 3) {
      q.segment(i, 3) << vertices[v];
    }
    return q;
  }

 protected:
  DeformableVolumeMesh<T> mesh_;
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd>;
TYPED_TEST_SUITE(DeformableMeshTest, ScalarTypes);

/* Verify that the mesh is a copy of the input mesh to make sure things are
 wired up correctly. */
TYPED_TEST(DeformableMeshTest, Construction) {
  /* The test class constructs the deformable mesh; invocation is implicit in
   test fixture. We'll simply confirm the listed property. */

  /* The contained mesh is a copy of the expected mesh. */
  EXPECT_TRUE(this->mesh_.mesh().Equal(this->MakeBox()));
}

TYPED_TEST(DeformableMeshTest, CopyConstructor) {
  using T = TypeParam;

  DeformableVolumeMesh<T> copy(this->mesh_);

  /* The copy's members are different *instances*, but equal values to the
   source. */
  EXPECT_NE(&copy.mesh(), &this->mesh_.mesh());
  EXPECT_TRUE(copy.mesh().Equal(this->mesh_.mesh()));

  /* Moving copy's vertex positions leaves mesh_'s vertex positions in place.
   We'll create a scaled version of the box and use its vertex positions. */
  constexpr double kScale = 0.25;
  const VolumeMesh<T> scaled = this->MakeBox(kScale);
  const VectorX<T> q = this->ExtractVertexPositions(scaled);
  copy.UpdateVertexPositions(q);
  EXPECT_FALSE(copy.mesh().Equal(this->mesh_.mesh()));
  /* However, the updated mesh matches the scaled mesh. */
  EXPECT_TRUE(copy.mesh().Equal(scaled));
}

TYPED_TEST(DeformableMeshTest, MoveConstructor) {
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

  /* Moving moved's vertex positions leaves mesh_'s vertex positions in place.
   We'll create a scaled version of the box and use its vertex positions. */
  constexpr double kScale = 0.25;
  const VolumeMesh<T> scaled = this->MakeBox(kScale);
  const VectorX<T> q = this->ExtractVertexPositions(scaled);
  moved.UpdateVertexPositions(q);
  EXPECT_FALSE(moved.mesh().Equal(this->mesh_.mesh()));
  /* However, the updated mesh matches the reference mesh. */
  EXPECT_TRUE(moved.mesh().Equal(scaled));
}

TYPED_TEST(DeformableMeshTest, CopyAssignment) {
  using T = TypeParam;
  DeformableVolumeMesh<T> dut(this->MakeBox(2.0));
  const DeformableVolumeMesh<T>& small = this->mesh_;

  /* Confirm that the dut mesh doesn't start equal to the small mesh. */
  ASSERT_FALSE(dut.mesh().Equal(small.mesh()));

  dut = small;

  /* The copy's members are different *instances*, but equal values to the
   source. */
  EXPECT_NE(&dut.mesh(), &small.mesh());
  EXPECT_TRUE(dut.mesh().Equal(small.mesh()));

  /* Moving copy's vertex positions leaves mesh_'s vertex positions in place.
   We'll create a scaled version of the box and use its vertex positions. */
  constexpr double kScale = 0.25;
  const VolumeMesh<T> scaled = this->MakeBox(kScale);
  const VectorX<T> q = this->ExtractVertexPositions(scaled);
  dut.UpdateVertexPositions(q);
  EXPECT_FALSE(dut.mesh().Equal(small.mesh()));
  /* However, the updated mesh matches the scaled mesh. */
  EXPECT_TRUE(dut.mesh().Equal(scaled));
}

TYPED_TEST(DeformableMeshTest, MoveAssignment) {
  using T = TypeParam;
  /* With move *assignment*, we need a deformable mesh with an initial value
   such that we can recognize successful reassignment. So, we create a mesh
   that is "big" (compared to the "small" source mesh) and then move the small
   mesh onto the big mesh. */
  DeformableVolumeMesh<T> dut(this->MakeBox(2.0));

  /* Construct a reference copy -- assumes the copy constructor works (see the
   CopyConstructor test). */
  const DeformableVolumeMesh<T> small_ref(this->mesh_);

  /* An alias for the "small" source deformable mesh. */
  DeformableVolumeMesh<T>& small = this->mesh_;

  /* Confirm that the dut mesh doesn't start equal to the small mesh. */
  ASSERT_FALSE(dut.mesh().Equal(small.mesh()));

  dut = std::move(small);

  /* The targets's members are different *instances*, but equal values to the
   reference. */
  EXPECT_FALSE(dut.mesh().Equal(small.mesh()));
  EXPECT_TRUE(dut.mesh().Equal(small_ref.mesh()));

  /* Moving dut's vertex positions leaves small_ref's vertex positions in place.
   We'll create a scaled version of the box and use its vertex positions. */
  constexpr double kScale = 0.25;
  const VolumeMesh<T> scaled = this->MakeBox(kScale);
  const VectorX<T> q = this->ExtractVertexPositions(scaled);
  dut.UpdateVertexPositions(q);
  EXPECT_FALSE(dut.mesh().Equal(small_ref.mesh()));
  /* However, the updated mesh matches the reference mesh. */
  EXPECT_TRUE(dut.mesh().Equal(scaled));
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
