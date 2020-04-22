#include "drake/geometry/query_results/contact_surface.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/mesh_field.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/math/rigid_transform.h"

// TODO(DamrongGuoy): Move to geometry/query_results/test/.
namespace drake {
namespace geometry {

// TODO(DamrongGuoy): Remove this helper class when ContactSurface allows
//  direct access to e_MN_.
template <typename T>
class ContactSurfaceTester {
 public:
  explicit ContactSurfaceTester(const geometry::ContactSurface<T>& surface)
      : surface_(surface) {}

  const SurfaceMeshFieldLinear<T, T>& e_MN() const {
    DRAKE_DEMAND(surface_.e_MN_ != nullptr);
    return *(surface_.e_MN_);
  }

  SurfaceMesh<T>& mutable_mesh_W() const {
    DRAKE_DEMAND(surface_.mesh_W_ != nullptr);
    return *(surface_.mesh_W_);
  }

 private:
  const geometry::ContactSurface<T>& surface_;
};

namespace {

using std::make_unique;
using std::move;

// TODO(DamrongGuoy): Consider splitting the test into several smaller tests
//  including a separated mesh test.
// Tests instantiation of ContactSurface and inspecting its components (the
// mesh and the mesh fields). We cannot access its mesh fields directly, so
// we check them by evaluating the field values at certain positions.
template<typename T> ContactSurface<T> TestContactSurface();

// Tests instantiation of ContactSurface using `double` as the underlying
// scalar type.
GTEST_TEST(ContactSurfaceTest, TestContactSurfaceDouble) {
  auto contact_surface = TestContactSurface<double>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no tests of
// differentiation.
GTEST_TEST(ContactSurfaceTest, TestContactSurfaceAutoDiffXd) {
  auto contact_surface = TestContactSurface<AutoDiffXd>();
}

template <typename T>
std::unique_ptr<SurfaceMesh<T>> GenerateMesh() {
// A simple mesh for a contact surface. It consists of two right
// triangles that make a square.
//
//   y
//   |
//   |
//   |
//   v3(0,1,0)  v2(1,1,0)
//   +-----------+
//   |         . |
//   |  f1  . .  |
//   |    . .    |
//   | . .   f0  |
//   |.          |
//   +-----------+---------- x
//   v0(0,0,0)  v1(1,0,0)
//
  const int face_data[2][3] = {{0, 1, 2}, {2, 3, 0}};
  std::vector<SurfaceFace> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  const Vector3<T> vertex_data[4] = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
  std::vector<SurfaceVertex<T>> vertices;
  for (int v = 0; v < 4; ++v) vertices.emplace_back(vertex_data[v]);
  auto surface_mesh =
      make_unique<SurfaceMesh<T>>(move(faces), move(vertices));
  return surface_mesh;
}

template <typename T>
ContactSurface<T> TestContactSurface() {
  auto id_M = GeometryId::get_new_id();
  auto id_N = GeometryId::get_new_id();
  auto surface_mesh = GenerateMesh<T>();

  // We record the reference for testing later.
  auto& surface_mesh_ref = *(surface_mesh.get());

  // Increasing values of `e` from one vertex to the next.
  // We give names to the values at vertices for testing later.
  const T e0{0.};
  const T e1{1.};
  const T e2{2.};
  const T e3{3.};
  std::vector<T> e_values = {e0, e1, e2, e3};
  auto e_field = make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", move(e_values), surface_mesh.get());

  ContactSurface<T> contact_surface(id_M, id_N, move(surface_mesh),
                                    move(e_field));

  // Start testing the ContactSurface<> data structure.
  EXPECT_EQ(id_M, contact_surface.id_M());
  EXPECT_EQ(id_N, contact_surface.id_N());
  // Check memory address of the mesh. We don't want to compare the mesh
  // objects themselves.
  EXPECT_EQ(&surface_mesh_ref, &contact_surface.mesh_W());
  EXPECT_EQ(2, contact_surface.mesh_W().num_faces());
  EXPECT_EQ(4, contact_surface.mesh_W().num_vertices());
  // Tests evaluation of `e` on face f0 {0, 1, 2}.
  {
    const SurfaceFaceIndex f0(0);
    const typename SurfaceMesh<T>::Barycentric b{0.2, 0.3, 0.5};
    const T expect_e = b(0) * e0 + b(1) * e1 + b(2) * e2;
    EXPECT_EQ(expect_e, contact_surface.EvaluateE_MN(f0, b));
  }
  // Tests area() of triangular faces.
  {
    EXPECT_EQ(T(0.5), contact_surface.mesh_W().area(SurfaceFaceIndex(0)));
    EXPECT_EQ(T(0.5), contact_surface.mesh_W().area(SurfaceFaceIndex(1)));
  }

  return contact_surface;
}

// Tests copy constructor of ContactSurface. We use `double` as a
// representative scalar type.
GTEST_TEST(ContactSurfaceTest, TestCopy) {
  ContactSurface<double> original = TestContactSurface<double>();
  // Copy constructor.
  ContactSurface<double> copy(original);

  // Confirm that it was a deep copy, i.e., the `original` mesh and the `copy`
  // mesh are different objects.
  EXPECT_NE(&original.mesh_W(), &copy.mesh_W());

  EXPECT_EQ(original.id_M(), copy.id_M());
  EXPECT_EQ(original.id_N(), copy.id_N());
  // We use `num_faces()` as a representative of the mesh. We do not check
  // everything in the mesh.
  EXPECT_EQ(original.mesh_W().num_faces(), copy.mesh_W().num_faces());

  // We check evaluation of field values only at one position.
  const SurfaceFaceIndex f(0);
  const typename SurfaceMesh<double>::Barycentric b{0.2, 0.3, 0.5};
  EXPECT_EQ(original.EvaluateE_MN(f, b), copy.EvaluateE_MN(f, b));
}

// Tests the equality comparisons.
GTEST_TEST(ContactSurfaceTest, TestEqual) {
  // Create contact surface for comparison.
  const ContactSurface<double> surface = TestContactSurface<double>();

  // Same contact surface.
  auto surface0 = ContactSurface<double>(surface);
  EXPECT_TRUE(surface.Equal(surface0));

  // Different mesh.
  auto surface1 = ContactSurface<double>(surface);
  ContactSurfaceTester<double>(surface1).mutable_mesh_W().ReverseFaceWinding();
  EXPECT_FALSE(surface.Equal(surface1));

  // Equal mesh, Different pressure field.
  // First, copy the mesh.
  auto mesh2 = make_unique<SurfaceMesh<double>>(surface.mesh_W());
  // TODO(DamrongGuoy): Remove this cast when we remove MeshField and use
  //  only MeshFieldLinear.
  auto field = dynamic_cast<const SurfaceMeshFieldLinear<double, double>*>(
                   &surface.e_MN());
  DRAKE_DEMAND(field);
  // Then, copy the field values and change it.
  std::vector<double> field2_values(field->values());
  field2_values.at(0) += 2.0;
  auto field2 = make_unique<SurfaceMeshFieldLinear<double, double>>(
                    field->name(), move(field2_values), mesh2.get());
  auto surface2 = ContactSurface<double>(surface.id_M(), surface.id_N(),
                                         move(mesh2), move(field2));
  EXPECT_FALSE(surface.Equal(surface2));
}

// Tests the constructor of ContactSurface that when id_M is greater than
// id_N, it will swap M and N.
GTEST_TEST(ContactSurfaceTest, TestSwapMAndN) {
  // Create the original contact surface for comparison later.
  const ContactSurface<double> original = TestContactSurface<double>();
  auto mesh = make_unique<SurfaceMesh<double>>(original.mesh_W());
  SurfaceMesh<double>* mesh_pointer = mesh.get();
  // TODO(DamrongGuoy): Remove `original_tester` when ContactSurface allows
  //  direct access to e_MN.
  const ContactSurfaceTester<double> original_tester(original);
  std::vector<double> e_MN_values = original_tester.e_MN().values();

  // Create id_M after id_N, so id_M > id_N. This condition will trigger
  // SwapMAndN in the constructor of ContactSurface.
  auto id_N = GeometryId::get_new_id();
  auto id_M = GeometryId::get_new_id();
  ASSERT_LT(id_N, id_M);
  ContactSurface<double> dut(
      id_M, id_N, move(mesh),
      make_unique<SurfaceMeshFieldLinear<double, double>>(
          "e_MN", move(e_MN_values), mesh_pointer));

  // We rely on the underlying meshes and mesh fields to *do* the right thing.
  // These tests are just to confirm that those things changed where we
  // expected to change (and not, where appropriate).

  // Ids have swapped.
  EXPECT_EQ(dut.id_M(), id_N);
  EXPECT_EQ(dut.id_N(), id_M);

  // Determines if two faces have the same indices in the same order.
  auto are_identical = [](const SurfaceFace& f1, const SurfaceFace& f2) {
    return f1.vertex(0) == f2.vertex(0) && f1.vertex(1) == f2.vertex(1) &&
           f1.vertex(2) == f2.vertex(2);
  };
  // Face winding is changed.
  for (SurfaceFaceIndex f(0); f < original.mesh_W().num_faces(); ++f) {
    EXPECT_FALSE(
        are_identical(dut.mesh_W().element(f), original.mesh_W().element(f)));
  }

  // Evaluate the mesh field, once per face for an arbitrary point Q on the
  // interior of the triangle. We expect e_MN function hasn't changed.
  const SurfaceMesh<double>::Barycentric b_Q{0.25, 0.25, 0.5};
  for (SurfaceFaceIndex f(0); f < original.mesh_W().num_faces(); ++f) {
    EXPECT_EQ(dut.EvaluateE_MN(f, b_Q), original.EvaluateE_MN(f, b_Q));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
