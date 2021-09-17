#include "drake/geometry/query_results/contact_surface.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/proximity/surface_mesh_field.h"
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

using Eigen::Vector3d;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

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
unique_ptr<SurfaceMesh<T>> GenerateMesh() {
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
  vector<SurfaceFace> faces;
  for (int f = 0; f < 2; ++f) faces.emplace_back(face_data[f]);
  vector<Vector3<T>> vertices = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
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
  vector<T> e_values = {e0, e1, e2, e3};
  auto e_field = make_unique<SurfaceMeshFieldLinear<T, T>>(move(e_values),
                                                           surface_mesh.get());

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
    const int f0{0};
    const typename SurfaceMesh<T>::template Barycentric<double> b{0.2, 0.3,
                                                                  0.5};
    const T expect_e = b(0) * e0 + b(1) * e1 + b(2) * e2;
    EXPECT_EQ(expect_e, contact_surface.e_MN().Evaluate(f0, b));
  }
  // Tests area() of triangular faces.
  {
    EXPECT_EQ(T(0.5), contact_surface.mesh_W().area(0));
    EXPECT_EQ(T(0.5), contact_surface.mesh_W().area(1));
  }

  return contact_surface;
}

// Tests the ContactSurface with constituent gradients: construction, successful
// reversal, access, etc.
GTEST_TEST(ContactSurfaceTest, ConstituentGradients) {
  // Define two ids: A and B. When we create a ContactSurface with A and B in
  // that order, then the data is taken as is (i.e., A --> M, B --> N). When
  // we reverse the order, ContactSurface will swap/reverse the data so that
  // to maintain the invariant that A --> M and B --> N.
  const auto id_A = GeometryId::get_new_id();
  const auto id_B = GeometryId::get_new_id();
  ASSERT_LT(id_A, id_B);

  unique_ptr<SurfaceMesh<double>> surface_mesh = GenerateMesh<double>();
  auto make_e_field = [](SurfaceMesh<double>* mesh) {
    vector<double> e_values{0, 1, 2, 3};
    return make_unique<SurfaceMeshFieldLinear<double, double>>(
        move(e_values), mesh, false /* calc_gradient */);
  };
  vector<Vector3d> grad_e;
  for (int i = 0; i < surface_mesh->num_elements(); ++i) {
    grad_e.push_back(Vector3d(i, i, i));
  }

  {
    // Case: Neither constituent gradient field is defined.
    const ContactSurface<double> surface(
        id_A, id_B, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), nullptr, nullptr);
    EXPECT_FALSE(surface.HasGradE_M());
    EXPECT_THROW(surface.EvaluateGradE_M_W(1), std::runtime_error);
    EXPECT_FALSE(surface.HasGradE_N());
    EXPECT_THROW(surface.EvaluateGradE_N_W(1), std::runtime_error);
  }

  {
    // Case: The first gradient is defined and id_A is first --> M has the
    // gradient.
    const ContactSurface<double> surface(
        id_A, id_B, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), make_unique<vector<Vector3d>>(grad_e),
        nullptr);
    EXPECT_TRUE(surface.HasGradE_M());
    EXPECT_EQ(surface.EvaluateGradE_M_W(1), grad_e[1]);
    EXPECT_FALSE(surface.HasGradE_N());
    EXPECT_THROW(surface.EvaluateGradE_N_W(1), std::runtime_error);
  }

  {
    // Case: The first gradient is defined and id_B is first --> N has the
    // gradient.
    const ContactSurface<double> surface(
        id_B, id_A, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), make_unique<vector<Vector3d>>(grad_e),
        nullptr);
    EXPECT_FALSE(surface.HasGradE_M());
    EXPECT_THROW(surface.EvaluateGradE_M_W(1), std::runtime_error);
    EXPECT_TRUE(surface.HasGradE_N());
    EXPECT_EQ(surface.EvaluateGradE_N_W(1), grad_e[1]);
  }

  {
    // Case: The second gradient is defined and id_B is second --> N has the
    // gradient.
    const ContactSurface<double> surface(
        id_A, id_B, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), nullptr,
        make_unique<vector<Vector3d>>(grad_e));
    EXPECT_FALSE(surface.HasGradE_M());
    EXPECT_THROW(surface.EvaluateGradE_M_W(1), std::runtime_error);
    EXPECT_TRUE(surface.HasGradE_N());
    EXPECT_EQ(surface.EvaluateGradE_N_W(1), grad_e[1]);
  }

  {
    // Case: The second gradient is defined and id_A is second --> M has the
    // gradient.
    const ContactSurface<double> surface(
        id_B, id_A, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), nullptr,
        make_unique<vector<Vector3d>>(grad_e));
    EXPECT_TRUE(surface.HasGradE_M());
    EXPECT_EQ(surface.EvaluateGradE_M_W(1), grad_e[1]);
    EXPECT_FALSE(surface.HasGradE_N());
    EXPECT_THROW(surface.EvaluateGradE_N_W(1), std::runtime_error);
  }

  vector<Vector3d> grad_e2;
  for (int i = 0; i < surface_mesh->num_elements(); ++i) {
    grad_e2.push_back(Vector3d(i + 1, i + 1, i + 1));
  }
  {
    // Case: Gradients 1 and 2 are given for A and B, so M has gradient 1 and
    // N has gradient 2.
    const ContactSurface<double> surface(
        id_A, id_B, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), make_unique<vector<Vector3d>>(grad_e),
        make_unique<vector<Vector3d>>(grad_e2));
    EXPECT_TRUE(surface.HasGradE_M());
    EXPECT_EQ(surface.EvaluateGradE_M_W(1), grad_e[1]);
    EXPECT_TRUE(surface.HasGradE_N());
    EXPECT_EQ(surface.EvaluateGradE_N_W(1), grad_e2[1]);
  }

  {
    // Case: Gradients 1 and 2 are given for B and A, so M has gradient 2 and
    // N has gradient 1.
    const ContactSurface<double> surface(
        id_B, id_A, make_unique<SurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), make_unique<vector<Vector3d>>(grad_e),
        make_unique<vector<Vector3d>>(grad_e2));
    EXPECT_TRUE(surface.HasGradE_M());
    EXPECT_EQ(surface.EvaluateGradE_M_W(1), grad_e2[1]);
    EXPECT_TRUE(surface.HasGradE_N());
    EXPECT_EQ(surface.EvaluateGradE_N_W(1), grad_e[1]);
  }
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
  const int f{0};
  const typename SurfaceMesh<double>::Barycentric<double> b{0.2, 0.3, 0.5};
  EXPECT_EQ(original.e_MN().Evaluate(f, b), copy.e_MN().Evaluate(f, b));
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
  const SurfaceMeshFieldLinear<double, double>& field = surface.e_MN();
  // Then, copy the field values and change it.
  vector<double> field2_values(field.values());
  field2_values.at(0) += 2.0;
  auto field2 = make_unique<SurfaceMeshFieldLinear<double, double>>(
      move(field2_values), mesh2.get());
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
  vector<double> e_MN_values = original_tester.e_MN().values();

  // Create id_M after id_N, so id_M > id_N. This condition will trigger
  // SwapMAndN in the constructor of ContactSurface.
  auto id_N = GeometryId::get_new_id();
  auto id_M = GeometryId::get_new_id();
  ASSERT_LT(id_N, id_M);
  ContactSurface<double> dut(
      id_M, id_N, move(mesh),
      make_unique<SurfaceMeshFieldLinear<double, double>>(move(e_MN_values),
                                                          mesh_pointer));

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
  for (int f = 0; f < original.mesh_W().num_faces(); ++f) {
    EXPECT_FALSE(
        are_identical(dut.mesh_W().element(f), original.mesh_W().element(f)));
  }

  // Evaluate the mesh field, once per face for an arbitrary point Q on the
  // interior of the triangle. We expect e_MN function hasn't changed.
  const SurfaceMesh<double>::Barycentric<double> b_Q{0.25, 0.25, 0.5};
  for (int f = 0; f < original.mesh_W().num_faces(); ++f) {
    EXPECT_EQ(dut.e_MN().Evaluate(f, b_Q), original.e_MN().Evaluate(f, b_Q));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
