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
//  direct access to e_MN_ and grad_h_MN_W_.
template <typename T>
class ContactSurfaceTester {
 public:
  explicit ContactSurfaceTester(const geometry::ContactSurface<T>& surface)
      : surface_(surface) {}

  const SurfaceMeshFieldLinear<T, T>& e_MN() const {
    DRAKE_DEMAND(surface_.e_MN_ != nullptr);
    return *(surface_.e_MN_);
  }

  const SurfaceMeshFieldLinear<Vector3<T>, T>& grad_h_MN_W() const {
    DRAKE_DEMAND(surface_.grad_h_MN_W_ != nullptr);
    return *(surface_.grad_h_MN_W_);
  }

 private:
  const geometry::ContactSurface<T>& surface_;
};

namespace {

using Eigen::AngleAxisd;
using math::RigidTransformd;
using Eigen::Vector3d;

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
      std::make_unique<SurfaceMesh<T>>(move(faces), std::move(vertices));
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
  auto e_field = std::make_unique<SurfaceMeshFieldLinear<T, T>>(
      "e", std::move(e_values), surface_mesh.get());

  // Slightly different values of grad_h_MN_W at each vertex.
  // We give names to the values at vertices for testing later.
  const Vector3<T> g0(-0.1, -0.1, 1.);
  const Vector3<T> g1(0.1, -0.1, 1.);
  const Vector3<T> g2(0.1, 0.1, 1.);
  const Vector3<T> g3(-0.1, 0.1, 1.);
  std::vector<Vector3<T>> grad_h_MN_W_values = {g0, g1, g2, g3};
  auto grad_h_MN_W_field =
      std::make_unique<SurfaceMeshFieldLinear<Vector3<T>, T>>(
          "grad_h_MN_W", std::move(grad_h_MN_W_values), surface_mesh.get());

  ContactSurface<T> contact_surface(id_M, id_N, std::move(surface_mesh),
                                    std::move(e_field),
                                    std::move(grad_h_MN_W_field));

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
  // Tests evaluation of `grad_h_MN_W` on face f1 {2, 3, 0}.
  {
    const SurfaceFaceIndex f1(1);
    const typename SurfaceMesh<T>::Barycentric b{0.6, 0.3, 0.1};
    // On face f1, we have these quantities.
    //---+--------+----------+-----------------
    // v | vertex | grad_h_MN_W | barycentric
    //---+--------+-------------+--------------
    // 0 |   v2   |      g2     |     0.6
    // 1 |   v3   |      g3     |     0.3
    // 2 |   v0   |      g0     |     0.1
    //---+--------+-------------+--------------
    const Vector3<T> expect_g = T(0.6) * g2 + T(0.3) * g3 + T(0.1) * g0;
    EXPECT_EQ(expect_g, contact_surface.EvaluateGrad_h_MN_W(f1, b));
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
  EXPECT_EQ(original.EvaluateGrad_h_MN_W(f, b), copy.EvaluateGrad_h_MN_W(f, b));
}

// Tests the constructor of ContactSurface that when id_M is greater than
// id_N, it will swap M and N.
GTEST_TEST(ContactSurfaceTest, TestSwapMAndN) {
  // Create the original contact surface for comparison later.
  const ContactSurface<double> original = TestContactSurface<double>();
  auto mesh = std::make_unique<SurfaceMesh<double>>(original.mesh_W());
  SurfaceMesh<double>* mesh_pointer = mesh.get();
  // TODO(DamrongGuoy): Remove `original_tester` when ContactSurface allows
  //  direct access to e_MN and grad_h_MN_W.
  const ContactSurfaceTester<double> original_tester(original);
  std::vector<double> e_MN_values = original_tester.e_MN().values();
  std::vector<Vector3<double>> grad_h_MN_W_values =
      original_tester.grad_h_MN_W().values();

  // Create id_M after id_N, so id_M > id_N. This condition will trigger
  // SwapMAndN in the constructor of ContactSurface.
  auto id_N = GeometryId::get_new_id();
  auto id_M = GeometryId::get_new_id();
  ASSERT_LT(id_N, id_M);
  ContactSurface<double> dut(
      id_M, id_N, std::move(mesh),
      std::make_unique<SurfaceMeshFieldLinear<double, double>>(
          "e_MN", std::move(e_MN_values), mesh_pointer),
      std::make_unique<SurfaceMeshFieldLinear<Vector3<double>, double>>(
          "grad_h_MN_W", std::move(grad_h_MN_W_values), mesh_pointer));

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

  // Test the mesh fields by evaluating each field, once per face for an
  // arbitrary point Q on the interior of the triangle. We expect:
  //    e_MN function hasn't changed.
  //    grad_H function has been mirrored.
  const SurfaceMesh<double>::Barycentric b_Q{0.25, 0.25, 0.5};
  for (SurfaceFaceIndex f(0); f < original.mesh_W().num_faces(); ++f) {
    EXPECT_EQ(dut.EvaluateE_MN(f, b_Q), original.EvaluateE_MN(f, b_Q));
    const Vector3d expected_normal = -original.EvaluateGrad_h_MN_W(f, b_Q);
    EXPECT_TRUE(CompareMatrices(dut.EvaluateGrad_h_MN_W(f, b_Q),
                                expected_normal,
                                std::numeric_limits<double>::epsilon()));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
