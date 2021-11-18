#include "drake/geometry/query_results/contact_surface.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/triangle_surface_mesh_field.h"
#include "drake/math/rigid_transform.h"

// TODO(DamrongGuoy): Move to geometry/query_results/test/.
namespace drake {
namespace geometry {

template <typename T>
class ContactSurfaceTester {
 public:
  explicit ContactSurfaceTester(const geometry::ContactSurface<T>& surface)
      : surface_(surface) {}

 private:
  const geometry::ContactSurface<T>& surface_;
};

namespace {

using Eigen::Vector3d;
using std::make_unique;
using std::move;
using std::unique_ptr;
using std::vector;

template <typename MeshType>
unique_ptr<MeshType> GenerateMesh() {
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
  using T = typename MeshType::ScalarType;
  vector<Vector3<T>> vertices = {
      {0., 0., 0.}, {1., 0., 0.}, {1., 1., 0.}, {0., 1., 0.}};
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    vector<SurfaceTriangle> faces{{0, 1, 2}, {2, 3, 0}};
    return make_unique<TriangleSurfaceMesh<T>>(move(faces), move(vertices));
  } else {
    vector<int> face_data{3, 0, 1, 2, 3, 2, 3, 0};
    return make_unique<PolygonSurfaceMesh<T>>(move(face_data), move(vertices));
  }
}

// Creates a field for a mesh created by GenerateMesh().
template <typename MeshType>
unique_ptr<MeshFieldLinear<typename MeshType::ScalarType, MeshType>> MakeField(
    vector<typename MeshType::ScalarType>&& e_values, const MeshType& mesh) {
  using T = typename MeshType::ScalarType;
  if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
    return make_unique<MeshFieldLinear<T, MeshType>>(move(e_values), &mesh);
  } else {
    // A MeshFieldLinear on PolygonSurfaceMesh requires the gradients to be
    // provided. So, we'll create the equivalent triangle mesh and field and
    // ask for *its* gradients. Because both meshes consist purely of triangles,
    // we can easily map from one to the other.
    auto tri_mesh = GenerateMesh<TriangleSurfaceMesh<T>>();
    TriangleSurfaceMeshFieldLinear<T, T> field(vector<T>(e_values),
                                               tri_mesh.get());
    vector<Vector3<T>> e_grad;
    for (int t = 0; t < tri_mesh->num_elements(); ++t) {
      e_grad.push_back(field.EvaluateGradient(t));
    }
    return make_unique<MeshFieldLinear<T, MeshType>>(move(e_values), &mesh,
                                                     move(e_grad));
  }
}

// Generates a ContactSurface from the mesh returned by GenerateMesh(). By
// default, certain invariants of the test will be validated. But that can be
// skipped if we only need a contact surface.
// We also test the sugar mesh representation-independent APIs if
// `test_return_value` is `true`. We don't test the mesh field internal values
// directly. Instead, we evaluate the field at various positions.
template <typename MeshType>
ContactSurface<typename MeshType::ScalarType> TestContactSurface(
    bool test_return_value = true) {
  using T = typename MeshType::ScalarType;
  auto id_M = GeometryId::get_new_id();
  auto id_N = GeometryId::get_new_id();
  auto surface_mesh = GenerateMesh<MeshType>();

  // We record the reference for testing later.
  auto& surface_mesh_ref = *(surface_mesh.get());

  // Increasing values of `e` from one vertex to the next.
  // We give names to the values at vertices for testing later.
  const T e0{0.};
  const T e1{1.};
  const T e2{2.};
  const T e3{3.};
  vector<T> e_values = {e0, e1, e2, e3};
  unique_ptr<MeshFieldLinear<T, MeshType>> e_field =
      MakeField(move(e_values), *surface_mesh);

  ContactSurface<T> contact_surface(id_M, id_N, move(surface_mesh),
                                    move(e_field));

  if (test_return_value) {
    // Start testing the ContactSurface<> data structure.
    EXPECT_EQ(id_M, contact_surface.id_M());
    EXPECT_EQ(id_N, contact_surface.id_N());
    EXPECT_EQ(2, contact_surface.num_faces());
    EXPECT_EQ(4, contact_surface.num_vertices());
    // Tests area() of triangular faces.
    EXPECT_EQ(T(0.5), contact_surface.area(0));
    EXPECT_EQ(T(0.5), contact_surface.area(1));
    EXPECT_EQ(1.0, contact_surface.total_area());
    // Tests centroids.
    EXPECT_TRUE(
        CompareMatrices(contact_surface.centroid(), Vector3<T>{0.5, 0.5, 0}));
    EXPECT_TRUE(
        CompareMatrices(contact_surface.centroid(0), Vector3<T>{2, 1, 0} / 3));
    EXPECT_TRUE(
        CompareMatrices(contact_surface.centroid(1), Vector3<T>{1, 2, 0} / 3));
    // Face normals
    EXPECT_TRUE(
        CompareMatrices(contact_surface.face_normal(0), Vector3<T>{0, 0, 1}));
    EXPECT_TRUE(
        CompareMatrices(contact_surface.face_normal(1), Vector3<T>{0, 0, 1}));

    // We'll confirm the surface properly references its field. The previous
    // tests already suggest correctness of mesh and field values.
    if constexpr (std::is_same_v<MeshType, TriangleSurfaceMesh<T>>) {
      EXPECT_EQ(contact_surface.representation(),
                HydroelasticContactRepresentation::kTriangle);
      EXPECT_TRUE(contact_surface.is_triangle());
      EXPECT_EQ(&surface_mesh_ref, &contact_surface.tri_mesh_W());
      EXPECT_EQ(contact_surface.tri_e_MN().EvaluateCartesian(
                    0, contact_surface.centroid(0)),
                (e0 + e1 + e2) / 3);
      EXPECT_EQ(contact_surface.tri_e_MN().EvaluateCartesian(
                    1, contact_surface.centroid(1)),
                (e2 + e3 + e0) / 3);
    } else {
      EXPECT_EQ(contact_surface.representation(),
                HydroelasticContactRepresentation::kPolygon);
      EXPECT_FALSE(contact_surface.is_triangle());
      EXPECT_EQ(&surface_mesh_ref, &contact_surface.poly_mesh_W());
      EXPECT_EQ(contact_surface.poly_e_MN().EvaluateCartesian(
                      0, contact_surface.centroid(0)),
                  (e0 + e1 + e2) / 3);
      EXPECT_EQ(contact_surface.poly_e_MN().EvaluateCartesian(
                      1, contact_surface.centroid(1)),
                  (e2 + e3 + e0) / 3);
    }
  }

  return contact_surface;
}

// Tests instantiation of ContactSurface using `double` as the underlying
// scalar type.
GTEST_TEST(ContactSurfaceTest, TestContactSurfaceDouble) {
  TestContactSurface<TriangleSurfaceMesh<double>>();
  TestContactSurface<PolygonSurfaceMesh<double>>();
}

// Smoke tests using `AutoDiffXd` as the underlying scalar type. The purpose
// of this test is simply to check that it compiles. There is no tests of
// differentiation.
GTEST_TEST(ContactSurfaceTest, TestContactSurfaceAutoDiffXd) {
  TestContactSurface<TriangleSurfaceMesh<AutoDiffXd>>();
  TestContactSurface<PolygonSurfaceMesh<AutoDiffXd>>();
}

// Tests the ContactSurface with constituent gradients: construction, successful
// reversal, access, etc. This logic doesn't depend on mesh representation
// so, we simply test it against one type.
GTEST_TEST(ContactSurfaceTest, ConstituentGradients) {
  // Define two ids: A and B. When we create a ContactSurface with A and B in
  // that order, then the data is taken as is (i.e., A --> M, B --> N). When
  // we reverse the order, ContactSurface will swap/reverse the data so that
  // to maintain the invariant that A --> M and B --> N.
  const auto id_A = GeometryId::get_new_id();
  const auto id_B = GeometryId::get_new_id();
  ASSERT_LT(id_A, id_B);

  unique_ptr<TriangleSurfaceMesh<double>> surface_mesh =
      GenerateMesh<TriangleSurfaceMesh<double>>();
  auto make_e_field = [](TriangleSurfaceMesh<double>* mesh) {
    vector<double> e_values{0, 1, 2, 3};
    return make_unique<TriangleSurfaceMeshFieldLinear<double, double>>(
        move(e_values), mesh, false /* calc_gradient */);
  };
  vector<Vector3d> grad_e;
  for (int i = 0; i < surface_mesh->num_elements(); ++i) {
    grad_e.push_back(Vector3d(i, i, i));
  }

  {
    // Case: Neither constituent gradient field is defined.
    const ContactSurface<double> surface(
        id_A, id_B, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
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
        id_A, id_B, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
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
        id_B, id_A, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
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
        id_A, id_B, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
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
        id_B, id_A, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
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
        id_A, id_B, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
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
        id_B, id_A, make_unique<TriangleSurfaceMesh<double>>(*surface_mesh),
        make_e_field(surface_mesh.get()), make_unique<vector<Vector3d>>(grad_e),
        make_unique<vector<Vector3d>>(grad_e2));
    EXPECT_TRUE(surface.HasGradE_M());
    EXPECT_EQ(surface.EvaluateGradE_M_W(1), grad_e2[1]);
    EXPECT_TRUE(surface.HasGradE_N());
    EXPECT_EQ(surface.EvaluateGradE_N_W(1), grad_e[1]);
  }
}

// Tests copy constructor of ContactSurface.
GTEST_TEST(ContactSurfaceTest, TestCopy) {
  ContactSurface<double> original_tri =
      TestContactSurface<TriangleSurfaceMesh<double>>(false /*test*/);
  ASSERT_EQ(original_tri.representation(),
            HydroelasticContactRepresentation::kTriangle);

  auto test_copy = [](const auto& source_mesh, const auto& target_mesh) {
    // They have different addresses; it is a deep copy.
    EXPECT_NE(&source_mesh, &target_mesh);
    // They are structurally similar; this doesn't confirm that all the values
    // are copied -- that should be tested by the mesh type's unit tests.
    EXPECT_EQ(source_mesh.num_elements(), target_mesh.num_elements());
    EXPECT_EQ(source_mesh.num_vertices(), target_mesh.num_vertices());
  };

  // Copy constructor.
  ContactSurface<double> copy_tri(original_tri);
  ASSERT_EQ(copy_tri.representation(),
            HydroelasticContactRepresentation::kTriangle);

  EXPECT_EQ(original_tri.id_M(), copy_tri.id_M());
  EXPECT_EQ(original_tri.id_N(), copy_tri.id_N());

  test_copy(original_tri.tri_mesh_W(), copy_tri.tri_mesh_W());

  // We check evaluation of field values only at one position.
  const int f{0};
  const Vector3d p_MC0 = original_tri.tri_mesh_W().element_centroid(f);
  EXPECT_EQ(original_tri.tri_e_MN().EvaluateCartesian(f, p_MC0),
            copy_tri.tri_e_MN().EvaluateCartesian(f, p_MC0));

  // Repeat the test for polygon mesh representation.
  ContactSurface<double> original_poly =
      TestContactSurface<PolygonSurfaceMesh<double>>(false /*test*/);
  ASSERT_EQ(original_poly.representation(),
            HydroelasticContactRepresentation::kPolygon);

  ContactSurface<double> copy_poly(original_poly);
  ASSERT_EQ(copy_poly.representation(),
            HydroelasticContactRepresentation::kPolygon);

  EXPECT_EQ(original_poly.id_M(), copy_poly.id_M());
  EXPECT_EQ(original_poly.id_N(), copy_poly.id_N());

  test_copy(original_poly.poly_mesh_W(), copy_poly.poly_mesh_W());

  // We check evaluation of field values only at one position.
  EXPECT_EQ(original_poly.poly_e_MN().EvaluateCartesian(f, p_MC0),
            copy_poly.poly_e_MN().EvaluateCartesian(f, p_MC0));

  // Because we now have different mesh representations (triangle and polygon).
  // Copy *assignment* should change the type of the representation. Let's
  // confirm.

  copy_tri = original_poly;
  ASSERT_EQ(copy_tri.representation(),
            HydroelasticContactRepresentation::kPolygon);
  copy_poly = original_tri;
  ASSERT_EQ(copy_poly.representation(),
            HydroelasticContactRepresentation::kTriangle);
}

// TODO(DamrongGuoy): This test should also be run with a polygon
// representation.
// Tests the equality comparisons.
GTEST_TEST(ContactSurfaceTest, TestEqual) {
  // Create contact surface for comparison.
  const ContactSurface<double> surface =
      TestContactSurface<TriangleSurfaceMesh<double>>(false /*test*/);

  // Same contact surface.
  auto surface0 = ContactSurface<double>(surface);
  EXPECT_TRUE(surface.Equal(surface0));

  // To get a "different" mesh, we'll copy the current contact surface so it's
  // all the same and then sneak around the const access to change the mesh by
  // reversing its winding. That will be sufficient to show mesh differences
  // imply constact surface differences (even if all else is bit identical).
  auto surface1 = ContactSurface<double>(surface);
  const_cast<TriangleSurfaceMesh<double>&>(surface1.tri_mesh_W())
      .ReverseFaceWinding();
  EXPECT_FALSE(surface.Equal(surface1));

  // Equal mesh, Different pressure field.
  // First, copy the mesh.
  auto mesh2 = make_unique<TriangleSurfaceMesh<double>>(surface.tri_mesh_W());
  const TriangleSurfaceMeshFieldLinear<double, double>& field =
      surface.tri_e_MN();
  // Then, copy the field values and change it.
  vector<double> field2_values(field.values());
  field2_values.at(0) += 2.0;
  auto field2 = make_unique<TriangleSurfaceMeshFieldLinear<double, double>>(
      move(field2_values), mesh2.get());
  auto surface2 = ContactSurface<double>(surface.id_M(), surface.id_N(),
                                         move(mesh2), move(field2));
  EXPECT_FALSE(surface.Equal(surface2));
}

// Tests the constructor of ContactSurface that when id_M is greater than
// id_N, it will swap M and N. The logic doesn't depend on mesh
// representation or scalar type, so we simply test it against one mesh type.
GTEST_TEST(ContactSurfaceTest, TestSwapMAndN) {
  // Create the original contact surface for comparison later.
  const ContactSurface<double> original =
      TestContactSurface<TriangleSurfaceMesh<double>>(false /*test*/);
  auto mesh = make_unique<TriangleSurfaceMesh<double>>(original.tri_mesh_W());
  TriangleSurfaceMesh<double>* mesh_pointer = mesh.get();
  vector<double> e_MN_values = original.tri_e_MN().values();

  // Create id_M after id_N, so id_M > id_N. This condition will trigger
  // SwapMAndN in the constructor of ContactSurface.
  auto id_N = GeometryId::get_new_id();
  auto id_M = GeometryId::get_new_id();
  ASSERT_LT(id_N, id_M);
  ContactSurface<double> dut(
      id_M, id_N, move(mesh),
      make_unique<TriangleSurfaceMeshFieldLinear<double, double>>(
          move(e_MN_values), mesh_pointer));

  // We rely on the underlying meshes and mesh fields to *do* the right thing.
  // These tests are just to confirm that those things changed where we
  // expected to change (and not, where appropriate).

  // Ids have swapped.
  EXPECT_EQ(dut.id_M(), id_N);
  EXPECT_EQ(dut.id_N(), id_M);

  // Determines if two faces have the same indices in the same order.
  auto are_identical = [](const SurfaceTriangle& f1,
                          const SurfaceTriangle& f2) {
    return f1.vertex(0) == f2.vertex(0) && f1.vertex(1) == f2.vertex(1) &&
           f1.vertex(2) == f2.vertex(2);
  };
  // Face winding is changed.
  for (int f = 0; f < original.tri_mesh_W().num_triangles(); ++f) {
    EXPECT_FALSE(are_identical(dut.tri_mesh_W().element(f),
                               original.tri_mesh_W().element(f)));
  }

  // Evaluate the mesh field, once per face for an arbitrary point Q on the
  // interior of the triangle. We expect e_MN function hasn't changed.
  const TriangleSurfaceMesh<double>::Barycentric<double> b_Q{0.25, 0.25, 0.5};
  for (int f = 0; f < original.tri_mesh_W().num_triangles(); ++f) {
    EXPECT_EQ(dut.tri_e_MN().Evaluate(f, b_Q),
              original.tri_e_MN().Evaluate(f, b_Q));
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
