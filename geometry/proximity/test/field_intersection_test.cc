#include "drake/geometry/proximity/field_intersection.h"

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/contact_surface_utility.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using hydroelastic::SoftMesh;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

// This fixture is for intersection of two tetrahedra with linear functions.
// We set up the tetrahedra and linear functions in such a way that
// the two functions have the equilibrium plane that intersects the two
// tetrahedra into an octagon. See the picture:
// geometry/proximity/images/two_linear_tetrahedra_intersect_into_octagon.png
class FieldIntersectionLowLevelTest : public ::testing::Test {
 public:
  FieldIntersectionLowLevelTest()
      : mesh0_M_(std::vector<VolumeElement>{{0, 1, 2, 3}},
                 std::vector<Vector3d>{{2.0, 0.0, 2.0},
                                       {-2.0, 0.0, 2.0},
                                       {0.0, 2.0, -2.0},
                                       {0.0, -2.0, -2.0}}),
        field0_M_({8e4, 4e4, 4e4, 0}, &mesh0_M_),  // (4 + x + y + z)*10⁴
        mesh1_N_(std::vector<VolumeElement>{{0, 1, 2, 3}},
                 std::vector<Vector3d>{{1.5, 1.5, 2.5},
                                       {-1.5, -1.5, 2.5},
                                       {-1.5, 1.5, -2.5},
                                       {1.5, -1.5, -2.5}}),
        field1_N_({7e4, 1e4, 4e4, 4e4}, &mesh1_N_)  // (4 + x + y)*10⁴
  {}

 protected:
  // Returns the mesh expressed in frame F given X_FG and the mesh expressed
  // in frame G.
  static VolumeMesh<double> TransformVolumeMesh(
      const RigidTransformd& X_FG, const VolumeMesh<double>& mesh_G) {
    std::vector<Vector3d> p_FVs;
    for (const Vector3d& p_GV : mesh_G.vertices()) {
      p_FVs.emplace_back(X_FG * p_GV);
    }
    return {std::vector<VolumeElement>(mesh_G.tetrahedra()), std::move(p_FVs)};
  }

  const VolumeMesh<double> mesh0_M_;
  const VolumeMeshFieldLinear<double, double> field0_M_;
  const VolumeMesh<double> mesh1_N_;
  const VolumeMeshFieldLinear<double, double> field1_N_;

  double tolerance(double scale = 1) {
    const double kEps = 1e-14;
    return (scale > 1) ? kEps * scale : kEps;
  }
};

// For simplicity, this test identifies the two frames of the two meshes to
// be the same, so we can easily calculate the expected equilibrium plane for
// verification. The next test will set up a complex rigid transform between
// two frames.
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneIdenticalFrames) {
  const auto X_MN = RigidTransformd::Identity();
  //     f0(x,y,z) = (4 + x + y + z)*10⁴   (1)
  //     f1(x,y,z) = (4 + x + y)*10⁴       (2)
  // The equilibrium plane satisfies:
  //       f0 - f1 = z*10⁴ = 0      (1)-(2) = 0.
  // Therefore, z = 0 is the equilibrium plane. We pick the plane normal
  // +UnitZ() instead of -UnitZ(), so that it points out of f1 and into f0.
  const Plane<double> expected_plane_M{Vector3d::UnitZ(), Vector3d::Zero()};

  // Initialize the plane to be different from the expected plane.
  Plane<double> plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};
  const bool success =
      CalcEquilibriumPlane(0, field0_M_, 0, field1_N_, X_MN, &plane_M);

  ASSERT_TRUE(success);
  EXPECT_TRUE(CompareMatrices(plane_M.unit_normal(),
                              expected_plane_M.unit_normal(), tolerance()));
  // The choice of this query point is arbitrary.
  const Vector3d p_MQ(0.1, 0.2, 0.3);
  const double expected_height = expected_plane_M.CalcHeight<double>(p_MQ);
  EXPECT_NEAR(plane_M.CalcHeight<double>(p_MQ), expected_height,
              tolerance(expected_height));
}

// Expresses the two meshes in two different frames F and G for a
// stronger test. Correctness of this test also depends on the previous test.
//
//     mesh0_F <--> mesh0_M  <--M=N--> mesh1_N <--> mesh1_G
//
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneComplexTransform) {
  const auto X_FG = RigidTransformd(RollPitchYawd(M_PI_4, M_PI / 3, M_PI_2),
                                    Vector3d(-1., -2., -3.));
  const auto X_FM = RigidTransformd(RollPitchYawd(M_PI, M_PI_2, M_PI / 3),
                                    Vector3d(2, -6, 4));
  // From the previous test, we know that identifying frame M with frame N
  // will give a simple expression of the expected equilibrium plane in frame M.
  const RigidTransformd X_MN = RigidTransformd::Identity();
  const Vector3d expected_normal_M = Vector3d::UnitZ();
  const Vector3d expected_p_M = Vector3d::Zero();
  Plane<double> expected_plane_F(X_FM.rotation() * expected_normal_M,
                                 X_FM * expected_p_M);
  const RigidTransformd X_GN = X_FG.InvertAndCompose(X_FM) * X_MN;

  const VolumeMesh<double> mesh0_F = TransformVolumeMesh(X_FM, mesh0_M_);
  const VolumeMeshFieldLinear<double, double> field0_F(
      std::vector<double>(field0_M_.values()), &mesh0_F);

  const VolumeMesh<double> mesh1_G = TransformVolumeMesh(X_GN, mesh1_N_);
  const VolumeMeshFieldLinear<double, double> field1_G(
      std::vector<double>(field1_N_.values()), &mesh1_G);

  // Initialize the plane arbitrarily.
  Plane<double> plane_F{Vector3d(-1, -2, -3), Vector3d(-4, -5, -6)};
  bool success = CalcEquilibriumPlane(0, field0_F, 0, field1_G, X_FG, &plane_F);

  ASSERT_TRUE(success);
  EXPECT_TRUE(CompareMatrices(plane_F.unit_normal(),
                              expected_plane_F.unit_normal(), tolerance()));
  // The choice of this query point is arbitrary.
  const Vector3d p_FQ(0.1, 0.2, 0.3);
  const double expected_height = expected_plane_F.CalcHeight<double>(p_FQ);
  EXPECT_NEAR(plane_F.CalcHeight<double>(p_FQ), expected_height,
              tolerance(expected_height));
}

// Tests special cases when the two linear functions have no equilibrium
// plane. It can happen when their gradients are deemed identical, which means
// the two functions are equal everywhere, or they are nowhere equal.
TEST_F(FieldIntersectionLowLevelTest, CalcEquilibriumPlaneNone) {
  // Same function in the same frame are equal everywhere, so there is no
  // equilibrium plane. (Mathematically speaking, every plane is an
  // equilibrium plane.)
  {
    // An arbitrary initial value of the plane.
    const Plane<double> init_plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};

    Plane<double> plane_M(init_plane_M);
    const bool success = CalcEquilibriumPlane(
        0, field0_M_, 0, field0_M_, RigidTransformd::Identity(), &plane_M);

    ASSERT_FALSE(success);
    // Verify that the plane has not changed from its initial value.
    EXPECT_EQ(plane_M.unit_normal(), init_plane_M.unit_normal());
    // The choice of this query point is arbitrary.
    const Vector3d p_FQ(0.1, 0.2, 0.3);
    const double expected_height = init_plane_M.CalcHeight<double>(p_FQ);
    EXPECT_EQ(plane_M.CalcHeight<double>(p_FQ), expected_height);
  }
  // Use a translational frame L with respect to frame M to make two functions
  // that are nowhere equal.
  {
    const RigidTransformd X_ML(Vector3d(0.4, 0.3, 0.5));
    std::unique_ptr<VolumeMeshFieldLinear<double, double>> field2_L_ptr =
        field0_M_.CloneAndSetMesh(&field0_M_.mesh());
    const VolumeMeshFieldLinear<double, double>& field2_L = *field2_L_ptr;

    // An arbitrary initial value of the plane.
    const Plane<double> init_plane_M{Vector3d::UnitX(), Vector3d(1, 2, 3)};

    Plane<double> plane_M(init_plane_M);
    const bool success =
        CalcEquilibriumPlane(0, field0_M_, 0, field2_L, X_ML, &plane_M);

    ASSERT_FALSE(success);
    // Verify that the plane has not changed from its initial value.
    EXPECT_EQ(plane_M.unit_normal(), init_plane_M.unit_normal());
    // The choice of this query point is arbitrary.
    const Vector3d p_FQ(0.1, 0.2, 0.3);
    const double expected_height = init_plane_M.CalcHeight<double>(p_FQ);
    EXPECT_EQ(plane_M.CalcHeight<double>(p_FQ), expected_height);
  }
}

// Tests two tetrahedra intersecting their pressure-equilibrium plane into an
// octahedron as shown in this picture:
// geometry/proximity/images/two_linear_tetrahedra_intersect_into_octagon.png
TEST_F(FieldIntersectionLowLevelTest, IntersectTetrahedra) {
  const int first_element_in_field0{0};
  const int first_element_in_field1{0};
  const RigidTransformd identity_X_MN = RigidTransformd::Identity();
  Plane<double> plane_M{Vector3d::UnitZ(), Vector3d::Zero()};
  bool success = CalcEquilibriumPlane(first_element_in_field0, field0_M_,
                                      first_element_in_field1, field1_N_,
                                      identity_X_MN, &plane_M);
  ASSERT_TRUE(success);

  const auto [polygon_M, faces] = IntersectTetrahedra(
      first_element_in_field0, field0_M_.mesh(), first_element_in_field1,
      field1_N_.mesh(), identity_X_MN, plane_M);

  ASSERT_EQ(polygon_M.size(), 8);
  ASSERT_EQ(faces.size(), 8);

  // Use empirical tolerance 1e-14 meters.
  const double kEps = 1e-14;
  // This check relies on the order of tetrahedral elements in the mesh. It
  // might need to change if the mesh changes. See the picture
  // two_linear_tetrahedra_intersect_into_octagon.png
  // to infer coordinates of the polygon's vertices together with this diagram:
  //
  //                  +Y
  //                   ^
  //                   |              Pi = position of i-th vertex
  //                   |
  //            P5     1      P4
  //                   |
  //                   |
  //     P6           0.5            P3
  //                   |
  //                   |
  //     +------+------0------+------+---------> +X
  //    -1    -0.5     |     0.5     1
  //                   |
  //     P7          -0.5            P2
  //                   |
  //                   |
  //            P0    -1      P1

  EXPECT_TRUE(CompareMatrices(polygon_M.at(0), Vector3d(-0.5, -1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(1), Vector3d(0.5, -1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(2), Vector3d(1, -0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(3), Vector3d(1, 0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(4), Vector3d(0.5, 1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(5), Vector3d(-0.5, 1, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(6), Vector3d(-1, 0.5, 0), kEps));
  EXPECT_TRUE(CompareMatrices(polygon_M.at(7), Vector3d(-1, -0.5, 0), kEps));

  // Verifies the expected intersected faces. The particular (cyclic) order of
  // the faces is not strictly important. What is important is that all 4 faces
  // of each tetrahedra appear, and that (by construction) the edges alternate
  // intersection with a face of tet0 and a face of tet1.
  EXPECT_EQ(faces.at(0), 2);  // Face 2 of tet0
  EXPECT_EQ(faces.at(1), 6);  // Face 2 of tet1
  EXPECT_EQ(faces.at(2), 1);  // Face 1 of tet0
  EXPECT_EQ(faces.at(3), 5);  // Face 1 of tet1
  EXPECT_EQ(faces.at(4), 3);  // Face 3 of tet0
  EXPECT_EQ(faces.at(5), 7);  // Face 3 of tet1
  EXPECT_EQ(faces.at(6), 0);  // Face 0 of tet0
  EXPECT_EQ(faces.at(7), 4);  // Face 0 of tet1
}

// Move the equilibrium plane so far away that the above IntersectTetrahedra
// test becomes no intersection.
TEST_F(FieldIntersectionLowLevelTest, IntersectTetrahedra_NoIntersection) {
  const int first_element_in_mesh0{0};
  const int first_element_in_mesh1{0};
  // This plane is far away from the two tetrahedra.
  const Plane<double> plane_M{Vector3d::UnitZ(), 5.0 * Vector3d::UnitZ()};

  const auto [polygon_M, faces] = IntersectTetrahedra(
      first_element_in_mesh0, field0_M_.mesh(), first_element_in_mesh1,
      field1_N_.mesh(), RigidTransformd::Identity(), plane_M);

  EXPECT_EQ(polygon_M.size(), 0);
  EXPECT_EQ(faces.size(), 0);
}

TEST_F(FieldIntersectionLowLevelTest, IsPlaneNormalAlongPressureGradient) {
  const int first_tetrahedron_in_field0{0};

  // The field0_M_ is (4 + x + y + z)*10⁴, so its gradient vector is
  // (10⁴, 10⁴, 10⁴). Therefore, this unit vector is along that gradient
  // vector.
  const Vector3d nhat_M_along_gradient = Vector3d(1, 1, 1).normalized();
  EXPECT_TRUE(IsPlaneNormalAlongPressureGradient(
      nhat_M_along_gradient, first_tetrahedron_in_field0, field0_M_));

  const Vector3d nhat_M_against_gradient = -nhat_M_along_gradient;
  EXPECT_FALSE(IsPlaneNormalAlongPressureGradient(
      nhat_M_against_gradient, first_tetrahedron_in_field0, field0_M_));
}

// The tet-based and tri-based algorithms will generally produce the same
// results. The exception is if one mesh is completely within the other (such
// that their surfaces don't intersect). In that case, the tri-based algorithm
// may incorrectly report no contact. We say "may" because there is a
// possibility that some surface elements are sufficiently close to each other
// that the leaf node bounding volumes of their respective trees overlap. In
// this case the search algorithm of the tri-based algorithm *might* be seeded
// with a candidate pair that calculates a part of the contact surface (if the
// candidate tetrahedra actually overlap and the contact polygon is not culled
// based on the pressure gradients). Thus, only if the surfaces do not intersect
// AND all leaf nodes of the BVHs are disjoint, then no contact surface will be
// reported. This test quantifies that difference.
// Note: this is not a desirable behavior.
GTEST_TEST(VolumeIntersectionTest, FullyPenetrated) {
  auto make_ball = [](double radius) {
    const Sphere ball(radius);
    auto mesh =
        std::make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
            ball, radius, TessellationStrategy::kSingleInteriorVertex));
    auto field = std::make_unique<VolumeMeshFieldLinear<double, double>>(
        MakeSpherePressureField<double>(ball, mesh.get(), 1e5));
    return SoftMesh(std::move(mesh), std::move(field));
  };

  SoftMesh big_sphere_M = make_ball(1.0);
  SoftMesh small_sphere_N = make_ball(1.0 / 8.0);

  // Place the small sphere well within the big sphere, without surfaces
  // touching.
  const RigidTransformd X_MN(0.5 * Vector3d::UnitX());

  std::array<std::unique_ptr<PolygonSurfaceMesh<double>>, 2> surface_01_M;
  std::array<std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>>, 2>
      e_MN_M;

  VolumeIntersector<PolyMeshBuilder<double>, Obb>().IntersectFields(
      big_sphere_M.pressure(), big_sphere_M.bvh(), small_sphere_N.pressure(),
      small_sphere_N.bvh(), X_MN, &surface_01_M[0], &e_MN_M[0]);

  VolumeIntersector<PolyMeshBuilder<double>, Obb>().IntersectFields(
      big_sphere_M.pressure(), big_sphere_M.surface_mesh_bvh(),
      big_sphere_M.tri_to_tet(), big_sphere_M.mesh_topology(),
      small_sphere_N.pressure(), small_sphere_N.surface_mesh_bvh(),
      small_sphere_N.tri_to_tet(), small_sphere_N.mesh_topology(), X_MN,
      &surface_01_M[1], &e_MN_M[1]);

  // Index 0 only uses tets.
  EXPECT_NE(surface_01_M[0].get(), nullptr);
  // Index 1 depends on intersection at the surface (and non-intersecting BVH
  // leaf nodes).
  EXPECT_EQ(surface_01_M[1].get(), nullptr);
}

class VolumeIntersectorTest : public ::testing::Test {
 public:
  VolumeIntersectorTest() {
    std::unique_ptr<VolumeMesh<double>> mesh =
        std::make_unique<VolumeMesh<double>>(
            MakeBoxVolumeMeshWithMa<double>(box_));
    std::unique_ptr<VolumeMeshFieldLinear<double, double>> field =
        std::make_unique<VolumeMeshFieldLinear<double, double>>(
            MakeBoxPressureField<double>(box_, mesh.get(),
                                         kBoxElasitcModulus_));
    // Get a mesh of an octahedron from a sphere specification by
    // specifying very coarse resolution hint.
    std::unique_ptr<VolumeMesh<double>> octahedron_mesh =
        std::make_unique<VolumeMesh<double>>(MakeSphereVolumeMesh<double>(
            sphere_, 10 * sphere_.radius(),
            TessellationStrategy::kSingleInteriorVertex));

    std::unique_ptr<VolumeMeshFieldLinear<double, double>> octahedron_field =
        std::make_unique<VolumeMeshFieldLinear<double, double>>(
            MakeSpherePressureField<double>(sphere_, octahedron_mesh.get(),
                                            kOctahedronElasticModulus_));

    box_M_ = SoftMesh(std::move(mesh), std::move(field));
    octahedron_N_ =
        SoftMesh(std::move(octahedron_mesh), std::move(octahedron_field));
  }

 protected:
  void SetUp() override {
    DRAKE_DEMAND(octahedron_N_.mesh().num_elements() == 8);
  }

  // Geometry 0 and its field.
  const Box box_{0.06, 0.10, 0.14};  // 6cm-thick compliant pad.
  const double kBoxElasitcModulus_{1.0e5};
  SoftMesh box_M_;

  // Geometry 1 and its field.
  const Sphere sphere_{0.03};  // 3cm-radius (6cm-diameter) finger tip.
  const double kOctahedronElasticModulus_{1.0e5};
  SoftMesh octahedron_N_;
};

// The two algorithms provided by VolumeIntersector only differ in the manner in
// which they find candidate tetrahedra pairs from the two MeshFieldLinear
// objects. Each end up calling CalcContactPolygon() with their candidate pairs.
// Rather than comparing contact surfaces directly -- which is cumbersome
// because we have to handle equivalent topology variants and floating point
// issues -- we infer equivalency by looking at the build history. The
// intersector stores the pair of tets which produce each element in the contact
// surface mesh. We'll simply confirm that the two sets tet-pairs match (and
// safely assume that they would produce the same surface).
template <class MeshBuilder>
bool ContactSurfacesAreEqual(
    const VolumeIntersector<MeshBuilder, Obb>& intersector_A,
    const typename MeshBuilder::MeshType& surface_A,
    const VolumeIntersector<MeshBuilder, Obb>& intersector_B,
    const typename MeshBuilder::MeshType& surface_B) {
  if (surface_A.num_elements() != surface_B.num_elements()) {
    return false;
  }

  // When computing a triangulated contact surface, a single tet pair can
  // produce several triangles, in which case the tet0/tet1 pair will be
  // recorded multiple times. Thus we use a multiset to also check that the
  // the number of surface elements produced by each tet pair visited in each
  // algorithm also matches.
  std::multiset<std::pair<int, int>> set_A, set_B;
  for (int i = 0; i < surface_A.num_elements(); ++i) {
    set_A.emplace(intersector_A.tet0_of_polygon(i),
                  intersector_A.tet1_of_polygon(i));
    set_B.emplace(intersector_B.tet0_of_polygon(i),
                  intersector_B.tet1_of_polygon(i));
  }

  return set_A == set_B;
}

TEST_F(VolumeIntersectorTest, IntersectFields) {
  const RigidTransformd X_MN = RigidTransformd(0.03 * Vector3d::UnitX());

  // Compute the contact surface using both algorithms provided by
  // VolumeIntersector and check that they produce identical contact surfaces.
  {
    SCOPED_TRACE("Use TriMeshBuilder.");
    std::array<std::unique_ptr<TriangleSurfaceMesh<double>>, 2> surface_01_M;
    std::array<std::unique_ptr<TriangleSurfaceMeshFieldLinear<double, double>>,
               2>
        e_MN_M;
    std::array<VolumeIntersector<TriMeshBuilder<double>, Obb>, 2> intersector;

    intersector[0].IntersectFields(
        box_M_.pressure(), box_M_.bvh(), octahedron_N_.pressure(),
        octahedron_N_.bvh(), X_MN, &surface_01_M[0], &e_MN_M[0]);

    intersector[1].IntersectFields(
        box_M_.pressure(), box_M_.surface_mesh_bvh(), box_M_.tri_to_tet(),
        box_M_.mesh_topology(), octahedron_N_.pressure(),
        octahedron_N_.surface_mesh_bvh(), octahedron_N_.tri_to_tet(),
        octahedron_N_.mesh_topology(), X_MN, &surface_01_M[1], &e_MN_M[1]);

    ASSERT_NE(surface_01_M[0].get(), nullptr);
    ASSERT_NE(surface_01_M[1].get(), nullptr);
    EXPECT_TRUE(ContactSurfacesAreEqual(intersector[0], *surface_01_M[0],
                                        intersector[1], *surface_01_M[1]));
  }
  {
    SCOPED_TRACE("Use PolyMeshBuilder.");
    std::array<std::unique_ptr<PolygonSurfaceMesh<double>>, 2> surface_01_M;
    std::array<std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>>,
               2>
        e_MN_M;
    std::array<VolumeIntersector<PolyMeshBuilder<double>, Obb>, 2> intersector;

    intersector[0].IntersectFields(
        box_M_.pressure(), box_M_.bvh(), octahedron_N_.pressure(),
        octahedron_N_.bvh(), X_MN, &surface_01_M[0], &e_MN_M[0]);

    intersector[1].IntersectFields(
        box_M_.pressure(), box_M_.surface_mesh_bvh(), box_M_.tri_to_tet(),
        box_M_.mesh_topology(), octahedron_N_.pressure(),
        octahedron_N_.surface_mesh_bvh(), octahedron_N_.tri_to_tet(),
        octahedron_N_.mesh_topology(), X_MN, &surface_01_M[1], &e_MN_M[1]);

    EXPECT_NE(surface_01_M[0].get(), nullptr);
    EXPECT_NE(surface_01_M[1].get(), nullptr);
    EXPECT_TRUE(ContactSurfacesAreEqual(intersector[0], *surface_01_M[0],
                                        intersector[1], *surface_01_M[1]));
  }
}

// Smoke tests that AutoDiffXd can build. No checking on the values of
// derivatives or whether the surfaces are equivalent.
TEST_F(VolumeIntersectorTest, IntersectFieldsAutoDiffXd) {
  const math::RigidTransform<AutoDiffXd> X_MN(0.03 *
                                              Vector3<AutoDiffXd>::UnitX());
  {
    SCOPED_TRACE("Use TriMeshBuilder.");
    std::array<std::unique_ptr<TriangleSurfaceMesh<AutoDiffXd>>, 2>
        surface_01_M;
    std::array<
        std::unique_ptr<TriangleSurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>,
        2>
        e_MN_M;
    std::array<VolumeIntersector<TriMeshBuilder<AutoDiffXd>, Obb>, 2>
        intersector;

    intersector[0].IntersectFields(
        box_M_.pressure(), box_M_.bvh(), octahedron_N_.pressure(),
        octahedron_N_.bvh(), X_MN, &surface_01_M[0], &e_MN_M[0]);

    intersector[1].IntersectFields(
        box_M_.pressure(), box_M_.surface_mesh_bvh(), box_M_.tri_to_tet(),
        box_M_.mesh_topology(), octahedron_N_.pressure(),
        octahedron_N_.surface_mesh_bvh(), octahedron_N_.tri_to_tet(),
        octahedron_N_.mesh_topology(), X_MN, &surface_01_M[1], &e_MN_M[1]);

    EXPECT_NE(surface_01_M[0].get(), nullptr);
    EXPECT_NE(surface_01_M[1].get(), nullptr);
  }
  {
    SCOPED_TRACE("Use PolyMeshBuilder.");
    std::array<std::unique_ptr<PolygonSurfaceMesh<AutoDiffXd>>, 2> surface_01_M;
    std::array<
        std::unique_ptr<PolygonSurfaceMeshFieldLinear<AutoDiffXd, AutoDiffXd>>,
        2>
        e_MN_M;
    std::array<VolumeIntersector<PolyMeshBuilder<AutoDiffXd>, Obb>, 2>
        intersector;

    intersector[0].IntersectFields(
        box_M_.pressure(), box_M_.bvh(), octahedron_N_.pressure(),
        octahedron_N_.bvh(), X_MN, &surface_01_M[0], &e_MN_M[0]);

    intersector[1].IntersectFields(
        box_M_.pressure(), box_M_.surface_mesh_bvh(), box_M_.tri_to_tet(),
        box_M_.mesh_topology(), octahedron_N_.pressure(),
        octahedron_N_.surface_mesh_bvh(), octahedron_N_.tri_to_tet(),
        octahedron_N_.mesh_topology(), X_MN, &surface_01_M[1], &e_MN_M[1]);

    EXPECT_NE(surface_01_M[0].get(), nullptr);
    EXPECT_NE(surface_01_M[1].get(), nullptr);
  }
}

// Special case of no intersection. Request PolygonSurfaceMesh<double> as the
// representative template argument.
TEST_F(VolumeIntersectorTest, IntersectFieldsNoIntersection) {
  // 1 meter apart is well separated.
  {
    SCOPED_TRACE("Use VolumeMesh BVH.");
    const RigidTransformd X_MN(Vector3d::UnitX());
    std::unique_ptr<PolygonSurfaceMesh<double>> surface_01_M;
    std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>> e_MN_M;
    VolumeIntersector<PolyMeshBuilder<double>, Obb>().IntersectFields(
        box_M_.pressure(), box_M_.bvh(), octahedron_N_.pressure(),
        octahedron_N_.bvh(), X_MN, &surface_01_M, &e_MN_M);

    EXPECT_EQ(surface_01_M.get(), nullptr);
    EXPECT_EQ(e_MN_M.get(), nullptr);
  }
  {
    SCOPED_TRACE("Use surface TriangleMesh BVH.");
    const RigidTransformd X_MN(Vector3d::UnitX());
    std::unique_ptr<PolygonSurfaceMesh<double>> surface_01_M;
    std::unique_ptr<PolygonSurfaceMeshFieldLinear<double, double>> e_MN_M;
    VolumeIntersector<PolyMeshBuilder<double>, Obb>().IntersectFields(
        box_M_.pressure(), box_M_.surface_mesh_bvh(), box_M_.tri_to_tet(),
        box_M_.mesh_topology(), octahedron_N_.pressure(),
        octahedron_N_.surface_mesh_bvh(), octahedron_N_.tri_to_tet(),
        octahedron_N_.mesh_topology(), X_MN, &surface_01_M, &e_MN_M);

    EXPECT_EQ(surface_01_M.get(), nullptr);
    EXPECT_EQ(e_MN_M.get(), nullptr);
  }
}

TEST_F(VolumeIntersectorTest, IntersectCompliantVolumes) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WN(0.03 * Vector3d::UnitX());
  {
    SCOPED_TRACE("Triangle contact surface with volume BVH.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W;
    HydroelasticVolumeIntersector<TriMeshBuilder<double>>()
        .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                   octahedron_N_, X_WN, &contact_patch_W,
                                   false /* use_surfaces */);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Triangle contact surface with surface BVH.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W;
    HydroelasticVolumeIntersector<TriMeshBuilder<double>>()
        .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                   octahedron_N_, X_WN, &contact_patch_W,
                                   true /* use_surfaces */);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Polygon contact surface with volume BVH.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W;
    HydroelasticVolumeIntersector<PolyMeshBuilder<double>>()
        .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                   octahedron_N_, X_WN, &contact_patch_W,
                                   false /* use_surfaces */);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
  {
    SCOPED_TRACE("Polygon contact surface with surface BVH.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W;
    HydroelasticVolumeIntersector<PolyMeshBuilder<double>>()
        .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                   octahedron_N_, X_WN, &contact_patch_W,
                                   true /* use_surfaces */);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Smoke tests that AutoDiffXd can build. No checking on the values of
// derivatives.
TEST_F(VolumeIntersectorTest, IntersectCompliantVolumesAutoDiffXd) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const math::RigidTransform<AutoDiffXd> X_WM =
      math::RigidTransform<AutoDiffXd>::Identity();
  const math::RigidTransform<AutoDiffXd> X_WN(0.03 *
                                              Vector3<AutoDiffXd>::UnitX());
  {
    SCOPED_TRACE("Triangle contact surface.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W;
    HydroelasticVolumeIntersector<TriMeshBuilder<AutoDiffXd>>()
        .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                   octahedron_N_, X_WN, &contact_patch_W);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Polygon contact surface.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W;
    HydroelasticVolumeIntersector<PolyMeshBuilder<AutoDiffXd>>()
        .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                   octahedron_N_, X_WN, &contact_patch_W);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Special case: no intersection. Request PolygonSurfaceMesh<double> as a
// representative.
TEST_F(VolumeIntersectorTest, IntersectCompliantVolumesNoIntersection) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  // 1 meter makes them well separated.
  const RigidTransformd X_WN(Vector3d::UnitX());

  std::unique_ptr<ContactSurface<double>> contact_patch_W;
  HydroelasticVolumeIntersector<PolyMeshBuilder<double>>()
      .IntersectCompliantVolumes(first_id, box_M_, X_WM, second_id,
                                 octahedron_N_, X_WN, &contact_patch_W);
  EXPECT_EQ(contact_patch_W.get(), nullptr);
}

TEST_F(VolumeIntersectorTest, ComputeContactSurfaceFromCompliantVolumes) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const RigidTransformd X_WM = RigidTransformd::Identity();
  const RigidTransformd X_WN(0.03 * Vector3d::UnitX());
  {
    SCOPED_TRACE("Request triangles.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_M_, X_WM, second_id, octahedron_N_, X_WN,
            HydroelasticContactRepresentation::kTriangle);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Request polygons.");
    std::unique_ptr<ContactSurface<double>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_M_, X_WM, second_id, octahedron_N_, X_WN,
            HydroelasticContactRepresentation::kPolygon);
    ASSERT_NE(contact_patch_W.get(), nullptr);
    EXPECT_EQ(contact_patch_W->representation(),
              HydroelasticContactRepresentation::kPolygon);
  }
}

// Smoke tests that AutoDiffXd can build. No checking on the values of
// derivatives.
TEST_F(VolumeIntersectorTest,
       ComputeContactSurfaceFromCompliantVolumesAutoDiffXd) {
  GeometryId first_id = GeometryId::get_new_id();
  GeometryId second_id = GeometryId::get_new_id();
  const math::RigidTransform<AutoDiffXd> X_WM =
      math::RigidTransform<AutoDiffXd>::Identity();
  const math::RigidTransform<AutoDiffXd> X_WN(0.03 *
                                              Vector3<AutoDiffXd>::UnitX());
  {
    SCOPED_TRACE("Request triangles.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_M_, X_WM, second_id, octahedron_N_, X_WN,
            HydroelasticContactRepresentation::kTriangle);
  }
  {
    SCOPED_TRACE("Request polygons.");
    std::unique_ptr<ContactSurface<AutoDiffXd>> contact_patch_W =
        ComputeContactSurfaceFromCompliantVolumes(
            first_id, box_M_, X_WM, second_id, octahedron_N_, X_WN,
            HydroelasticContactRepresentation::kPolygon);
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
