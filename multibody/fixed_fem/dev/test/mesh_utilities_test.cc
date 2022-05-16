#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/mesh_to_vtk.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace multibody {
namespace fem {

namespace {

using Eigen::Vector3d;
using geometry::Box;
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeMeshFieldLinear;
using geometry::internal::ComputeEulerCharacteristic;
using math::RigidTransform;
using std::abs;
using std::find_if;
// TODO(xuchenhan-tri): The following test method is taken and modified from
// geometry::proximity::internal::MakeBoxVolumeMeshWithMaTest. Remove the
// redundent code path when this goes out of dev/.

// Verifies that a tetrahedral
// mesh of a box from MakeDiamondCubicBoxVolumeMesh() satisfies all these
// properties:
//
//   A. The mesh is conforming.
//   B. The mesh conforms to the box.
//
// @retval true when all conditions passed.
bool VerifyDiamondCubicBoxMesh(const VolumeMesh<double>& mesh, const Box& box,
                               const RigidTransform<double>& X_WB) {
  // A. The mesh is conforming.
  // A1. The mesh has unique vertices.
  const int num_vertices = mesh.num_vertices();
  for (int i = 0; i < num_vertices; ++i) {
    for (int j = i + 1; j < num_vertices; ++j) {
      const bool vertex_is_unique = mesh.vertex(i) != mesh.vertex(j);
      EXPECT_TRUE(vertex_is_unique) << "The mesh has duplicated vertices.";
      if (!vertex_is_unique) {
        return false;
      }
    }
  }
  // A2. Euler characteristic = 1. This is a necessary condition (but may not
  //     be sufficient) for conforming tetrahedra (two tetrahedra intersect in
  //     their shared face, or shared edge, or shared vertex, or not at all.
  //     There is no partial overlapping of two tetrahedra.).
  const int euler_characteristic = ComputeEulerCharacteristic(mesh);
  EXPECT_EQ(1, euler_characteristic)
      << "The mesh's tetrahedra are not conforming because the mesh's Euler "
         "characteristic is "
      << euler_characteristic << " instead of 1.";
  if (euler_characteristic != 1) {
    return false;
  }

  // B. The mesh conforms to the box.
  // B1. The mesh has a vertex at each of the 8 corner points of the box.
  const Vector3d half_size = box.size() / 2.;
  for (const double x : {-half_size.x(), half_size.x()}) {
    for (const double y : {-half_size.y(), half_size.y()}) {
      for (const double z : {-half_size.z(), half_size.z()}) {
        const Vector3d corner(X_WB * Vector3d(x, y, z));
        const bool corner_is_a_mesh_vertex =
            mesh.vertices().end() !=
            find_if(mesh.vertices().begin(), mesh.vertices().end(),
                    [&corner](const Vector3d& v) -> bool {
                      return v == corner;
                    });
        EXPECT_TRUE(corner_is_a_mesh_vertex)
            << "A corner point of the box is missing from the mesh vertices.";
        if (!corner_is_a_mesh_vertex) {
          return false;
        }
      }
    }
  }
  // B2. No mesh's vertex is outside the box.
  // TODO(xuchenhan-tri): The epsilon here should be tuned to the condition
  // number of the rigid transform.
  const double epsilon = 1e-14;
  for (int i = 0; i < num_vertices; ++i) {
    const bool vertex_is_inside_or_on_boundary =
        ((X_WB.inverse() * mesh.vertex(i)).array().abs() <=
         (1 + epsilon) * half_size.array())
            .all();
    EXPECT_TRUE(vertex_is_inside_or_on_boundary)
        << "A mesh vertex is outside the box.";
    if (!vertex_is_inside_or_on_boundary) {
      return false;
    }
  }
  // B3. The volume of the mesh equals the volume of the box.
  const double mesh_volume = mesh.CalcVolume();
  const double box_volume = box.width() * box.depth() * box.height();
  // We estimate the tolerance from:
  // 1. There are about 4 multiply+add in calculating a tetrahedron's volume,
  //    so we have 8 epsilons.
  // 2. The rounding errors accumulate over the number of tetrahedra.
  const double volume_tolerance(mesh.num_elements() * 8.0 *
                                std::numeric_limits<double>::epsilon());
  EXPECT_NEAR(mesh_volume, box_volume, volume_tolerance)
      << "The mesh's volume does not equal the box's volume.";
  if (abs(mesh_volume - box_volume) > volume_tolerance) {
    return false;
  }
  return true;
}

GTEST_TEST(MeshUtilitiesTest, DiamondCubicBoxVolumeMesh) {
  const double dx = 0.1;
  const RigidTransform<double> X_WB(Eigen::Quaternion<double>(1, 2, 3, 4),
                                    Vector3d(1, 2, 3));
  const Box cube(Box::MakeCube(2 * dx));
  const internal::ReferenceDeformableGeometry<double> cube_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(cube, dx, X_WB);
  const VolumeMesh<double>& cube_mesh = cube_geometry.mesh();
  /* 2 x 2 x 2 = 8 cubes. Each cube is split into 5 tetrahedrons. */
  EXPECT_EQ(cube_mesh.num_elements(), 8 * 5);
  /* 3 x 3 x 3 = 27 vertices. From the cubes. No additional vertex is introduced
    while splitting into tetrahedra. */
  EXPECT_EQ(cube_mesh.num_vertices(), 27);
  /* Verify that the mesh is conforming and conforms to the box. */
  EXPECT_TRUE(VerifyDiamondCubicBoxMesh(cube_mesh, cube, X_WB));

  const Box rectangle(1.2 * dx, 2.3 * dx, 3.5 * dx);
  const internal::ReferenceDeformableGeometry<double> rectangle_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(rectangle, dx, X_WB);
  const VolumeMesh<double>& rectangle_mesh = rectangle_geometry.mesh();
  /* 2 x 3 x 4 = 24 cubes. Each cube is split into 5 tetrahedrons. */
  EXPECT_EQ(rectangle_mesh.num_elements(), 120);
  /* 3 x 4 x 5 = 60 vertices. From the cubes. No additional vertex is introduced
    while splitting into tetrahedra. */
  EXPECT_EQ(rectangle_mesh.num_vertices(), 60);
  /* Verify that the mesh is conforming and conforms to the box. */
  EXPECT_TRUE(VerifyDiamondCubicBoxMesh(rectangle_mesh, rectangle, X_WB));
}

/* Verifies that the signed distance field is correct at all vertex locations.
 */
GTEST_TEST(MeshUtilitiesTest, SignedDistanceField) {
  constexpr double dx = 0.1;
  constexpr double half_Lx = 1.1 * dx;
  constexpr double half_Ly = 1.2 * dx;
  constexpr double half_Lz = 2.3 * dx;
  const Box box(2.0 * half_Lx, 2.0 * half_Ly, 2.0 * half_Lz);
  const internal::ReferenceDeformableGeometry<double> box_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(box, dx,
                                                    math::RigidTransformd());
  const VolumeMesh<double>& mesh = box_geometry.mesh();
  const VolumeMeshFieldLinear<double, double>& mesh_field =
      box_geometry.signed_distance();
  for (int i = 0; i < mesh.num_vertices(); ++i) {
    const double signed_distance = mesh_field.EvaluateAtVertex(i);
    const Vector3d& r_WV = mesh.vertex(i);
    // clang-format off
    const std::array<double, 6> distance_to_box_faces = {
        half_Lx - r_WV(0), r_WV(0) + half_Lx,
        half_Ly - r_WV(1), r_WV(1) + half_Ly,
        half_Lz - r_WV(2), r_WV(2) + half_Lz };
    // clang-format on
    const double distance_to_surface = *std::min_element(
        std::begin(distance_to_box_faces), std::end(distance_to_box_faces));
    EXPECT_DOUBLE_EQ(-distance_to_surface, signed_distance);
  }
}

GTEST_TEST(MeshUtilitiesTest, StarRefineBoundaryTetrahedra) {
  // Refine one tetrahedron into four tetrahedra.
  {
    const VolumeMesh<double> one_element_mesh(
        std::vector<VolumeElement>{{0, 1, 2, 3}},
        std::vector<Vector3d>{Vector3d::Zero(), Vector3d::UnitX(),
                              Vector3d::UnitY(), Vector3d::UnitZ()});
    ASSERT_EQ(one_element_mesh.num_elements(), 1);
    ASSERT_EQ(one_element_mesh.num_vertices(), 4);

    const VolumeMesh<double> refined_mesh =
        StarRefineBoundaryTetrahedra(one_element_mesh);

    EXPECT_EQ(refined_mesh.num_elements(), 4);
    EXPECT_EQ(refined_mesh.num_vertices(), 5);
  }

  // All tetrahedra of the octahedron mesh are non-boundary tetrahedra, so we
  // expect no change to the mesh.
  {
    const VolumeMesh<double> octahedron_mesh =
        MakeOctahedronVolumeMesh<double>();

    const VolumeMesh<double> no_refine_mesh =
        StarRefineBoundaryTetrahedra(octahedron_mesh);

    EXPECT_EQ(no_refine_mesh.num_elements(), octahedron_mesh.num_elements());
    EXPECT_EQ(no_refine_mesh.num_vertices(), octahedron_mesh.num_vertices());
  }

  // A coarse tetrahedral mesh of a cube consists of 6 tethedra, all of which
  // are boundary tetrahedra.
  {
    const Box cube_10cm = Box::MakeCube(0.1);
    const double resolution_hint_20cm = 0.2;
    const VolumeMesh<double> cube_mesh =
        geometry::internal::MakeBoxVolumeMesh<double>(cube_10cm,
                                                      resolution_hint_20cm);
    ASSERT_EQ(cube_mesh.num_elements(), 6);
    ASSERT_EQ(cube_mesh.num_vertices(), 8);

    const VolumeMesh<double> refined_cube_mesh =
        StarRefineBoundaryTetrahedra(cube_mesh);

    // The star-refinement is expected to create 4 x 6 = 24 tetrahedra.
    EXPECT_EQ(refined_cube_mesh.num_elements(), 24);
    // The star-refinement adds as many vertices as the number of boundary
    // tetrahedra, so we have 8 (original) + 6 (new) = 14 vertices.
    EXPECT_EQ(refined_cube_mesh.num_vertices(), 14);
  }
}

// TODO(DamrongGuoy): Remove the following unit tests. For now, it's ok to be
//  in `dev` directory, but it shouldn't go out of `dev`.

// Verification by Visualization (another kind of V&V, not the standard
// Verification and Validation. 8)
// 1. bazel build //multibody/fixed_fem/dev:mesh_utilities_test
// 2. bazel-bin/multibody/fixed_fem/dev/mesh_utilities_test to get *.vtk files.
// 3. paraview, Load Data: *.vtk
GTEST_TEST(MeshUtilitiesTest, VisuallyVerifyStarRefineBoundaryTetrahedra) {
  constexpr double dx = 0.1;
  constexpr double half_Lx = 1.1 * dx;
  constexpr double half_Ly = 1.2 * dx;
  constexpr double half_Lz = 2.3 * dx;
  const Box box(2.0 * half_Lx, 2.0 * half_Ly, 2.0 * half_Lz);
  const internal::ReferenceDeformableGeometry<double> box_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(box, dx,
                                                    math::RigidTransformd());
  {
    const std::string file_name("box_signed_distance.vtk");
    geometry::internal::WriteVolumeMeshFieldLinearToVtk(
        file_name, "Approximated signed distance [meter]",
        box_geometry.signed_distance(),
        "Signed distance function in " + file_name);
  }

  const VolumeMesh<double> refined_box_mesh =
      StarRefineBoundaryTetrahedra(box_geometry.mesh());

  // The max field value is the half the minimal dimension of the box,
  // so that the field will mimic the "unsigned" distance-to-boundary function.
  const double field_value_bound = half_Lx;
  VolumeMeshFieldLinear<double, double> refined_box_positive_distance_function =
      geometry::internal::MakeBoxPressureField<double>(
          box, &refined_box_mesh, field_value_bound);
  {
    const std::string file_name("refined_box_positive_distance.vtk");
    geometry::internal::WriteVolumeMeshFieldLinearToVtk(
        file_name, "Positive distance [meter]",
        refined_box_positive_distance_function,
        "Positive distance function in " + file_name);
  }
}

}  // namespace
}  // namespace fem
}  // namespace multibody
}  // namespace drake
