#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/proximity_utilities.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

namespace {

using Eigen::Vector3d;
using geometry::Box;
using geometry::VolumeElement;
using geometry::VolumeMesh;
using geometry::VolumeVertex;
using geometry::VolumeVertexIndex;
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
  for (VolumeVertexIndex i(0); i < num_vertices; ++i) {
    for (VolumeVertexIndex j(i + 1); j < num_vertices; ++j) {
      const bool vertex_is_unique =
          mesh.vertex(i).r_MV() != mesh.vertex(j).r_MV();
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
        const VolumeVertex<double> corner(X_WB * Vector3d(x, y, z));
        const bool corner_is_a_mesh_vertex =
            mesh.vertices().end() !=
            find_if(mesh.vertices().begin(), mesh.vertices().end(),
                    [&corner](const VolumeVertex<double>& v) -> bool {
                      return v.r_MV() == corner.r_MV();
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
  for (VolumeVertexIndex i(0); i < num_vertices; ++i) {
    const bool vertex_is_inside_or_on_boundary =
        ((X_WB.inverse() * mesh.vertex(i).r_MV()).array().abs() <=
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

GTEST_TEST(MeshUtilitiesTest, MakeDiamondCubicBoxVolumeMesh) {
  const double dx = 0.1;
  const RigidTransform<double> X_WB(Eigen::Quaternion<double>(1, 2, 3, 4),
                                    Vector3d(1, 2, 3));
  const Box cube(Box::MakeCube(2 * dx));
  const VolumeMesh<double> cube_mesh =
      MakeDiamondCubicBoxVolumeMesh<double>(cube, dx, X_WB);
  /* 2 x 2 x 2 = 8 cubes. Each cube is split into 5 tetrahedrons. */
  EXPECT_EQ(cube_mesh.num_elements(), 8 * 5);
  /* 3 x 3 x 3 = 27 vertices. From the cubes. No additional vertex is introduced
    while splitting into tetrahedra. */
  EXPECT_EQ(cube_mesh.num_vertices(), 27);
  /* Verify that the mesh is conforming and conforms to the box. */
  EXPECT_TRUE(VerifyDiamondCubicBoxMesh(cube_mesh, cube, X_WB));

  const Box rectangle(1.2 * dx, 2.3 * dx, 3.5 * dx);
  const VolumeMesh<double> rectangle_mesh =
      MakeDiamondCubicBoxVolumeMesh<double>(rectangle, dx, X_WB);
  /* 2 x 3 x 4 = 24 cubes. Each cube is split into 5 tetrahedrons. */
  EXPECT_EQ(rectangle_mesh.num_elements(), 120);
  /* 3 x 4 x 5 = 60 vertices. From the cubes. No additional vertex is introduced
    while splitting into tetrahedra. */
  EXPECT_EQ(rectangle_mesh.num_vertices(), 60);
  /* Verify that the mesh is conforming and conforms to the box. */
  EXPECT_TRUE(VerifyDiamondCubicBoxMesh(rectangle_mesh, rectangle, X_WB));
}
}  // namespace
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
