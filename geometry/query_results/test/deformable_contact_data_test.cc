#include "drake/geometry/query_results/deformable_contact_data.h"

#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/fem/mesh_utilities.h"

namespace drake {
namespace geometry {
namespace {

using Eigen::Vector3d;
using internal::Bvh;
using internal::DeformableVolumeMesh;
using internal::Obb;
using internal::deformable::ComputeTetMeshTriMeshContact;
using internal::deformable::DeformableContactSurface;
using internal::deformable::DeformableGeometry;
using internal::deformable::DeformableRigidContactPair;
using multibody::fem::MakeOctahedronVolumeMesh;
using std::move;
using std::vector;

const GeometryId kDeformableGeometryId = GeometryId::get_new_id();
const GeometryId kRigidGeometryId = GeometryId::get_new_id();

/* The MakePyramidSurface() method is stolen from mesh_intersection_test.cc. */

/* Generates a simple surface mesh of a pyramid with vertices on the
 coordinate axes and the origin like this:

                +Z   -X
                 |   /
              v5 ●  ● v3
                 | /
        v4    v0 |/
  -Y-----●-------●------●---+Y
                /      v2
               /
              ● v1
             /
           +X
*/
template <typename T>
TriangleSurfaceMesh<T> MakePyramidSurface() {
  const int face_data[8][3] = {// The top four faces share the apex vertex v5.
                               {1, 2, 5},
                               {2, 3, 5},
                               {3, 4, 5},
                               {4, 1, 5},
                               // The bottom four faces share the origin v0.
                               {4, 3, 0},
                               {3, 2, 0},
                               {2, 1, 0},
                               {1, 4, 0}};
  vector<geometry::SurfaceTriangle> faces;
  for (auto& face : face_data) {
    faces.emplace_back(face);
  }
  // clang-format off
  vector<Vector3<T>> vertices = {
      { 0,  0, 0},
      { 1,  0, 0},
      { 0,  1, 0},
      {-1,  0, 0},
      { 0, -1, 0},
      { 0,  0, 1}
  };
  // clang-format on
  return TriangleSurfaceMesh<T>(move(faces), move(vertices));
}

/* Returns the contact surface between the rigid pyramid surface (see
 MakePyramidSurface()) and the deforamble octahedron volume (see
 MakeOctahedronVolumeMesh()) where the pose of the rigid mesh in the deformable
 mesh's frame is given by X_DR. */
DeformableContactSurface<double> MakeDeformableContactSurface(
    const math::RigidTransformd& X_DR) {
  const DeformableVolumeMesh<double> volume_D(
      MakeOctahedronVolumeMesh<double>());
  /* Deformable contact assumes the rigid surface is double-valued, regardless
   of the scalar value for the volume mesh.  */
  const TriangleSurfaceMesh<double> surface_R = MakePyramidSurface<double>();
  const Bvh<Obb, TriangleSurfaceMesh<double>> bvh_R(surface_R);
  return ComputeTetMeshTriMeshContact<double>(volume_D, surface_R, bvh_R, X_DR);
}

/* Makes a DeformableContactData with a single contact pair using the given
 `contact_surface`. Unused parameters are set to arbitrary values. */
DeformableContactData<double> MakeDeformableContactData(
    DeformableContactSurface<double> contact_surface,
    const DeformableGeometry& deformable_geometry) {
  DeformableRigidContactPair<double> contact_pair(
      move(contact_surface), kRigidGeometryId, kDeformableGeometryId);
  vector<DeformableRigidContactPair<double>> contact_pairs;
  contact_pairs.push_back(move(contact_pair));
  return DeformableContactData<double>(move(contact_pairs),
                                       deformable_geometry);
}

DeformableGeometry MakeOctahedronDeformableGeometry() {
  VolumeMesh<double> volume_mesh = MakeOctahedronVolumeMesh<double>();
  /* We use a sphere to approximate the shape of the tetrahedron. In fact, the
   shape is only used to calculate the sign distance, and the sphere correctly
   provides the sign-distance for all vertices in the octahedron. (The interior
   vertex has distance 1, and all other vertices have distance 0). */
  const Sphere sphere(1.0);
  return DeformableGeometry(sphere, move(volume_mesh));
}

GTEST_TEST(DeformableContactDataTest, DeformableContactData) {
  /* Recall that the octahedron looks like.
                  +Z   -X
                   |   /
                v5 ●  ● v3
                   | /
         v4     v0 |/
    -Y----●--------●------●----+Y
                  /|      v2
                 / |
             v1 ●  ● v6
               /   |
             +X    |
                  -Z
   Rotate the the rigid pyramid around x-axis by -90 degrees (right-handed) and
   then shift it along the positive y-axis so that only its square base
   intersects the right pyramidal region of the deformable octahedron. As a
   result, all vertices except v4 are participating in contact. */
  const auto X_DR = math::RigidTransformd(
      math::RotationMatrix<double>::MakeXRotation(-M_PI / 2),
      Vector3<double>(0, 0.5, 0));
  DeformableContactSurface<double> contact_surface =
      MakeDeformableContactSurface(X_DR);
  const DeformableContactData<double> contact_data = MakeDeformableContactData(
      move(contact_surface), MakeOctahedronDeformableGeometry());

  EXPECT_EQ(contact_data.num_contact_pairs(), 1);
  /* v0, v1, v2, v3, v5, v6 are participating in contact so they get new indexes
   0, 1, 2, 3, 4, 5. v4 is not participating in contact and gets new index 6. */
  EXPECT_EQ(contact_data.num_vertices_in_contact(), 6);
  const std::vector<int> permuted_vertex_indexes = {0, 1, 2, 3, 6, 4, 5};
  EXPECT_EQ(contact_data.permuted_vertex_indexes(), permuted_vertex_indexes);
  /* Verify that permuted_vertex_indexes() and permuted_to_original_indexes()
   are inverses to each other. */
  const std::vector<int>& permuted_to_original_indexes =
      contact_data.permuted_to_original_indexes();
  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(i, permuted_vertex_indexes[permuted_to_original_indexes[i]]);
    EXPECT_EQ(i, permuted_to_original_indexes[permuted_vertex_indexes[i]]);
  }
  EXPECT_EQ(contact_data.deformable_geometry_id(), kDeformableGeometryId);
}

GTEST_TEST(DeformableContactDataTest, EmptyDeformableContactData) {
  /* Move the rigid pyramid way down, so there is no contact. */
  const auto X_DR = math::RigidTransformd(Vector3<double>(0, 0, -15));
  DeformableContactSurface<double> contact_surface =
      MakeDeformableContactSurface(X_DR);
  const DeformableContactData<double> contact_data = MakeDeformableContactData(
      move(contact_surface), MakeOctahedronDeformableGeometry());

  EXPECT_EQ(contact_data.num_contact_pairs(), 1);
  EXPECT_EQ(contact_data.num_vertices_in_contact(), 0);
  const std::vector<int> vertex_indexes = {0, 1, 2, 3, 4, 5, 6};
  EXPECT_EQ(contact_data.permuted_vertex_indexes(), vertex_indexes);
  EXPECT_EQ(contact_data.permuted_to_original_indexes(), vertex_indexes);
}

}  // namespace
}  // namespace geometry
}  // namespace drake
