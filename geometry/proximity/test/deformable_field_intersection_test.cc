#include "drake/geometry/proximity/deformable_field_intersection.h"

#include <vector>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

// Translates a deformable mesh into or out of contact.
void Translate(DeformableVolumeMeshWithBvh<double>* deform_W,
               const Vector3d& displacement_W) {
  int num_vertices = deform_W->mesh().num_vertices();
  VectorXd q_W(3 * num_vertices);
  for (int v = 0; v < num_vertices; ++v) {
    q_W.segment<3>(3 * v) = deform_W->mesh().vertex(v) + displacement_W;
  }
  deform_W->UpdateVertexPositions(q_W);
}

class AddDeformableDeformableContactSurfaceTest : public ::testing::Test {
 public:
  AddDeformableDeformableContactSurfaceTest()
      : id0_{GeometryId::get_new_id()},
        // For simplicity, we use a mesh of one tetrahedron that we imagine
        // it is a part of a larger mesh with the boundary surface on the
        // X-Y plane and the interior volume above the X-Y plane.
        //
        //      +Z
        //       |
        //       v3
        //       |
        //       |
        //     v0+------v2---+Y
        //      /
        //     /
        //   v1
        //   /
        // +X
        //
        deform0_W_{VolumeMesh<double>({{0, 1, 2, 3}},
                                      {Vector3d::Zero(), Vector3d::UnitX(),
                                       Vector3d::UnitY(), Vector3d::UnitZ()})},
        // We set the signed-distance value at vertices v0, v1, v2 on the
        // boundary surface to be zero, and set the signed-distance value
        // at vertex v3 in the interior to -1 corresponding to the height
        // of vertex v3 above the boundary surface on X-Y plane.
        signed_distance0_W_{{0, 0, 0, -1}, &deform0_W_.mesh()},
        id1_{GeometryId::get_new_id()},
        // For simplicity, we use a mesh similar to the previous one, but we
        // imagine it is a part of a larger mesh with the interior volume
        // below the X-Y plane.
        //
        //      +Z
        //       |
        //       |
        //     v0+------v2---+Y
        //      /|
        //     / |
        //   v1  v3
        //   /   |
        // +X   -Z
        //
        // The list of vertex indices {1, 0, 2, 3} below is different from the
        // previous mesh, so we can conveniently distinguish them in the test.
        deform1_W_{VolumeMesh<double>(
            {{1, 0, 2, 3}}, {Vector3d::Zero(), Vector3d::UnitX(),
                             Vector3d::UnitY(), -Vector3d::UnitZ()})} {
    deformable_contact_W_.RegisterDeformableGeometry(
        id0_, deform0_W_.mesh().num_vertices());
    deformable_contact_W_.RegisterDeformableGeometry(
        id1_, deform1_W_.mesh().num_vertices());
  }

 protected:
  const GeometryId id0_;
  // The first deformable mesh is fixed.
  const DeformableVolumeMeshWithBvh<double> deform0_W_;
  const VolumeMeshFieldLinear<double, double> signed_distance0_W_;
  const GeometryId id1_;
  // The second deformable mesh will make contact or not depending on the test.
  // After deformation, we will create its signed distance field.
  DeformableVolumeMeshWithBvh<double> deform1_W_;
  DeformableContact<double> deformable_contact_W_;
};

TEST_F(AddDeformableDeformableContactSurfaceTest, NoContact) {
  // Translate the second deformable mesh out of contact.
  Translate(&deform1_W_, Vector3d(0, 0, -0.2));
  // Create the signed distance field after deformation. The actual values
  // do not matter because there is no contact.
  VolumeMeshFieldLinear<double, double> sdf1_W{{0, 0, 0, -1},
                                               &deform1_W_.mesh()};

  AddDeformableDeformableContactSurface(sdf1_W, deform1_W_, id1_,
                                        signed_distance0_W_, deform0_W_, id0_,
                                        &deformable_contact_W_);

  EXPECT_EQ(deformable_contact_W_.contact_surfaces().size(), 0);
}

TEST_F(AddDeformableDeformableContactSurfaceTest, HaveContact) {
  ASSERT_EQ(deformable_contact_W_.contact_surfaces().size(), 0);
  // Translate the second deformable mesh into contact.
  Translate(&deform1_W_, Vector3d(0, 0, 0.2));
  // Create the signed distance field after deformation. Assign the first
  // three vertices the value of zero because they are on the boundary
  // surface. Assign the last vertex the value -1 because it is in
  // the interior volume.
  VolumeMeshFieldLinear<double, double> sdf1_W{{0, 0, 0, -1},
                                               &deform1_W_.mesh()};
  ASSERT_TRUE(id0_ < id1_);
  AddDeformableDeformableContactSurface(sdf1_W, deform1_W_, id1_,
                                        signed_distance0_W_, deform0_W_, id0_,
                                        &deformable_contact_W_);

  // Check ContactParticipation. It corresponds to the identity permutation
  // because all vertices participate in contact.
  {
    EXPECT_EQ(deformable_contact_W_.contact_participation(id0_)
                  .CalcPartialPermutation()
                  .vertex()
                  .permutation(),
              std::vector<int>({0, 1, 2, 3}));
    EXPECT_EQ(deformable_contact_W_.contact_participation(id1_)
                  .CalcPartialPermutation()
                  .vertex()
                  .permutation(),
              std::vector<int>({0, 1, 2, 3}));
  }

  // Check DeformableContactSurface.
  {
    ASSERT_EQ(deformable_contact_W_.contact_surfaces().size(), 1);
    const DeformableContactSurface<double>& contact_surface =
        deformable_contact_W_.contact_surfaces().at(0);
    ASSERT_EQ(contact_surface.id_A(), id0_);
    ASSERT_EQ(contact_surface.id_B(), id1_);
    EXPECT_EQ(contact_surface.num_contact_points(), 1);
    // We can use contact_mesh_W() without verification because
    // AddDeformableDeformableContactSurface() uses VolumeIntersector
    // to compute the surface mesh and adds data specific to
    // DeformableContact afterward. The correctness of the contact mesh
    // is already tested in VolumeIntersector.
    EXPECT_EQ(contact_surface.contact_points_W().at(0),
              contact_surface.contact_mesh_W().element_centroid(0));
    EXPECT_EQ(
        contact_surface.signed_distances().at(0),
        sdf1_W.EvaluateCartesian(0, contact_surface.contact_points_W().at(0)));
    EXPECT_EQ(contact_surface.tet_barycentric_coordinates_A().at(0),
              deform0_W_.mesh().CalcBarycentric(
                  contact_surface.contact_points_W().at(0), 0));
    EXPECT_EQ(contact_surface.tet_contact_vertex_indexes_A().at(0),
              Vector4<int>(0, 1, 2, 3));
    EXPECT_EQ(contact_surface.barycentric_coordinates_B().at(0),
              deform1_W_.mesh().CalcBarycentric(
                  contact_surface.contact_points_W().at(0), 0));
    EXPECT_EQ(contact_surface.contact_vertex_indexes_B().at(0),
              Vector4<int>(1, 0, 2, 3));
    // Contact normal points downward *out of* geometry B (id1_) *into*
    // geometry A (id0_).
    EXPECT_EQ(contact_surface.nhats_W().at(0), Vector3d::UnitZ());
    EXPECT_TRUE(contact_surface.is_B_deformable());
  }
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
