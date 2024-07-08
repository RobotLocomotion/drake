#include "drake/multibody/plant/hydroelastic_contact_info.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::SurfaceTriangle;
using geometry::TriangleSurfaceMesh;
using math::RigidTransform;

// Creates an arbitrary surface mesh.
std::unique_ptr<TriangleSurfaceMesh<double>> CreateTriangleMesh() {
  std::vector<Vector3<double>> vertices = {
      {0.5, 0.5, -0.5},
      {-0.5, -0.5, -0.5},
      {-0.5, 0.5, -0.5},
  };
  std::vector<SurfaceTriangle> faces;
  faces.emplace_back(0, 1, 2);
  return std::make_unique<TriangleSurfaceMesh<double>>(std::move(faces),
                                                       std::move(vertices));
}

// Creates an arbitrary contact surface between the two given geometries.
std::unique_ptr<ContactSurface<double>> CreateContactSurface(
    GeometryId halfspace_id, GeometryId block_id) {
  auto mesh = CreateTriangleMesh();
  std::vector<double> e_MN(mesh->num_vertices(), 0.0);
  TriangleSurfaceMesh<double>* mesh_pointer = mesh.get();
  return std::make_unique<ContactSurface<double>>(
      halfspace_id, block_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          std::move(e_MN), mesh_pointer));
}

// Returns a distinct spatial force.
SpatialForce<double> MakeSpatialForce() {
  return SpatialForce<double>(Vector3<double>(1, 2, 3),
                              Vector3<double>(4, 5, 6));
}

// Returns a distinct vector (containing a single element) of quadrature point
// data.
std::vector<HydroelasticQuadraturePointData<double>> GetQuadraturePointData() {
  HydroelasticQuadraturePointData<double> data;
  data.p_WQ = Vector3<double>(3.0, 5.0, 7.0);
  data.face_index = 1;
  data.vt_BqAq_W = Vector3<double>(11.0, 13.0, 17.0);
  data.traction_Aq_W = Vector3<double>(19.0, 23.0, 29.0);
  return {data};
}

HydroelasticContactInfo<double> CreateContactInfo(
    std::unique_ptr<ContactSurface<double>>* contact_surface,
    std::unique_ptr<HydroelasticContactInfo<double>>* contact_info) {
  // Create the contact surface using a duplicated arbitrary ID and identity
  // pose; pose and geometry IDs are irrelevant for this test.
  GeometryId arbitrary_id = GeometryId::get_new_id();
  *contact_surface = CreateContactSurface(arbitrary_id, arbitrary_id);

  // Create the HydroelasticContactInfo using particular spatial force and
  // quadrature point data.
  std::vector<HydroelasticQuadraturePointData<double>> quadrature_point_data =
      GetQuadraturePointData();
  return HydroelasticContactInfo<double>(contact_surface->get(),
                                         MakeSpatialForce(),
                                         std::move(quadrature_point_data));
}

// Verifies that the HydroelasticContactInfo structure uses the raw pointer
// and the unique pointer, as appropriate, on copy construction.
GTEST_TEST(HydroelasticContactInfo, CopyConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);

  // Verify that copy construction used the raw pointer.
  EXPECT_EQ(contact_surface.get(), &copy.contact_surface());

  // Copy it again and make sure that the surface is new.
  HydroelasticContactInfo<double> copy2 = copy;
  EXPECT_NE(contact_surface.get(), &copy2.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(copy.F_Ac_W().translational(), MakeSpatialForce().translational());
  EXPECT_EQ(copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());

  // Verify that the quadrature point data was copied.
  EXPECT_EQ(copy.quadrature_point_data(), GetQuadraturePointData());
}

// Verifies that the HydroelasticContactInfo structure transfers ownership of
// the ContactSurface.
GTEST_TEST(HydroelasticContactInfo, MoveConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);
  HydroelasticContactInfo<double> moved_copy = std::move(copy);

  // Verify that the move construction retained the raw pointer.
  EXPECT_EQ(contact_surface.get(), &moved_copy.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(moved_copy.F_Ac_W().translational(),
            MakeSpatialForce().translational());
  EXPECT_EQ(moved_copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());

  // Verify that the quadrature point data was copied.
  EXPECT_EQ(moved_copy.quadrature_point_data(), GetQuadraturePointData());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
