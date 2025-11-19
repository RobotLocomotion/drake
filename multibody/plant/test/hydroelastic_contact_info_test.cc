#include "drake/multibody/plant/hydroelastic_contact_info.h"

#include <memory>
#include <utility>
#include <vector>

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

// Creates an arbitrary surface mesh.
std::unique_ptr<TriangleSurfaceMesh<double>> MakeTriangleMesh() {
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
std::unique_ptr<ContactSurface<double>> MakeContactSurface() {
  GeometryId arbitrary_id = GeometryId::get_new_id();
  auto mesh = MakeTriangleMesh();
  std::vector<double> e_MN(mesh->num_vertices(), 0.0);
  TriangleSurfaceMesh<double>* mesh_pointer = mesh.get();
  return std::make_unique<ContactSurface<double>>(
      arbitrary_id, arbitrary_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(
          std::move(e_MN), mesh_pointer));
}

// Returns a distinct spatial force.
SpatialForce<double> MakeSpatialForce() {
  return SpatialForce<double>(Vector3<double>(1, 2, 3),
                              Vector3<double>(4, 5, 6));
}

GTEST_TEST(HydroelasticContactInfo, BarePointerConstruction) {
  // When passed a bare pointer surface, it is kept without any copying.
  auto original_surface = MakeContactSurface();
  HydroelasticContactInfo<double> dut(original_surface.get(),
                                      MakeSpatialForce());
  ASSERT_EQ(&dut.contact_surface(), original_surface.get());

  // Self-assigning does not copy.
  {
    const auto& other = dut;
    dut = other;
  }
  ASSERT_EQ(&dut.contact_surface(), original_surface.get());

  // Copying the dut causes the surface to be copied as well.
  HydroelasticContactInfo<double> copy1(dut);
  ASSERT_NE(&copy1.contact_surface(), original_surface.get());

  // Copying again shares the surface, no extra copying.
  HydroelasticContactInfo<double> copy2(copy1);
  ASSERT_EQ(&copy2.contact_surface(), &copy1.contact_surface());

  // Copy-assign shares the surface, no extra copying.
  HydroelasticContactInfo<double> copy_assign(MakeContactSurface(),
                                              SpatialForce<double>::Zero());
  copy_assign = copy2;
  ASSERT_EQ(&copy_assign.contact_surface(), &copy1.contact_surface());

  // Verify that the spatial force makes it all the way through.
  EXPECT_EQ(copy_assign.F_Ac_W().translational(),
            MakeSpatialForce().translational());
  EXPECT_EQ(copy_assign.F_Ac_W().rotational(), MakeSpatialForce().rotational());
}

GTEST_TEST(HydroelasticContactInfo, UniquePtrConstruction) {
  // When passed a unique_ptr surface, it is kept without any copying.
  auto temp_surface = MakeContactSurface();
  const ContactSurface<double>* const original_surface = temp_surface.get();
  HydroelasticContactInfo<double> dut(std::move(temp_surface),
                                      MakeSpatialForce());
  ASSERT_EQ(&dut.contact_surface(), original_surface);

  // Copying still shares the original surface.
  HydroelasticContactInfo<double> copy1(dut);
  ASSERT_EQ(&copy1.contact_surface(), original_surface);

  // Copying again still shares the original surface.
  HydroelasticContactInfo<double> copy2(copy1);
  ASSERT_EQ(&copy2.contact_surface(), original_surface);

  // Copying again still shares the surface, no extra copying.
  HydroelasticContactInfo<double> copy_assign(MakeContactSurface(),
                                              SpatialForce<double>::Zero());
  copy_assign = copy2;
  ASSERT_EQ(&copy_assign.contact_surface(), original_surface);

  // Verify that the spatial force make it all the way through.
  EXPECT_EQ(copy_assign.F_Ac_W().translational(),
            MakeSpatialForce().translational());
  EXPECT_EQ(copy_assign.F_Ac_W().rotational(), MakeSpatialForce().rotational());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
