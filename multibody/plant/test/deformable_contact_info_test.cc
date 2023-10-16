#include "drake/multibody/plant/deformable_contact_info.h"

#include <gtest/gtest.h>

using drake::geometry::GeometryId;
using drake::geometry::PolygonSurfaceMesh;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace {

GTEST_TEST(DeformableContactInfo, ConstructAndAccess) {
  std::vector<Vector3d> vertices = {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  std::vector<int> face_data{3, 0, 1, 2, 3, 2, 3, 0};
  const PolygonSurfaceMesh<double> contact_mesh(std::move(face_data),
                                                std::move(vertices));
  GeometryId id_A = GeometryId::get_new_id();
  GeometryId id_B = GeometryId::get_new_id();
  const SpatialForce<double> F_Ac_W(Vector3d(1, 2, 3), Vector3d(4, 5, 6));
  std::vector<DeformableContactPointData<double>> contact_point_data;
  contact_point_data.emplace_back(Vector3d(11, 22, 33), 42,
                                  Vector3d(44, 55, 66), Vector3d(77, 88, 99));
  const DeformableContactInfo<double> dut(id_A, id_B, contact_mesh, F_Ac_W,
                                          contact_point_data);

  EXPECT_EQ(dut.id_A(), id_A);
  EXPECT_EQ(dut.id_B(), id_B);
  EXPECT_TRUE(dut.contact_mesh().Equal(contact_mesh));
  EXPECT_EQ(dut.F_Ac_W().translational(), F_Ac_W.translational());
  EXPECT_EQ(dut.F_Ac_W().rotational(), F_Ac_W.rotational());
  EXPECT_EQ(dut.contact_point_data(), contact_point_data);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
