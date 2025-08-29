#include "drake/geometry/internal_geometry.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::make_unique;

// Create two instances of the given Properties type with different properties:
// ('group1', 'value') in the first, ('group2', 'value') in the second.
template <typename Properties>
std::pair<Properties, Properties> MakePropertyPair() {
  Properties p1;
  p1.AddProperty("group1", "value", 1);
  Properties p2;
  p2.AddProperty("group2", "value", 2);
  return std::make_pair(p1, p2);
}

// Confirms that properties get set properly.
GTEST_TEST(InternalGeometryTest, PropertyAssignment) {
  InternalGeometry geometry;

  {
    // Proximity.
    EXPECT_FALSE(geometry.has_proximity_role());
    const auto [p1, p2] = MakePropertyPair<ProximityProperties>();
    DRAKE_EXPECT_NO_THROW(geometry.SetRole(p1));
    EXPECT_TRUE(geometry.has_proximity_role());
    EXPECT_TRUE(
        geometry.proximity_properties()->HasProperty("group1", "value"));
    EXPECT_FALSE(
        geometry.proximity_properties()->HasProperty("group2", "value"));
    // Resetting proximity properties is not an error.
    DRAKE_EXPECT_NO_THROW(geometry.SetRole(p2));
    EXPECT_FALSE(
        geometry.proximity_properties()->HasProperty("group1", "value"));
    EXPECT_TRUE(
        geometry.proximity_properties()->HasProperty("group2", "value"));
    EXPECT_EQ(geometry.proximity_properties(),
              geometry.properties(Role::kProximity));
  }

  {
    // Illustration.
    EXPECT_FALSE(geometry.has_illustration_role());
    const auto [p1, p2] = MakePropertyPair<IllustrationProperties>();
    DRAKE_EXPECT_NO_THROW(geometry.SetRole(p1));
    EXPECT_TRUE(geometry.has_illustration_role());
    EXPECT_TRUE(
        geometry.illustration_properties()->HasProperty("group1", "value"));
    EXPECT_FALSE(
        geometry.illustration_properties()->HasProperty("group2", "value"));
    // Resetting illustration properties is not an error.
    DRAKE_EXPECT_NO_THROW(geometry.SetRole(p2));
    EXPECT_FALSE(
        geometry.illustration_properties()->HasProperty("group1", "value"));
    EXPECT_TRUE(
        geometry.illustration_properties()->HasProperty("group2", "value"));
    EXPECT_EQ(geometry.illustration_properties(),
              geometry.properties(Role::kIllustration));
  }

  {
    // Perception.
    EXPECT_FALSE(geometry.has_perception_role());
    DRAKE_EXPECT_NO_THROW(geometry.SetRole(PerceptionProperties()));
    EXPECT_TRUE(geometry.has_perception_role());
    // Cannot yet overwrite perception properties.
    DRAKE_EXPECT_THROWS_MESSAGE(
        geometry.SetRole(PerceptionProperties()),
        "Geometry already has perception role assigned");
    EXPECT_TRUE(geometry.has_perception_role());
    EXPECT_EQ(geometry.perception_properties(),
              geometry.properties(Role::kPerception));
  }
}

GTEST_TEST(InternalGeometryTest, SetShape) {
  InternalGeometry geometry;

  // Default constructor has no shape at all.
  if constexpr (!kDrakeAssertIsArmed) {
    // When assert is armed, the call to shape() throws because
    // copyable_unique_ptr rejects dereferencing a null pointer.
    ASSERT_EQ(&geometry.shape(), nullptr);
  }

  // Set it to a couple of arbitrary shapes to confirm the change registers.
  const Sphere s(1.5);
  geometry.SetShape(s);
  EXPECT_EQ(s.type_name(), geometry.shape().type_name());

  const Box b(1, 2, 3);
  geometry.SetShape(b);
  EXPECT_EQ(b.type_name(), geometry.shape().type_name());
}

GTEST_TEST(InternalGeometryTest, SetPose) {
  const SourceId source_id = SourceId::get_new_id();
  const Sphere sphere(1.5);
  const FrameId frame_id = FrameId::get_new_id();
  const GeometryId geometry_id = GeometryId::get_new_id();
  const std::string name("geometry");

  const RigidTransformd X_FGold(RotationMatrixd::MakeXRotation(M_PI / 3),
                                Vector3d(1, 2, 3));
  const RigidTransformd X_FGNew(RotationMatrixd::MakeYRotation(M_PI / 7),
                                Vector3d(4, 5, 6));

  InternalGeometry geometry(source_id, sphere.Clone(), frame_id, geometry_id,
                            name, X_FGold);
  EXPECT_TRUE(CompareMatrices(geometry.X_FG().GetAsMatrix34(),
                              X_FGold.GetAsMatrix34()));
  geometry.set_pose(X_FGNew);
  EXPECT_TRUE(CompareMatrices(geometry.X_FG().GetAsMatrix34(),
                              X_FGNew.GetAsMatrix34()));
}

// Tests the removal of all roles.
GTEST_TEST(InternalGeometryTest, RemoveRole) {
  // Configure a geometry with all roles; we assume from previous unit tests
  // that the geometry's state is correct.
  InternalGeometry geometry;
  geometry.SetRole(ProximityProperties());
  geometry.SetRole(IllustrationProperties());
  geometry.SetRole(PerceptionProperties());

  // Two notes on the structure of this test:
  //  1. As currently formulated, the correctness of the test depends on the
  //     order of these actions. Changing the order can lead to meaningless
  //     test failure.
  //  2. This test doesn't exhaustively test all permutations of removing a
  //     role. (There are 8 unique configurations and 24 total possible removal
  //     invocations.) We assume that the *suggestion* of independence suggested
  //     here is actually true.

  // Case: Remove proximity, other roles persist.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));
  EXPECT_FALSE(geometry.has_role(Role::kUnassigned));

  // Case: Redundant removal of role is a no-op.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));
  EXPECT_FALSE(geometry.has_role(Role::kUnassigned));

  // Case: Remove illustration, only perception remains.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));
  EXPECT_FALSE(geometry.has_role(Role::kUnassigned));

  // Case: Redundant removal of role is a no-op.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));
  EXPECT_FALSE(geometry.has_role(Role::kUnassigned));

  // Case: Remove perception, no roles exist.
  DRAKE_EXPECT_NO_THROW(geometry.RemovePerceptionRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));
  EXPECT_TRUE(geometry.has_role(Role::kUnassigned));

  // Case: Redundant removal of role is a no-op.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));
  EXPECT_TRUE(geometry.has_role(Role::kUnassigned));
}

GTEST_TEST(InternalGeometryTest, DeformableGeometry) {
  SourceId source_id = SourceId::get_new_id();
  Sphere sphere(1.0);
  constexpr double kRezHint = .5;
  FrameId frame_id = FrameId::get_new_id();
  GeometryId deformable_geometry_id = GeometryId::get_new_id();
  std::string name = "sphere";
  const RigidTransformd X_FG(RollPitchYawd(1, 2, 3), Vector3d(3, 4, 5));
  const VolumeMesh<double> expected_mesh_G = MakeSphereVolumeMesh<double>(
      sphere, kRezHint, TessellationStrategy::kDenseInteriorVertices);

  // Confirms that a deformable geometry can be constructed.
  InternalGeometry geometry(source_id, make_unique<Sphere>(sphere), frame_id,
                            deformable_geometry_id, name, X_FG, kRezHint);
  const VolumeMesh<double>* reference_mesh_G = geometry.reference_mesh();
  ASSERT_NE(reference_mesh_G, nullptr);
  EXPECT_TRUE(expected_mesh_G.Equal(*reference_mesh_G));

  EXPECT_TRUE(geometry.X_FG().IsExactlyEqualTo(X_FG));
  // Meshed geometry is never anchored.
  EXPECT_TRUE(geometry.is_dynamic());
  // Meshed geometry is always deformable.
  EXPECT_TRUE(geometry.is_deformable());

  // Confirms that the a geometry created without resolution hint is not a
  // deformable geometry and doesn't have a reference mesh.
  GeometryId rigid_geometry_id = GeometryId::get_new_id();
  InternalGeometry rigid_geometry(source_id, make_unique<Sphere>(sphere),
                                  frame_id, rigid_geometry_id, name,
                                  RigidTransformd());
  EXPECT_EQ(rigid_geometry.reference_mesh(), nullptr);
  EXPECT_FALSE(rigid_geometry.is_deformable());
}

GTEST_TEST(InternalGeometryTest, Rename) {
  InternalGeometry geometry;
  EXPECT_TRUE(geometry.name().empty());
  {
    // Scope the new name to make sure the object takes a copy.
    std::string new_name("new_name");
    geometry.set_name(new_name);
  }
  EXPECT_EQ(geometry.name(), "new_name");
}

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
