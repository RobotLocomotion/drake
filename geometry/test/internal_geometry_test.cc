#include "drake/geometry/internal_geometry.h"

#include <utility>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/proximity/make_sphere_mesh.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
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
  }
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

  // Case: Redundant removal of role is a no-op.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveProximityRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_TRUE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove illustration, only perception remains.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Redundant removal of role is a no-op.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_TRUE(geometry.has_role(Role::kPerception));

  // Case: Remove perception, no roles exist.
  DRAKE_EXPECT_NO_THROW(geometry.RemovePerceptionRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));

  // Case: Redundant removal of role is a no-op.
  DRAKE_EXPECT_NO_THROW(geometry.RemoveIllustrationRole());
  EXPECT_FALSE(geometry.has_role(Role::kProximity));
  EXPECT_FALSE(geometry.has_role(Role::kIllustration));
  EXPECT_FALSE(geometry.has_role(Role::kPerception));
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
  // The immediate parent of a deformable geometry is the frame.
  EXPECT_TRUE(geometry.X_PG().IsExactlyEqualTo(X_FG));
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

}  // namespace
}  // namespace internal
}  // namespace geometry
}  // namespace drake
