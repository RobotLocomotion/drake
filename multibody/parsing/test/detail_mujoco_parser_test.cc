#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::FrameId;
using geometry::GeometryId;
using geometry::Role;
using geometry::SceneGraph;
using geometry::SceneGraphInspector;
using geometry::ShapeName;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;

// TODO(russt): Additional tests are required to get complete coverage of all
// error/exception handling.

GTEST_TEST(MujocoParser, GymModels) {
  // Confirm successful parsing of the MuJoCo models in the DeepMind control
  // suite.
  std::string prev_log_level = logging::set_log_level("off");
  for (const auto& f :
       {"acrobot", "cartpole", "cheetah", "finger", "fish", "hopper",
        "humanoid", "humanoid_CMU", "lqr", "manipulator", "pendulum",
        "point_mass", "reacher", "stacker", "swimmer", "walker"}) {
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);

    const std::string filename = FindResourceOrThrow(
        fmt::format("drake/multibody/parsing/dm_control/suite/{}.xml", f));
    AddModelFromMujocoXml({.file_name = &filename}, f, std::nullopt, &plant);

    EXPECT_TRUE(plant.HasModelInstanceNamed(f));
  }
  logging::set_log_level(prev_log_level);
}

GTEST_TEST(MujocoParser, Option) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <option gravity="0 -9.81 0"/>
</mujoco>
)""";

  AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &xml}, "test",
                        std::nullopt, &plant);
  EXPECT_TRUE(CompareMatrices(plant.gravity_field().gravity_vector(),
                              Vector3d{0, -9.81, 0}));
}

GTEST_TEST(MujocoParser, GeometryTypes) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="default" size="0.1"/>
    <geom name="plane" type="plane"/>
    <geom name="sphere" type="sphere" size="0.1"/>
    <geom name="capsule" type="capsule" size="0.1 2.0"/>
    <geom name="ellipsoid" type="ellipsoid" size="0.1 0.2 0.3"/>
    <geom name="cylinder" type="cylinder" size="0.1 2.0"/>
    <geom name="box" type="box" size="0.1 2.0 3.0"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &xml}, "test",
                        std::nullopt, &plant);

  const SceneGraphInspector<double>& inspector = scene_graph.model_inspector();

  auto CheckShape = [&inspector](const std::string& geometry_name,
                                 const std::string& shape_name) {
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, geometry_name);
    EXPECT_EQ(geometry::ShapeName(inspector.GetShape(geom_id)).name(),
              shape_name);
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kPerception, geometry_name);
    EXPECT_EQ(geometry::ShapeName(inspector.GetShape(geom_id)).name(),
              shape_name);
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kIllustration, geometry_name);
    EXPECT_EQ(geometry::ShapeName(inspector.GetShape(geom_id)).name(),
              shape_name);
  };

  CheckShape("default", "Sphere");
  CheckShape("sphere", "Sphere");
  CheckShape("capsule", "Capsule");
  CheckShape("ellipsoid", "Ellipsoid");
  CheckShape("cylinder", "Cylinder");
  CheckShape("box", "Box");

  // TODO(russt): Check the sizes of the shapes.  (It seems that none of our
  // existing parser tests actually check the sizes of of the shapes; the Reify
  // workflow makes it painful.)

  // Check unrecognized geom types.
  xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="garbage" type="garbage"/>
  </worldbody>
</mujoco>
)""";
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &xml},
                            "test_unrecognized", std::nullopt, &plant),
      ".*Unrecognized.*");
}

GTEST_TEST(MujocoParser, GeometryPose) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // By default, angles are in degrees.
  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="identity" type="sphere" size="0.1" />
    <geom name="quat" quat="0 1 0 0" pos="1 2 3" type="sphere" size="0.1" />
    <geom name="axisangle" axisangle="4 5 6 30" pos="1 2 3" type="sphere" 
          size="0.1" />
    <geom name="euler" euler="30 45 60" pos="1 2 3" type="sphere" size="0.1" />
    <geom name="xyaxes" xyaxes="0 1 0 -1 0 0" pos="1 2 3" type="sphere" 
          size="0.1" />
    <geom name="zaxis" zaxis="0 1 0" pos="1 2 3" type="sphere" size="0.1" />
    <geom name="fromto_capsule" fromto="-1 -3 -3 -1 -1 -3"
          quat="0.2 0.4 0.3 .1" pos="1 2 3" type="capsule" size="0.1" />
    <geom name="fromto_cylinder" fromto="-1 -3 -3 -1 -1 -3"
          quat="0.2 0.4 0.3 .1" pos="1 2 3" type="cylinder" size="0.1" />
  </worldbody>
</mujoco>
)""";

  // Explicitly set radians.
  std::string radians_xml = R"""(
<mujoco model="test">
  <compiler angle="radian"/>
  <worldbody>
    <geom name="axisangle_rad" axisangle="4 5 6 0.5" pos="1 2 3" type="sphere" 
          size="0.1" />
    <geom name="euler_rad" euler="0.5 0.7 1.05" pos="1 2 3" type="sphere"
          size="0.1" />
  </worldbody>
</mujoco>
)""";

  // Explicitly set degrees.
  std::string degrees_xml = R"""(
<mujoco model="test">
  <compiler angle="degree"/>
  <worldbody>
    <geom name="axisangle_deg" axisangle="4 5 6 30" pos="1 2 3" type="sphere" 
          size="0.1" />
    <geom name="euler_deg" euler="30 45 60" pos="1 2 3" type="sphere"
          size="0.1" />
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &xml}, "test",
                        std::nullopt, &plant);
  AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &radians_xml},
                        "radians_test", std::nullopt, &plant);
  AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &degrees_xml},
                        "degrees_test", std::nullopt, &plant);

  const SceneGraphInspector<double>& inspector = scene_graph.model_inspector();

  auto CheckPose = [&inspector](const std::string& geometry_name,
                                const RigidTransformd& X_PG) {
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, geometry_name);
    EXPECT_TRUE(
        inspector.GetPoseInParent(geom_id).IsNearlyEqualTo(X_PG, 1e-14));
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kPerception, geometry_name);
    EXPECT_TRUE(
        inspector.GetPoseInParent(geom_id).IsNearlyEqualTo(X_PG, 1e-14));
    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kIllustration, geometry_name);
    EXPECT_TRUE(
        inspector.GetPoseInParent(geom_id).IsNearlyEqualTo(X_PG, 1e-14));
  };

  const Vector3d p{1, 2, 3};
  CheckPose("identity", RigidTransformd());
  CheckPose("quat", RigidTransformd(Eigen::Quaternion<double>{0, 1, 0, 0}, p));
  CheckPose("axisangle",
            RigidTransformd(
                Eigen::AngleAxis<double>(M_PI / 6.0, Vector3d{4, 5, 6}), p));
  CheckPose("euler", RigidTransformd(
                         RollPitchYawd{M_PI / 6.0, M_PI / 4.0, M_PI / 3.0}, p));
  CheckPose("xyaxes",
            RigidTransformd(RotationMatrixd::MakeZRotation(M_PI / 2.0), p));
  CheckPose("zaxis",
            RigidTransformd(RotationMatrixd::MakeXRotation(-M_PI / 2.0), p));
  CheckPose("fromto_capsule",
            RigidTransformd(RotationMatrixd::MakeXRotation(-M_PI / 2.0), -p));
  CheckPose("fromto_cylinder",
            RigidTransformd(RotationMatrixd::MakeXRotation(-M_PI / 2.0), -p));

  CheckPose(
      "axisangle_rad",
      RigidTransformd(Eigen::AngleAxis<double>(0.5, Vector3d{4, 5, 6}), p));
  CheckPose("euler_rad", RigidTransformd(RollPitchYawd{0.5, 0.7, 1.05}, p));
  CheckPose("axisangle_deg",
            RigidTransformd(
                Eigen::AngleAxis<double>(M_PI / 6.0, Vector3d{4, 5, 6}), p));
  CheckPose(
      "euler_deg",
      RigidTransformd(RollPitchYawd{M_PI / 6.0, M_PI / 4.0, M_PI / 3.0}, p));
}

GTEST_TEST(MujocoParser, GeometryProperties) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="default" type="sphere" size="0.1"/>
    <geom name="sphere" type="sphere" size="0.1" friction="0.9 0.0 0.0"
          rgba="1 0 0 1"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({.file_name = nullptr, .file_contents = &xml}, "test",
                        std::nullopt, &plant);

  const SceneGraphInspector<double>& inspector = scene_graph.model_inspector();

  auto CheckProperties = [&inspector](const std::string& geometry_name,
                                      double mu, const Vector4d& rgba) {
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, geometry_name);
    const geometry::ProximityProperties* proximity_prop =
        inspector.GetProximityProperties(geom_id);
    EXPECT_TRUE(proximity_prop);
    EXPECT_TRUE(proximity_prop->HasProperty("material", "coulomb_friction"));
    const auto& friction = proximity_prop->GetProperty<CoulombFriction<double>>(
        "material", "coulomb_friction");
    EXPECT_EQ(friction.static_friction(), mu);
    EXPECT_EQ(friction.dynamic_friction(), mu);

    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kPerception, geometry_name);
    const geometry::PerceptionProperties* perception_prop =
        inspector.GetPerceptionProperties(geom_id);
    EXPECT_TRUE(perception_prop);
    EXPECT_TRUE(perception_prop->HasProperty("phong", "diffuse"));
    EXPECT_TRUE(CompareMatrices(
        perception_prop->GetProperty<Vector4d>("phong", "diffuse"), rgba));

    geom_id = inspector.GetGeometryIdByName(inspector.world_frame_id(),
                                            Role::kIllustration, geometry_name);
    const geometry::IllustrationProperties* illustration_prop =
        inspector.GetIllustrationProperties(geom_id);
    EXPECT_TRUE(illustration_prop);
    EXPECT_TRUE(illustration_prop->HasProperty("phong", "diffuse"));
    EXPECT_TRUE(CompareMatrices(
        illustration_prop->GetProperty<Vector4d>("phong", "diffuse"), rgba));
  };

  CheckProperties("default", 1.0, Vector4d{.5, .5, .5, 1});
  CheckProperties("sphere", 0.9, Vector4d{1, 0, 0, 1});
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
