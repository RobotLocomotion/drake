#include "drake/multibody/parsing/detail_mujoco_parser.h"

#include <filesystem>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/weld_joint.h"

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
    AddModelFromMujocoXml({DataSource::kFilename, &filename}, f, {}, &plant);

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

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);
  EXPECT_TRUE(CompareMatrices(plant.gravity_field().gravity_vector(),
                              Vector3d{0, -9.81, 0}));
}

GTEST_TEST(MujocoParser, GeometryTypes) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <default class="default_box">
    <geom type="box" size="0.1 0.2 0.3"/>
    <default class="sub">
      <geom size="0.4 0.5 0.6"/>
    </default>
  </default>
  <worldbody>
    <geom name="default" size="0.1"/>
    <geom name="plane" type="plane"/>
    <geom name="sphere" type="sphere" size="0.1"/>
    <geom name="capsule" type="capsule" size="0.1 2.0"/>
    <geom name="ellipsoid" type="ellipsoid" size="0.1 0.2 0.3"/>
    <geom name="cylinder" type="cylinder" size="0.1 2.0"/>
    <geom name="box" type="box" size="0.1 2.0 3.0"/>
    <geom name="box_from_default" class="default_box"/>
    <geom name="ellipsoid_from_default" type="ellipsoid" class="default_box"/>
    <geom name="box_from_sub" class="sub"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);
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

  // TODO(russt): Check the sizes of the shapes.  (It seems that none of our
  // existing parser tests actually check the sizes of of the shapes; the Reify
  // workflow makes it painful.)  Note that we do have some coverage from this
  // by testing for inertias set from the geometry below.

  CheckShape("default", "Sphere");
  CheckShape("plane", "HalfSpace");
  CheckShape("sphere", "Sphere");
  CheckShape("capsule", "Capsule");
  CheckShape("ellipsoid", "Ellipsoid");
  CheckShape("cylinder", "Cylinder");
  CheckShape("box", "Box");
  CheckShape("box_from_default", "Box");
  CheckShape("ellipsoid_from_default", "Ellipsoid");
  CheckShape("box_from_sub", "Box");
}

GTEST_TEST(MujocoParser, UnrecognizedGeometryTypes) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="garbage" type="garbage"/>
  </worldbody>
</mujoco>
)""";
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromMujocoXml({DataSource::kContents, &xml},
                            "test_unrecognized", {}, &plant),
      ".*Unrecognized.*");
}

GTEST_TEST(MujocoParser, GeometryPose) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  // By default, angles are in degrees.
  std::string xml = R"""(
<mujoco model="test">
  <default class="default_pose">
    <geom pos="1 2 3" quat="0 1 0 0"/>
  </default>
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
    <geom name="from_default" type="sphere" size="0.1" class="default_pose" />
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

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);
  AddModelFromMujocoXml({DataSource::kContents, &radians_xml},
                        "radians_test", {}, &plant);
  AddModelFromMujocoXml({DataSource::kContents, &degrees_xml},
                        "degrees_test", {}, &plant);

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
  CheckPose("from_default",
            RigidTransformd(Eigen::Quaternion<double>{0, 1, 0, 0}, p));

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

GTEST_TEST(MujocoParser, GeometryPoseErrors) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  const std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <geom name="two_angles" type="sphere" size="0.1" pos="1 2 3"
          quat="0 1 0 0" euler="30 45 60" />
  </worldbody>
</mujoco>
)""";
  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant),
      ".*has more than one orientation attribute specified.*");
}

GTEST_TEST(MujocoParser, GeometryProperties) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <asset>
    <material name="Orange" rgba="1 0.5 0 1"/>
  </asset>
  <default class="default_rgba">
    <geom rgba="0 1 0 1"/>
  </default>
  <worldbody>
    <geom name="default" type="sphere" size="0.1"/>
    <geom name="default_rgba" type="sphere" size="0.1" class="default_rgba"/>
    <geom name="sphere" type="sphere" size="0.1" friction="0.9 0.0 0.0"
          rgba="1 0 0 1"/>
    <geom name="orange" type="sphere" size="0.1" material="Orange"/>
    <geom name="red" type="sphere" size="0.1" material="Orange" rgba="1 0 0 1"/>
    <geom name="green" type="sphere" size="0.1" material="Orange"
          class="default_rgba"/>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);

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
  CheckProperties("default_rgba", 1.0, Vector4d{0, 1, 0, 1});
  CheckProperties("sphere", 0.9, Vector4d{1, 0, 0, 1});
  CheckProperties("orange", 1.0, Vector4d{1, 0.5, 0, 1});
  // If both material and rgba are specified, rgba wins.
  CheckProperties("red", 1.0, Vector4d{1, 0, 0, 1});
  CheckProperties("green", 1.0, Vector4d{0, 1, 0, 1});
}

void TestMesh(std::string expected_filename, std::string mesh_asset,
              std::string compiler = "", double expected_scale = 1.0) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = fmt::format(
      R"""(
<mujoco model="test">
  {}
  <asset>
    {}
  </asset>
  <worldbody>
    <geom name="box_geom" type="mesh" mesh="box"/>
  </worldbody>
</mujoco>
)""",
      compiler, mesh_asset);

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);

  const SceneGraphInspector<double>& inspector = scene_graph.model_inspector();
  GeometryId geom_id = inspector.GetGeometryIdByName(
      inspector.world_frame_id(), Role::kProximity, "box_geom");
  auto* mesh =
      dynamic_cast<const geometry::Mesh*>(&inspector.GetShape(geom_id));
  EXPECT_NE(mesh, nullptr);
  EXPECT_EQ(mesh->filename(), expected_filename);
  EXPECT_EQ(mesh->scale(), expected_scale);
}

GTEST_TEST(MujocoParser, MeshFiles) {
  std::string box_obj = std::filesystem::canonical(FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj"));

  // Absolute path, referencing the obj directly.
  std::string mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}"/>)""", box_obj);
  TestMesh(box_obj, mesh_asset);

  // Absolute path, referencing an stl with an obj replacement available.
  std::string quad_cube_stl =
      FindResourceOrThrow("drake/geometry/test/quad_cube.stl");
  std::string quad_cube_obj = std::filesystem::canonical(
      FindResourceOrThrow("drake/geometry/test/quad_cube.obj"));
  mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}"/>)""", quad_cube_stl);
  TestMesh(quad_cube_obj, mesh_asset);

  // Relative path (from string, so path is relative to cwd).
  mesh_asset =
      R"""(<mesh name="box" file="multibody/parsing/test/box_package/meshes/box.obj"/>)""";
  TestMesh(box_obj, mesh_asset);

  // Absolute path in the meshdir compiler attribute.
  mesh_asset =
      R"""(<mesh name="box" file="box.obj"/>)""";
  std::string compiler =
      fmt::format(R"""(<compiler meshdir={}/>)""",
                  std::filesystem::path(box_obj).parent_path());
  TestMesh(box_obj, mesh_asset, compiler);

  // Relative path + meshdir compiler attribute.
  mesh_asset =
      R"""(<mesh name="box" file="box.obj"/>)""";
  compiler = R"""(<compiler meshdir="multibody/parsing/test/box_package/meshes"/>)""";
  TestMesh(box_obj, mesh_asset, compiler);

  // Relative path (from file)
  {
    MultibodyPlant<double> plant(0.0);
    SceneGraph<double> scene_graph;
    plant.RegisterAsSourceForSceneGraph(&scene_graph);

    const std::string file = FindResourceOrThrow(
        "drake/multibody/parsing/test/box_package/mjcfs/box.xml");

    AddModelFromMujocoXml({DataSource::kFilename, &file}, "test", {}, &plant);

    const SceneGraphInspector<double>& inspector =
        scene_graph.model_inspector();
    GeometryId geom_id = inspector.GetGeometryIdByName(
        inspector.world_frame_id(), Role::kProximity, "box_geom");
    auto* mesh =
        dynamic_cast<const geometry::Mesh*>(&inspector.GetShape(geom_id));
    EXPECT_NE(mesh, nullptr);
    EXPECT_EQ(mesh->filename(), box_obj);
    EXPECT_EQ(mesh->scale(), 1.0);
  }

  // Test the scale attribute.
  mesh_asset =
      fmt::format(R"""(<mesh name="box" file="{}" scale="2 2 2"/>)""", box_obj);
  TestMesh(box_obj, mesh_asset, "", 2.0);
}

GTEST_TEST(MujocoParser, InertiaFromGeometry) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string box_obj = FindResourceOrThrow(
      "drake/multibody/parsing/test/box_package/meshes/box.obj");

  std::string xml = fmt::format(R"""(
<mujoco model="test">
  <default class="main">
    <geom mass="2.53"/>
    <default class="default_box">
      <geom type="box" size="0.1 0.2 0.3"/>
      <default class="sub">
        <geom size="0.4 0.5 0.6"/>
      </default>
    </default>
  </default>
  <asset>
    <mesh name="box_mesh" file="{}"/>
  </asset>
  <worldbody>
    <body name="default">
      <geom name="default" size="0.1"/>
    </body>
    <body name="sphere">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" type="sphere" size="0.1"/>
    </body>
    <body name="capsule">
      <geom name="capsule" type="capsule" size="0.1 2.0"/>
    </body>
    <body name="ellipsoid">
      <geom name="ellipsoid" type="ellipsoid" size="0.1 0.2 0.3"/>
    </body>
    <body name="cylinder">
      <geom name="cylinder" type="cylinder" size="0.1 2.0"/>
    </body>
    <body name="box">
      <geom name="box" type="box" size="0.1 2.0 3.0"/>
    </body>
    <body name="box_from_default">
      <geom name="box_from_default" class="default_box"/>
    </body>
    <body name="ellipsoid_from_default">
      <geom name="ellipsoid_from_default" type="ellipsoid" class="default_box"/>
    </body>
    <body name="box_from_sub" childclass="sub">
      <geom name="box_from_sub"/>
    </body>
    <body name="two_spheres">
      <geom name="thing1" type="sphere" size="0.1"/>
      <geom name="thing2" type="sphere" size="0.2"/>
    </body>
    <body name="offset_cylinder">
      <geom name="offset_cylinder" type="cylinder" size="0.4 2.0" pos="2 0 0"
            zaxis="1 0 0"/>
    </body>
    <body name="box_from_mesh">
      <geom name="box_from_mesh" type="mesh" mesh="box_mesh" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
)""",
                                box_obj);

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);

  xml = fmt::format(R"""(
<mujoco model="test_auto">
  <compiler inertiafromgeom="auto"/>
  <asset>
    <mesh name="box_mesh" file="{}"/>
  </asset>
  <worldbody>
    <body name="sphere_auto">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" mass="2.53" type="sphere" size="0.1"/>
    </body>
    <body name="box_w_density">
      <geom name="box_w_density" density="123" type="box" size=".4 .5 .6"/>
    </body>
    <body name="box_default_density">
      <geom name="box_w_density" type="box" size=".4 .5 .6"/>
    </body>
    <body name="box_from_mesh_w_density">
      <geom name="box_from_mesh_w_density" type="mesh" mesh="box_mesh"
            density="1.0"/>
    </body>
  </worldbody>
</mujoco>
)""",
                    box_obj);

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test_auto", {}, &plant);

  xml = R"""(
<mujoco model="test_true">
  <compiler inertiafromgeom="true"/>
  <default class="main">
    <geom mass="2.53"/>
  </default>
  <worldbody>
    <body name="sphere_true">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" type="sphere" size="0.1"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test_true", {}, &plant);

  xml = R"""(
<mujoco model="test_false">
  <compiler inertiafromgeom="false"/>
  <default class="main">
    <geom mass="2.53"/>
  </default>
  <worldbody>
    <body name="sphere_false">
      <inertial mass="524" diaginertia="1 2 3"/>
      <geom name="sphere" type="sphere" size="0.1"/>
    </body>
    <body name="full_inertia">
      <inertial mass="1.23" fullinertia="2 2 2 -.75 -.75 -.75"/>
    </body>
    <body name="full_inertia_w_pose">
      <inertial mass="1.23" fullinertia="2 2 2 -.75 -.75 -.75"
               quat="0.2 0.4 0.3 .1" pos="1 2 3"/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({DataSource::kContents, &xml},
                        "test_false", {}, &plant);

  plant.Finalize();

  auto context = plant.CreateDefaultContext();

  auto check_body = [&plant, &context](
                        const std::string& body_name,
                        const UnitInertia<double>& unit_M_BBo_B) {
    const Body<double>& body = plant.GetBodyByName(body_name);
    EXPECT_TRUE(CompareMatrices(
        body.CalcSpatialInertiaInBodyFrame(*context).CopyToFullMatrix6(),
        SpatialInertia<double>(2.53, Vector3d::Zero(), unit_M_BBo_B)
            .CopyToFullMatrix6(),
        1e-14));
  };

  auto check_body_spatial = [&plant, &context](
                                const std::string& body_name,
                                const SpatialInertia<double>& M_BBo_B,
                                double tol = 1e-14) {
    const Body<double>& body = plant.GetBodyByName(body_name);
    EXPECT_TRUE(CompareMatrices(
        body.CalcSpatialInertiaInBodyFrame(*context).CopyToFullMatrix6(),
        M_BBo_B.CopyToFullMatrix6(), tol));
  };

  const SpatialInertia<double> inertia_from_inertial_tag =
      SpatialInertia<double>::MakeFromCentralInertia(
          524, Vector3d::Zero(), RotationalInertia<double>(1, 2, 3));

  check_body("default", UnitInertia<double>::SolidSphere(0.1));
  check_body_spatial("sphere", inertia_from_inertial_tag);
  check_body("capsule", UnitInertia<double>::SolidCapsule(0.1, 4.0));
  check_body("ellipsoid", UnitInertia<double>::SolidEllipsoid(0.1, 0.2, 0.3));
  check_body("cylinder", UnitInertia<double>::SolidCylinder(0.1, 4.0));
  check_body("box", UnitInertia<double>::SolidBox(0.2, 4.0, 6.0));
  check_body("box_from_default", UnitInertia<double>::SolidBox(0.2, 0.4, 0.6));
  check_body("ellipsoid_from_default",
             UnitInertia<double>::SolidEllipsoid(0.1, 0.2, 0.3));
  check_body("box_from_sub", UnitInertia<double>::SolidBox(0.8, 1.0, 1.2));
  SpatialInertia<double> M_BBo_B = SpatialInertia<double>(
      2.53, Vector3d::Zero(), UnitInertia<double>::SolidSphere(0.1));
  M_BBo_B += SpatialInertia<double>(2.53, Vector3d::Zero(),
                                    UnitInertia<double>::SolidSphere(0.2));
  check_body_spatial("two_spheres", M_BBo_B);
  // Use the equation for the moment of inertia of a cylinder about one end.
  M_BBo_B = SpatialInertia<double>(
      2.53,                                    // mass
      Vector3d{2.0, 0.0, 0.0},                 // Center of mass
      UnitInertia<double>(0.16 / 2.0,          // 1/2 r²
                          .04 + (16.0 / 3.0),  // 1/4 r^2+ 1/3 L^2)
                          .04 + (16.0 / 3.0)));
  check_body_spatial("offset_cylinder", M_BBo_B);
  check_body_spatial(
      "box_from_mesh",
      SpatialInertia<double>(1.0, Vector3d::Zero(),
                             UnitInertia<double>::SolidCube(2.0)),
      1e-13);

  check_body_spatial("sphere_auto", inertia_from_inertial_tag);
  check_body("sphere_true", UnitInertia<double>::SolidSphere(0.1));
  check_body_spatial("sphere_false", inertia_from_inertial_tag);

  check_body_spatial(
      "box_w_density",
      SpatialInertia<double>(123 * .8 * 1.2, Vector3d::Zero(),
                             UnitInertia<double>::SolidBox(0.8, 1.0, 1.2)));
  check_body_spatial(
      "box_default_density",
      SpatialInertia<double>(1000 * .8 * 1.2, Vector3d::Zero(),
                             UnitInertia<double>::SolidBox(0.8, 1.0, 1.2)));
  check_body_spatial(
      "box_from_mesh_w_density",
      SpatialInertia<double>(8.0, Vector3d::Zero(),
                             UnitInertia<double>::SolidCube(2.0)),
      1e-12);

  // A cube rotating about its corner.
  RotationalInertia<double> I_BFo_B(2, 2, 2, -.75, -.75, -.75);
  check_body_spatial("full_inertia",
                     SpatialInertia<double>::MakeFromCentralInertia(
                         1.23, Vector3d::Zero(), I_BFo_B));
  // The rotational component of this inertia is ignored, because the full
  // inertia was specified.
  check_body_spatial("full_inertia_w_pose",
                     SpatialInertia<double>::MakeFromCentralInertia(
                         1.23, Vector3d{1, 2, 3}, I_BFo_B));
}

GTEST_TEST(MujocoParser, Joint) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <default>
    <geom type="sphere" size="1"/>
  </default>
  <worldbody>
    <body name="freejoint" pos="1 2 3" euler="30 45 60">
      <freejoint name="freejoint"/>
    </body>
    <body name="free" pos="1 2 3" euler="30 45 60">
      <joint type="free" name="free"/>
    </body>
    <body name="ball" pos="1 2 3" euler="30 45 60">
      <joint type="ball" name="ball" damping="0.1" pos=".1 .2 .3"/>
    </body>
    <body name="slide" pos="1 2 3" euler="30 45 60">
      <joint type="slide" name="slide" damping="0.2" pos=".1 .2 .3"
             axis="1 0 0" limited="true" range="-2 1.5"/>
    </body>
    <body name="hinge" pos="1 2 3" euler="30 45 60">
      <joint type="hinge" name="hinge" damping="0.3" pos=".1 .2 .3"
             axis="0 1 0" limited="true" range="-30 60"/>
    </body>
    <body name="default" pos="1 2 3" euler="30 45 60">
      <!-- without the limited=true tag -->
      <joint name="default" damping="0.4" range="-20 15"/>
    </body>
    <body name="weld" pos="1 2 3" euler="30 45 60"/>
    <body name="two_hinges" pos="1 2 3" euler="30 45 60">
      <joint type="hinge" name="hinge1" damping="0.5" pos=".1 .2 .3"
             axis="1 0 0"/>
      <joint type="hinge" name="hinge2" damping="0.6" pos=".1 .2 .3"
             axis="0 1 0"/>
    </body>
    <body name="default_joint">
      <!-- provides code coverage for defaults logic. -->
      <joint/>
    </body>
  </worldbody>
</mujoco>
)""";

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);
  plant.Finalize();

  auto context = plant.CreateDefaultContext();

  RigidTransformd X_WB(RollPitchYawd{M_PI / 6.0, M_PI / 4.0, M_PI / 3.0},
                       Vector3d{1.0, 2.0, 3.0});
  Vector3d pos{.1, .2, .3};

  const Body<double>& freejoint_body = plant.GetBodyByName("freejoint");
  EXPECT_FALSE(plant.HasJointNamed("freejoint"));
  EXPECT_TRUE(freejoint_body.is_floating());
  EXPECT_TRUE(plant.GetFreeBodyPose(*context, freejoint_body)
                  .IsNearlyEqualTo(X_WB, 1e-14));

  const Body<double>& free_body = plant.GetBodyByName("free");
  EXPECT_FALSE(plant.HasJointNamed("free"));
  EXPECT_TRUE(free_body.is_floating());
  EXPECT_TRUE(
      plant.GetFreeBodyPose(*context, free_body).IsNearlyEqualTo(X_WB, 1e-14));

  const BallRpyJoint<double>& ball_joint =
      plant.GetJointByName<BallRpyJoint>("ball");
  EXPECT_EQ(ball_joint.damping(), 0.1);
  EXPECT_TRUE(ball_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(
      plant.GetBodyByName("ball").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));

  const PrismaticJoint<double>& slide_joint =
      plant.GetJointByName<PrismaticJoint>("slide");
  EXPECT_EQ(slide_joint.damping(), 0.2);
  EXPECT_TRUE(slide_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(
      CompareMatrices(slide_joint.translation_axis(), Vector3d{1, 0, 0}));
  EXPECT_TRUE(
      plant.GetBodyByName("slide").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  EXPECT_TRUE(CompareMatrices(
      plant.GetJointByName("slide").position_lower_limits(), Vector1d{-2.0}));
  EXPECT_TRUE(CompareMatrices(
      plant.GetJointByName("slide").position_upper_limits(), Vector1d{1.5}));

  const RevoluteJoint<double>& hinge_joint =
      plant.GetJointByName<RevoluteJoint>("hinge");
  EXPECT_EQ(hinge_joint.damping(), 0.3);
  EXPECT_TRUE(hinge_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(CompareMatrices(hinge_joint.revolute_axis(), Vector3d{0, 1, 0}));
  EXPECT_TRUE(
      plant.GetBodyByName("hinge").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  EXPECT_TRUE(
      CompareMatrices(plant.GetJointByName("hinge").position_lower_limits(),
                      Vector1d{-M_PI / 6.0}, 1e-14));
  EXPECT_TRUE(
      CompareMatrices(plant.GetJointByName("hinge").position_upper_limits(),
                      Vector1d{M_PI / 3.0}, 1e-14));

  const RevoluteJoint<double>& default_joint =
      plant.GetJointByName<RevoluteJoint>("default");
  EXPECT_EQ(default_joint.damping(), 0.4);
  EXPECT_TRUE(default_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyIdentity(1e-14));
  EXPECT_TRUE(
      CompareMatrices(default_joint.revolute_axis(), Vector3d{0, 0, 1}));
  EXPECT_TRUE(
      plant.GetBodyByName("default").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  EXPECT_TRUE(
      CompareMatrices(plant.GetJointByName("default").position_lower_limits(),
                      Vector1d{-std::numeric_limits<double>::infinity()}));
  EXPECT_TRUE(
      CompareMatrices(plant.GetJointByName("default").position_upper_limits(),
                      Vector1d{std::numeric_limits<double>::infinity()}));

  const RevoluteJoint<double>& hinge1_joint =
      plant.GetJointByName<RevoluteJoint>("hinge1");
  EXPECT_EQ(hinge1_joint.damping(), 0.5);
  EXPECT_TRUE(CompareMatrices(hinge1_joint.revolute_axis(), Vector3d{1, 0, 0}));
  EXPECT_TRUE(hinge1_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  const RevoluteJoint<double>& hinge2_joint =
      plant.GetJointByName<RevoluteJoint>("hinge2");
  EXPECT_EQ(hinge2_joint.damping(), 0.6);
  EXPECT_TRUE(CompareMatrices(hinge2_joint.revolute_axis(), Vector3d{0, 1, 0}));
  EXPECT_TRUE(hinge2_joint.frame_on_parent()
                  .CalcPoseInBodyFrame(*context)
                  .IsNearlyEqualTo(RigidTransformd(pos), 1e-14));
  EXPECT_TRUE(plant.GetBodyByName("two_hinges")
                  .EvalPoseInWorld(*context)
                  .IsNearlyEqualTo(X_WB, 1e-14));

  EXPECT_TRUE(
      plant.GetBodyByName("weld").EvalPoseInWorld(*context).IsNearlyEqualTo(
          X_WB, 1e-14));
  const WeldJoint<double>& weld_joint =
      plant.GetJointByName<WeldJoint>("world_welds_to_weld");
  EXPECT_TRUE(weld_joint.X_FM().IsNearlyEqualTo(X_WB, 1e-14));
}

GTEST_TEST(MujocoParser, JointThrows) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <worldbody>
    <body name="free" pos="1 2 3" euler="30 45 60">
      <joint type="free" name="free"/>
      <joint type="hinge"/>
    </body>
  </worldbody>
</mujoco>
)""";

  DRAKE_EXPECT_THROWS_MESSAGE(
      AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant),
      ".*a free joint is defined.*");
}

GTEST_TEST(MujocoParser, Motor) {
  MultibodyPlant<double> plant(0.0);
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string xml = R"""(
<mujoco model="test">
  <default>
    <geom type="sphere" size="1"/>
  </default>
  <worldbody>
    <body>
      <joint type="hinge" name="hinge0" axis="0 1 0"/>
    </body>
    <body>
      <joint type="hinge" name="hinge1" axis="0 1 0"/>
    </body>
    <body>
      <joint type="hinge" name="hinge2" axis="0 1 0"/>
    </body>
    <body>
      <joint type="hinge" name="hinge3" axis="0 1 0"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge0"/>
    <!-- intentionally asymmetric effort limits to cover the warning code -->
    <motor name="motor1" joint="hinge1" ctrllimited="true" ctrlrange="-1 2"/>
    <motor name="motor2" joint="hinge2" ctrllimited="true" ctrlrange="-2 2"
           forcelimited="true" forcerange="-.5 .4"/>
    <!-- malformed limits will be ignored -->
    <motor name="motor3" joint="hinge3" ctrllimited="true" ctrlrange="2 1"
           forcelimited="true" forcerange="2 1"/>
  </actuator>
</mujoco>
)""";

  AddModelFromMujocoXml({DataSource::kContents, &xml}, "test", {}, &plant);
  plant.Finalize();

  EXPECT_EQ(plant.get_actuation_input_port().size(), 4);

  const JointActuator<double>& motor0 =
      plant.get_joint_actuator(JointActuatorIndex(0));
  EXPECT_EQ(motor0.name(), "motor0");
  EXPECT_EQ(motor0.joint().name(), "hinge0");
  EXPECT_EQ(motor0.effort_limit(), std::numeric_limits<double>::infinity());

  const JointActuator<double>& motor1 =
      plant.get_joint_actuator(JointActuatorIndex(1));
  EXPECT_EQ(motor1.name(), "motor1");
  EXPECT_EQ(motor1.joint().name(), "hinge1");
  EXPECT_EQ(motor1.effort_limit(), 2);

  const JointActuator<double>& motor2 =
      plant.get_joint_actuator(JointActuatorIndex(2));
  EXPECT_EQ(motor2.name(), "motor2");
  EXPECT_EQ(motor2.joint().name(), "hinge2");
  EXPECT_EQ(motor2.effort_limit(), .5);

  const JointActuator<double>& motor3 =
      plant.get_joint_actuator(JointActuatorIndex(3));
  EXPECT_EQ(motor3.name(), "motor3");
  EXPECT_EQ(motor3.joint().name(), "hinge3");
  EXPECT_EQ(motor3.effort_limit(), std::numeric_limits<double>::infinity());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
