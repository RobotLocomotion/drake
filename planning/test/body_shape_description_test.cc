#include "planning/body_shape_description.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/robot_diagram_builder.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace planning {
namespace {

using geometry::GeometryId;
using geometry::Sphere;
using math::RigidTransformd;
using math::RollPitchYaw;
using multibody::MultibodyPlant;

GTEST_TEST(BodyShapeDescriptionTest, Arbitrary) {
  const Sphere sphere{1.23};
  const RigidTransformd pose(RollPitchYaw<double>{1, 3, 5},
                             Eigen::Vector3d{7, 9, 11});
  const std::string model{"acme_arm"};
  const std::string body{"link1"};
  BodyShapeDescription dut(sphere, pose, model, body);

  EXPECT_EQ(dynamic_cast<const Sphere&>(dut.shape()).radius(), sphere.radius());
  EXPECT_TRUE(dut.pose_in_body().IsExactlyEqualTo(pose));
  EXPECT_EQ(dut.model_instance_name(), model);
  EXPECT_EQ(dut.body_name(), body);
}

GTEST_TEST(BodyShapeDescriptionTest, MovedFrom) {
  const Sphere sphere{1.23};
  const RigidTransformd pose(RollPitchYaw<double>{1, 3, 5},
                             Eigen::Vector3d{7, 9, 11});
  const std::string model{"acme_arm"};
  const std::string body{"link1"};
  BodyShapeDescription dut(sphere, pose, model, body);
  BodyShapeDescription receiver{std::move(dut)};

  // We don't really care what the moved-from object contains, but we do want
  // behavior to be defined.
  EXPECT_THROW(dut.shape(), std::exception);
  EXPECT_TRUE(dut.pose_in_body().IsExactlyEqualTo(pose));
  EXPECT_TRUE(dut.model_instance_name().empty());
  EXPECT_TRUE(dut.body_name().empty());
}

GTEST_TEST(BodyShapeDescriptionTest, FromPlant) {
  planning::RobotDiagramBuilder<double> builder;
  const std::string model = R"""(
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='my_model'>
    <link name='link'>
      <visual name='link_visual'>
        <geometry>
          <sphere><radius>3.25</radius></sphere>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
)""";
  builder.mutable_parser().AddModelsFromString(model, "sdf");
  auto diagram = builder.BuildDiagram();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant = diagram->plant();
  auto& plant_context = diagram->plant_context(*diagram_context);
  auto& inspector = diagram->scene_graph().model_inspector();
  std::vector<GeometryId> geometries = inspector.GetAllGeometryIds();
  ASSERT_EQ(geometries.size(), 1);

  // A correct set of inputs yields a populated shape description.
  auto dut = MakeBodyShapeDescription(plant, plant_context, geometries[0]);
  EXPECT_EQ(dynamic_cast<const Sphere&>(dut.shape()).radius(), 3.25);
  EXPECT_TRUE(dut.pose_in_body().IsExactlyIdentity());
  EXPECT_EQ(dut.model_instance_name(), "my_model");
  EXPECT_EQ(dut.body_name(), "link");

  // Wrong context throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeBodyShapeDescription(plant, *diagram_context, geometries[0]),
      ".*instead of the appropriate subsystem Context.*\n.*");
  // Invalid GeometryId throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeBodyShapeDescription(plant, plant_context, {}),
      ".*is_valid.*failed.*");
  // GeometryId for nonexistent geometry throws.
  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeBodyShapeDescription(plant, plant_context, GeometryId::get_new_id()),
      ".*geometry.*has not been registered.*");
}

GTEST_TEST(BodyShapeDescriptionTest, NoSceneGraphThrows) {
  systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<MultibodyPlant<double>>(0.0);
  plant->Finalize();
  auto diagram = builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context = plant->GetMyContextFromRoot(*diagram_context);

  DRAKE_EXPECT_THROWS_MESSAGE(
      MakeBodyShapeDescription(*plant, plant_context, {}),
      ".*geometry_source_is_registered.*failed.*");
}

}  // namespace
}  // namespace planning
}  // namespace drake
