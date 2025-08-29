#include "drake/planning/body_shape_description.h"

#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/robot_diagram_builder.h"

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

  // A moved-from object should reject any attempt at using it.
  BodyShapeDescription other(std::move(dut));
  EXPECT_THROW(dut.shape(), std::exception);
  EXPECT_THROW(dut.pose_in_body(), std::exception);
  EXPECT_THROW(dut.model_instance_name(), std::exception);
  EXPECT_THROW(dut.body_name(), std::exception);
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
  builder.parser().AddModelsFromString(model, "sdf");
  auto diagram = builder.Build();
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
}

}  // namespace
}  // namespace planning
}  // namespace drake
