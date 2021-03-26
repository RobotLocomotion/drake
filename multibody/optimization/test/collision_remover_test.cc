#include "drake/multibody/optimization/collision_remover.h"

#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/common/find_resource.h"

using Contextd = drake::systems::Context<double>;
using MultibodyPlantd = drake::multibody::MultibodyPlant<double>;
using RigidTransformd = drake::math::RigidTransform<double>;

namespace drake {
namespace multibody {
namespace {

class CollisionRemoverFixture : public ::testing::Test {
 public:
  CollisionRemoverFixture() {
    drake::systems::DiagramBuilder<double> builder;
    std::tie(plant, scene_graph) =
        drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.001);
    drake::multibody::Parser(plant, scene_graph)
        .AddAllModelsFromFile(FindResourceOrThrow(
            "drake/multibody/optimization/test/collision_remover_test.urdf"));
    plant->Finalize();
    diagram = builder.Build();
    pendulum_1 = plant->GetBodyByName("pendulum_1_arm").index();
    pendulum_2 = plant->GetBodyByName("pendulum_2_arm").index();
    brick = plant->GetBodyByName("floating_brick").index();
    dut = std::make_unique<CollisionRemover>(
        diagram.get(), plant, scene_graph);
    known_valid_position = plant->GetPositions(
        diagram->GetSubsystemContext(*plant, *diagram->CreateDefaultContext()));
  }
  ~CollisionRemoverFixture() {}

  std::unique_ptr<drake::systems::Diagram<double>> diagram;
  MultibodyPlantd* plant = nullptr;
  drake::geometry::SceneGraph<double>* scene_graph = nullptr;
  std::unique_ptr<CollisionRemover> dut;
  BodyIndex pendulum_1{};
  BodyIndex pendulum_2{};
  BodyIndex brick{};
  drake::VectorX<double> known_valid_position{};
};

// Check the fixture (and the collision checker) to ensure that we get the
// collisions we expect.  Otherwise none of our other tests would mean much.
TEST_F(CollisionRemoverFixture, FixtureSmokeTest) {
  auto root_context = diagram->CreateDefaultContext();
  Contextd& plant_context =
      diagram->GetMutableSubsystemContext(*plant, root_context.get());
  EXPECT_FALSE(dut->Collides(plant_context, std::nullopt));

  // Turn the left pendulum to collide with the right pendulum and the brick.
  plant->GetJointByName<RevoluteJoint>("pendulum_1_theta")
      .set_angle(&plant_context, -1.5);
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, pendulum_2}}));
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, brick}}));
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_2, brick}}));
  EXPECT_TRUE(dut->Collides(plant_context, std::nullopt));

  // Turn the right pendulum to a parallel position, so there is no collision
  // between the pendula but the brick is still in collision.
  plant->GetJointByName<RevoluteJoint>("pendulum_2_theta")
      .set_angle(&plant_context, -1.5);
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_1, pendulum_2}}));
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, brick}}));
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_2, brick}}));
  EXPECT_TRUE(dut->Collides(plant_context, std::nullopt));

  // Move the brick out of collision.
  plant->SetFreeBodyPose(
      &plant_context, plant->get_body(brick),
      RigidTransformd(Eigen::Vector3d{0., 0., 1.}));
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_1, pendulum_2}}));
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_1, brick}}));
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_2, brick}}));
  EXPECT_FALSE(dut->Collides(plant_context, std::nullopt));
}

// Raise one pendulum into collision; let AdjustPositions rotate it out of
// collision.
TEST_F(CollisionRemoverFixture, SingleJointTest) {
  auto root_context = diagram->CreateDefaultContext();
  Contextd& plant_context =
      diagram->GetMutableSubsystemContext(*plant, root_context.get());
  const RevoluteJoint<double>& theta_joint =
      plant->GetJointByName<RevoluteJoint>("pendulum_1_theta");
  theta_joint.set_angle(&plant_context, -1.5);

  EXPECT_TRUE(dut->Collides(plant_context, std::nullopt));
  dut->AdjustPositions(root_context.get(), {theta_joint.index()}, {},
                       std::nullopt,
                       known_valid_position);
  EXPECT_FALSE(dut->Collides(plant_context, std::nullopt));
}

// Move the brick into collision; let AdjustPositions slide it out of
// collision.
TEST_F(CollisionRemoverFixture, SingleBodyTest) {
  auto root_context = diagram->CreateDefaultContext();
  Contextd& plant_context =
      diagram->GetMutableSubsystemContext(*plant, root_context.get());
  plant->SetFreeBodyPose(&plant_context, plant->get_body(brick),
                         RigidTransformd(Eigen::Vector3d{-0.24, 0., 0.}));

  EXPECT_TRUE(dut->Collides(plant_context, std::nullopt));
  dut->AdjustPositions(root_context.get(), {}, {brick},
                       std::nullopt,
                       known_valid_position);
  EXPECT_FALSE(dut->Collides(plant_context, std::nullopt));
}

// Bring all three objects into collision.  Resolve the collision one element
// at a time (this is the preferred use case).
TEST_F(CollisionRemoverFixture, StepByStepTest) {
  auto root_context = diagram->CreateDefaultContext();
  Contextd& plant_context =
      diagram->GetMutableSubsystemContext(*plant, root_context.get());
  const RevoluteJoint<double>& theta_1_joint =
      plant->GetJointByName<RevoluteJoint>("pendulum_1_theta");
  const RevoluteJoint<double>& theta_2_joint =
      plant->GetJointByName<RevoluteJoint>("pendulum_2_theta");
  theta_1_joint.set_angle(&plant_context, -1.5);
  theta_2_joint.set_angle(&plant_context, 1.5);

  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, pendulum_2}}));
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, brick}}));
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_2, brick}}));

  EXPECT_TRUE(dut->Collides(plant_context, std::nullopt));

  dut->AdjustPositions(root_context.get(), {theta_1_joint.index()}, {},
                       {{pendulum_1, pendulum_2}},
                       known_valid_position);
  EXPECT_FALSE(dut->Collides(plant_context, {{pendulum_1, pendulum_2}}));

  dut->AdjustPositions(root_context.get(), {}, {brick},
                       std::nullopt,
                       known_valid_position);
  EXPECT_FALSE(dut->Collides(plant_context, std::nullopt));
}

// Bring all three objects into collision.  Resolve the collisions in a single
// call (this is a common starting case, although it is expensive and not
// usually what you want to end up with).
TEST_F(CollisionRemoverFixture, BigBangTest) {
  auto root_context = diagram->CreateDefaultContext();
  Contextd& plant_context =
      diagram->GetMutableSubsystemContext(*plant, root_context.get());
  const RevoluteJoint<double>& theta_1_joint =
      plant->GetJointByName<RevoluteJoint>("pendulum_1_theta");
  const RevoluteJoint<double>& theta_2_joint =
      plant->GetJointByName<RevoluteJoint>("pendulum_2_theta");
  theta_1_joint.set_angle(&plant_context, -1.5);
  theta_2_joint.set_angle(&plant_context, 1.5);

  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, pendulum_2}}));
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_1, brick}}));
  EXPECT_TRUE(dut->Collides(plant_context, {{pendulum_2, brick}}));

  EXPECT_TRUE(dut->Collides(plant_context, std::nullopt));
  dut->AdjustPositions(root_context.get(),
                       {theta_1_joint.index(), theta_1_joint.index()},
                       {brick},
                       {},
                       known_valid_position);
  EXPECT_FALSE(dut->Collides(plant_context, std::nullopt));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
