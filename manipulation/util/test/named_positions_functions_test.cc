#include "drake/manipulation/util/named_positions_functions.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/revolute_joint.h"

using drake::multibody::BallRpyJoint;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;

namespace drake {
namespace manipulation {
namespace {

GTEST_TEST(ApplyNamedPositionsAsDefaultsTest, Basic) {
  MultibodyPlant<double> plant(0.01);
  Parser(&plant).AddModelsFromUrl(
      "package://drake/multibody/benchmarks/acrobot/acrobot.sdf");
  plant.Finalize();

  // Set one initial joint position by name.
  NamedPositions input;
  input["acrobot"]["ShoulderJoint"] = drake::Vector1d{0.2};
  ApplyNamedPositionsAsDefaults(input, &plant);

  // The requested default value appears in the default context.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  const auto& shoulder = plant.GetJointByName<RevoluteJoint>("ShoulderJoint");
  EXPECT_EQ(shoulder.get_angle(*context), 0.2);

  // The other joint was unaffected by the Apply function.
  const auto& elbow = plant.GetJointByName<RevoluteJoint>("ElbowJoint");
  EXPECT_EQ(elbow.get_angle(*context), 0.0);

  // Try setting a joint with the wrong number of positions. Be somewhat picky
  // about the resulting message.
  input["acrobot"]["ShoulderJoint"] = Eigen::Vector3d::Zero();
  DRAKE_EXPECT_THROWS_MESSAGE(ApplyNamedPositionsAsDefaults(input, &plant),
                              ".*set_default_positions.*positions.*"
                              "input.*3.*not.*acrobot::ShoulderJoint.*1.*");
}

GTEST_TEST(ApplyNamedPositionsAsDefaultsTest, MultiDof) {
  MultibodyPlant<double> plant(0.01);
  Parser(&plant).AddModelsFromUrl(
      "package://drake_models/manipulation_station/cylinder.sdf");
  plant.template AddJoint<BallRpyJoint>("ball", plant.world_body(), {},
                                        plant.GetBodyByName("base_link"), {});
  plant.Finalize();

  // Set the initial joint position by name.
  NamedPositions input;
  const Eigen::Vector3d rpy{0.1, 0.2, 0.3};
  input["cylinder"]["ball"] = rpy;
  ApplyNamedPositionsAsDefaults(input, &plant);

  // The requested default value appears in the default context.
  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();
  const auto& ball = plant.GetJointByName<BallRpyJoint>("ball");
  EXPECT_EQ(ball.get_angles(*context), rpy);

  // Try setting a joint with the wrong number of positions. Be somewhat picky
  // about the resulting message.
  input["cylinder"]["ball"] = Eigen::Vector4d::Zero();
  DRAKE_EXPECT_THROWS_MESSAGE(ApplyNamedPositionsAsDefaults(input, &plant),
                              ".*set_default_positions.*positions.*"
                              "input.*4.*not.*cylinder::ball.*3.*");
}

}  // namespace
}  // namespace manipulation
}  // namespace drake
