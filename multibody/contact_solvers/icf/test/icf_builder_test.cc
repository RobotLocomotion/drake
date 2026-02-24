#include "drake/multibody/contact_solvers/icf/icf_builder.h"

#include <limits>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {
namespace {

using drake::math::RigidTransformd;
using Eigen::Vector2d;
using Eigen::Vector3d;

constexpr double kInf = std::numeric_limits<double>::infinity();

constexpr char kRobotXml[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="robot">
    <worldbody>
      <body>
        <inertial mass="0.1" diaginertia="0.1 0.1 0.1"/>
        <joint name="joint1" type="hinge" axis="0 1 0" pos="0 0 0.1"/>
        <body>
          <inertial mass="0.1" diaginertia="0.1 0.1 0.1"/>
          <joint name="joint2" type="hinge" axis="0 1 0" pos="0 0 -0.1" range="-45.0 45.0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )""";

GTEST_TEST(IcfBuilder, Limits) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum1").AddModelsFromString(kRobotXml, "xml");
  Parser(&plant, "Pendulum2").AddModelsFromString(kRobotXml, "xml");
  plant.Finalize();
  EXPECT_EQ(plant.num_model_instances(), 4);  // Default, world, pendulums 1&2.
  EXPECT_EQ(plant.num_velocities(), 4);

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  const double time_step = 0.01;
  IcfBuilder<double> builder(&plant);
  IcfModel<double> model;
  builder.UpdateModel(plant_context, time_step, nullptr, nullptr, &model);
  EXPECT_EQ(model.num_cliques(), 2);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  EXPECT_EQ(model.num_limit_constraints(), 2);

  // Check the limit constraints produced.
  const auto& pool = model.limit_constraints_pool();
  for (int k = 0; k < model.num_limit_constraints(); ++k) {
    SCOPED_TRACE(fmt::format("index {}", k));
    EXPECT_EQ(pool.clique()[k], k);
    EXPECT_EQ(pool.constraint_size()[k], 2);
    ASSERT_EQ(ssize(pool.ql()[k]), 2);
    ASSERT_EQ(ssize(pool.qu()[k]), 2);
    ASSERT_EQ(ssize(pool.q0()[k]), 2);
    for (int dof = 0; dof < 2; ++dof) {
      SCOPED_TRACE(fmt::format("dof {}", dof));
      EXPECT_EQ(pool.q0()[k](dof), 0);
    }
    EXPECT_EQ(pool.ql()[k](0), -kInf);
    EXPECT_EQ(pool.qu()[k](0), kInf);
    EXPECT_EQ(pool.ql()[k](1), -M_PI / 4);
    EXPECT_EQ(pool.qu()[k](1), M_PI / 4);
  }
}

GTEST_TEST(IcfBuilder, Coupler) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum1").AddModelsFromString(kRobotXml, "xml");
  Parser(&plant, "Pendulum2").AddModelsFromString(kRobotXml, "xml");
  plant.AddCouplerConstraint(plant.get_joint(JointIndex(0)),
                             plant.get_joint(JointIndex(1)), 0.8);
  plant.Finalize();

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  const double time_step = 0.01;
  IcfBuilder<double> builder(&plant);
  IcfModel<double> model;
  builder.UpdateModel(plant_context, time_step, nullptr, nullptr, &model);
  EXPECT_EQ(model.num_cliques(), 2);
  EXPECT_EQ(model.num_velocities(), plant.num_velocities());
  ASSERT_EQ(model.num_coupler_constraints(), 1);

  // Check the coupler constraint produced.
  const auto& pool = model.coupler_constraints_pool();
  EXPECT_EQ(pool.constraint_to_clique()[0], 0);
  EXPECT_EQ(pool.dofs()[0].first, 0);
  EXPECT_EQ(pool.dofs()[0].second, 1);
  EXPECT_EQ(pool.gear_ratio()[0], 0.8);
}

// TODO(#23992): the limitation checked in this test is a regression from SAP
// coupler constraints.
GTEST_TEST(IcfBuilder, CouplerBad) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum1").AddModelsFromString(kRobotXml, "xml");
  Parser(&plant, "Pendulum2").AddModelsFromString(kRobotXml, "xml");
  // The bad coupler selects joints from different trees.
  plant.AddCouplerConstraint(plant.get_joint(JointIndex(0)),
                             plant.get_joint(JointIndex(3)), 0.8);
  plant.Finalize();

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  const double time_step = 0.01;
  IcfBuilder<double> builder(&plant);
  IcfModel<double> model;
  DRAKE_EXPECT_THROWS_MESSAGE(
      builder.UpdateModel(plant_context, time_step, nullptr, nullptr, &model),
      ".*only.*same tree.*");
}

GTEST_TEST(IcfBuilder, RetryStep) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};

  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(kRobotXml, "xml");
  plant.AddJointActuator("elbow", plant.GetJointByName("joint2"));
  // The second pendulum is unactuated, to test fewer actuators than cliques.
  Parser(&plant, "Pendulum2").AddModelsFromString(kRobotXml, "xml");
  plant.Finalize();
  EXPECT_EQ(plant.num_velocities(), 4);

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*diagram_context);

  IcfLinearFeedbackGains<double> feedback;
  feedback.K.resize(plant.num_velocities());
  feedback.K << 0.01, 0.0101, 0.0102, 0.0103;
  feedback.b.resize(plant.num_velocities());
  feedback.b << 0.02, 0.0201, 0.0202, 0.0203;
  const double time_step = 0.01;
  IcfBuilder<double> builder(&plant);
  IcfModel<double> model;
  VectorX<double> v;
  v.setConstant(plant.num_velocities(), 0.03);
  const auto& pool = model.gain_constraints_pool();

  auto check_gain_constraints = [&](IcfLinearFeedbackGains<double>*
                                        actuation_feedback,
                                    IcfLinearFeedbackGains<double>*
                                        external_feedback) {
    // This test model has two pendulums, and so two cliques.  External
    // forces generate one constraint per clique. For actuation, there is
    // only one, since the second pendulum is unactuated.
    ASSERT_EQ(model.num_cliques(), 2);
    EXPECT_EQ(pool.num_constraints(), 2 * (external_feedback != nullptr) +
                                          (actuation_feedback != nullptr));
    // Make a vector of flags predicting which constraints are actuation.
    std::vector<int> is_actuation(pool.num_constraints(), 0);
    if (actuation_feedback != nullptr) {
      is_actuation[pool.num_constraints() - 1] = 1;
    }
    for (int k = 0; k < pool.num_constraints(); ++k) {
      SCOPED_TRACE(fmt::format("index {}", k));
      if (is_actuation[k]) {
        EXPECT_EQ(pool.clique()[k], 0);
        EXPECT_TRUE(CompareMatrices(pool.K()[k], feedback.K.head(2)));
        EXPECT_TRUE(CompareMatrices(pool.b()[k], feedback.b.head(2)));
        EXPECT_TRUE(CompareMatrices(pool.le()[k], Vector2d::Constant(-kInf)));
        EXPECT_TRUE(CompareMatrices(pool.ue()[k], Vector2d::Constant(kInf)));
      } else {
        EXPECT_EQ(pool.clique()[k], k);
        EXPECT_TRUE(CompareMatrices(pool.K()[k], feedback.K.segment(2 * k, 2)));
        EXPECT_TRUE(CompareMatrices(pool.b()[k], feedback.b.segment(2 * k, 2)));
        EXPECT_TRUE(CompareMatrices(pool.le()[k], Vector2d::Constant(-kInf)));
        EXPECT_TRUE(CompareMatrices(pool.ue()[k], Vector2d::Constant(kInf)));
      }
      EXPECT_EQ(pool.constraint_size()[k], 2);
    }
  };

  // Run a long step (pretending it failed error bounds) and then a "retry
  // step", for all combinations of feedback parameters. The retry step should
  // be equivalent to having done a full step of the same duration.
  for (int k = 0; k < 4; ++k) {
    const bool has_actuation = (k & 1) != 0;
    const bool has_external = (k & 2) != 0;
    SCOPED_TRACE(
        fmt::format("actuation {}, external {}", has_actuation, has_external));
    IcfLinearFeedbackGains<double>* actuation_feedback =
        has_actuation ? &feedback : nullptr;
    IcfLinearFeedbackGains<double>* external_feedback =
        has_external ? &feedback : nullptr;

    // Do a long step to populate all internals.
    builder.UpdateModel(plant_context, 2 * time_step, actuation_feedback,
                        external_feedback, &model);
    check_gain_constraints(actuation_feedback, external_feedback);
    // Do the "Retry step."
    model.UpdateTimeStep(time_step);
    builder.UpdateFeedbackGains(actuation_feedback, external_feedback, &model);
    check_gain_constraints(actuation_feedback, external_feedback);
    IcfData<double> data1;
    model.ResizeData(&data1);
    model.CalcData(v, &data1);

    // Do an equivalent step to produce the expected values.
    builder.UpdateModel(plant_context, time_step, actuation_feedback,
                        external_feedback, &model);
    check_gain_constraints(actuation_feedback, external_feedback);
    IcfData<double> data2;
    model.ResizeData(&data2);
    model.CalcData(v, &data2);

    // Resulting data should match.
    EXPECT_EQ(data1.v(), data2.v());
    EXPECT_EQ(data1.Av(), data2.Av());
    EXPECT_EQ(data1.momentum_cost(), data2.momentum_cost());
    EXPECT_EQ(data1.cost(), data2.cost());
    EXPECT_EQ(data1.gradient(), data2.gradient());
    ASSERT_EQ(data1.V_WB().size(), data2.V_WB().size());
    for (int m = 0; m < data1.V_WB().size(); ++m) {
      EXPECT_TRUE(CompareMatrices(data1.V_WB()[m], data2.V_WB()[m]));
    }
  }
}

GTEST_TEST(IcfBuilder, BallConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};
  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(kRobotXml, "xml");

  plant.AddBallConstraint(plant.get_body(BodyIndex(0)), Vector3d::Zero(),
                          plant.get_body(BodyIndex(1)));

  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(&plant),
                              ".*not.*support.*1 ball constraint.*");
}

GTEST_TEST(IcfBuilder, DistanceConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};
  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(kRobotXml, "xml");

  plant.AddDistanceConstraint(plant.get_body(BodyIndex(0)), Vector3d::Zero(),
                              plant.get_body(BodyIndex(1)), Vector3d::Zero(),
                              0.01);
  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(&plant),
                              ".*not.*support.*1 distance constraint\\(s\\).*");
}

GTEST_TEST(IcfBuilder, TendonConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};
  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(kRobotXml, "xml");

  plant.AddTendonConstraint({JointIndex{0}}, {0.01}, {}, {{-1.0}}, {}, {}, {});

  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(&plant),
                              ".*not.*support.*1 tendon constraint\\(s\\).*");
}

GTEST_TEST(IcfBuilder, WeldConstraintUnsupported) {
  systems::DiagramBuilder<double> diagram_builder;
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.0};
  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(kRobotXml, "xml");

  plant.AddWeldConstraint(plant.get_body(BodyIndex(0)), RigidTransformd(),
                          plant.get_body(BodyIndex(1)), RigidTransformd());
  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(&plant),
                              ".*not.*support.*1 weld constraint\\(s\\).*");
}

GTEST_TEST(IcfBuilder, DeformableUnsupported) {
  systems::DiagramBuilder<double> diagram_builder;
  // TODO(#23768): MultibodyPlant (specifically, DeformableModel) does not yet
  // support deformable bodies on continuous plants. Internally, ICF doesn't
  // much care about the plant's discrete/continuous configuration.
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};
  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  plant.mutable_deformable_model().RegisterDeformableBody(
      std::make_unique<geometry::GeometryInstance>(
          RigidTransformd(), geometry::Sphere(1.0), "ball"),
      fem::DeformableBodyConfig<double>(), 1.0);
  plant.Finalize();

  DRAKE_EXPECT_THROWS_MESSAGE(IcfBuilder<double>(&plant),
                              ".*deformable.*bodies.* == 0.*fail.*");
}

GTEST_TEST(IcfBuilder, JointLockingUnsupported) {
  systems::DiagramBuilder<double> diagram_builder;
  // TODO(#23764): MultibodyPlant does not yet support joint locking on
  // continuous plants. Internally, ICF doesn't much care about the plant's
  // discrete/continuous configuration.
  multibody::MultibodyPlantConfig plant_config{.time_step = 0.1};
  MultibodyPlant<double>& plant =
      multibody::AddMultibodyPlant(plant_config, &diagram_builder);

  Parser(&plant, "Pendulum").AddModelsFromString(kRobotXml, "xml");
  // Remove a joint to exercise non-contiguous joint indexing.
  plant.RemoveJoint(plant.get_joint(JointIndex(0)));

  plant.Finalize();
  IcfBuilder<double> dut(&plant);

  auto diagram = diagram_builder.Build();
  auto diagram_context = diagram->CreateDefaultContext();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  IcfModel<double> model;
  // Bug regression check: don't accidentally throw by mistakenly iterating
  // over stale joint indices.
  EXPECT_NO_THROW(
      dut.UpdateModel(plant_context, 0.01, nullptr, nullptr, &model));

  plant.get_joint(JointIndex(1)).Lock(&plant_context);

  // Actual joint locking check: now something is locked, refuse to give wrong
  // answers, and explain that joint locking is the problem.
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.UpdateModel(plant_context, 0.01, nullptr, nullptr, &model),
      ".*joint 1.*locked.*");
}

}  // namespace
}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
