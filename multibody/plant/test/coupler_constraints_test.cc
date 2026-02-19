#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace {

using drake::math::RigidTransformd;
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

GTEST_TEST(IcfBuilder, CouplerAcrossTrees) {
  for (double time_step : {0.0, 0.01}) {
    systems::DiagramBuilder<double> diagram_builder;
    auto [plant, scene_graph] =
        AddMultibodyPlantSceneGraph(&diagram_builder, time_step);

    Parser(&plant, "Pendulum1").AddModelsFromString(kRobotXml, "xml");
    Parser(&plant, "Pendulum2").AddModelsFromString(kRobotXml, "xml");
    // The coupler selects joints from different trees.
    plant.AddCouplerConstraint(plant.get_joint(JointIndex(0)),
                               plant.get_joint(JointIndex(3)), 0.8);
    plant.Finalize();

    auto diagram = diagram_builder.Build();
    systems::Simulator simulator(*diagram);
    systems::SimulatorConfig config{
        .integration_scheme = "cenic", .max_step_size = 0.1, .accuracy = 1e-3};
    ApplySimulatorConfig(config, &simulator);
    if (plant.is_discrete()) {
      // SAP coupler constraint is fine with arbitrary topology.
      simulator.AdvanceTo(10.0);
    } else {
      // CENIC coupler constraint is not. TODO(#23992): the limitation checked
      // in this test is a regression from SAP coupler constraints.
      DRAKE_EXPECT_THROWS_MESSAGE(simulator.AdvanceTo(10.0),
                                  "IcfBuilder: Couplers are only allowed "
                                  "within DoFs in the same tree.");
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
