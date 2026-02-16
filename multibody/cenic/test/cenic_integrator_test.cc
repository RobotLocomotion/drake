#include "drake/multibody/cenic/cenic_integrator.h"

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace cenic {
namespace {

constexpr char kBallOnTableMjcf[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="robot">
    <worldbody>
      <geom name="floor" type="box" pos="0 0 -0.1" size="50 50 0.1" />
      <body name="link1" pos="0 0 1.0">
        <joint name="joint1" type="free"/>
        <geom type="sphere" size="0.1" pos="0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )""";

using Eigen::VectorXd;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::Simulator;

/* A base test case with utilities for setting up a CENIC simulation. */
class SimulationTestScenario : public testing::Test {
 protected:
  void SetUp() override { AddModels(); }

  /* Add models to the plant. */
  virtual void AddModels() = 0;

  /* Set defualt initial conditions. */
  virtual void SetInitialConditions() = 0;

  /* Creates the system diagram and sets up a simulation with CENIC. */
  void Build() {
    plant_.Finalize();
    diagram_ = builder_->Build();
    auto context = diagram_->CreateDefaultContext();

    // Set initial conditions.
    plant_context_ = &plant_.GetMyMutableContextFromRoot(context.get());
    SetInitialConditions();

    // Set up a simulator with the CENIC integrator.
    simulator_ =
        std::make_unique<Simulator<double>>(*diagram_, std::move(context));
    integrator_ = &simulator_->reset_integrator<CenicIntegrator<double>>();
  }

  // Only available prior to Build().
  std::unique_ptr<DiagramBuilder<double>> builder_{
      std::make_unique<DiagramBuilder<double>>()};

  // Only available after Build().
  std::unique_ptr<Diagram<double>> diagram_;
  Context<double>* plant_context_{nullptr};
  std::unique_ptr<Simulator<double>> simulator_;
  CenicIntegrator<double>* integrator_{nullptr};

  // Always available.
  MultibodyPlant<double>& plant_{
      AddMultibodyPlantSceneGraph(builder_.get(), 0.0).plant};
};

class BallOnTable : public SimulationTestScenario {
 protected:
  void AddModels() override {
    Parser(builder_.get()).AddModelsFromString(kBallOnTableMjcf, "xml");
  }

  void SetInitialConditions() override {
    plant_.SetVelocities(plant_context_, VectorXd::Zero(6));
  }
};

TEST_F(BallOnTable, NoCrashBuildSimulator) {
  Build();
  EXPECT_TRUE(integrator_ != nullptr);
}

}  // namespace
}  // namespace cenic
}  // namespace multibody
}  // namespace drake
