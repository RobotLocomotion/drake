#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/multibody/multibody_tree/joints/prismatic_joint.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_plant/multibody_plant.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

namespace drake {

using systems::Context;
using systems::Simulator;

namespace multibody {
namespace multibody_plant {
namespace {

// This parametrized fixture allows us to run inclined planes tests using either
// a continuous plant model or a discrete plant model.
class JointLimitsTest : public ::testing::TestWithParam<bool> {
 public:
  void SetUp() override {
    // If "true" the plant is modeled as a discrete system with periodic
    // updates.
    time_stepping_ = GetParam();

    // The period of the periodic updates for the discrete plant model or zero
    // when the plant is modeled as a continuous system.
    time_step_ = time_stepping_ ? 1.0e-3 : 0.0;

    

    // Relative tolerance (unitless) used to verify the numerically computed
    // results against the analytical solution.
    // Notice we can use a much tighter tolerance with the time-stepping
    // approach given that both the penetration allowance and the stiction
    // tolerance values are much smaller than those used for the continuous
    // model of the plant.
    relative_tolerance_ = time_stepping_ ? 5.5e-4 : 5.5e-3;
  }

 protected:
  bool time_stepping_;
  double time_step_{0};  // in seconds.
  double penetration_allowance_{1.0e-3};  // in meters.
  double stiction_tolerance_{1.0e-3};  // in meters per second.
  double relative_tolerance_{1.0e-3};  // dimensionless.
};

// This test creates a simple multibody model of a sphere rolling down an
// inclined plane. After simulating the model for a given length of time, this
// test verifies the numerical solution against analytical results obtained from
// an energy conservation analysis.
TEST_P(JointLimitsTest, PrismaticJoint) {
  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 1.0e-6;

  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double mass = 1.0;      // Mass of the body, [kg]
  const double box_size = 0.3;  // The size of the box shaped body, [m].
  //const double g = 9.81;        // Acceleration of gravity, [m/s²]

  MultibodyPlant<double> plant(time_step_);
  //const MultibodyTree<double>& model = plant.model();
  const auto M_B = SpatialInertia<double>::MakeFromCentralInertia(
          mass, Vector3<double>::Zero(),
          mass * UnitInertia<double>::SolidBox(box_size, box_size, box_size));
  const RigidBody<double>& body = plant.AddRigidBody("Body", M_B);
  const PrismaticJoint<double>& slider = plant.AddJoint<PrismaticJoint>(
      "Slider", plant.world_body(), {}, body, {}, Vector3<double>::UnitZ(),
      0.0 /* damping */, 0.0 /* lower limit */, 1.0 /* upper limit */);
  plant.AddJointActuator("ForceAlongZ", slider);
  plant.Finalize();

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 1);
  DRAKE_DEMAND(plant.num_positions() == 1);

  Simulator<double> simulator(plant);
  simulator.get_mutable_integrator()->set_target_accuracy(target_accuracy);
  Context<double>& context = simulator.get_mutable_context();
  context.FixInputPort(0, Vector1<double>::Constant(-10.0));
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  PRINT_VAR(slider.get_translation(context));
  PRINT_VAR(slider.get_translation_rate(context));
}

// Instantiate the tests.
INSTANTIATE_TEST_CASE_P(ContinuousAndTimeSteppingTest, JointLimitsTest,
                        ::testing::Bool());

}  // namespace
}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

