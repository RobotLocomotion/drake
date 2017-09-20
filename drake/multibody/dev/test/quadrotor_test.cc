/// @file
///
/// This demo sets up a passive Quadrotor plant in a world described by the
/// warehouse model. The robot simply rests on the floor within the walls
/// of the warehouse.

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/dev/time_stepping_rigid_body_plant.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {

using multibody::joints::kFixed;
using multibody::joints::kRollPitchYaw;
using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;
using parsers::sdf::AddModelInstancesFromSdfFile;
using systems::InputPortDescriptor;
using systems::RigidBodyPlant;
using systems::TimeSteppingRigidBodyPlant;
using systems::Context;

namespace multibody {
namespace {

template <typename T>
class Quadrotor : public systems::Diagram<T> {
 public:
  Quadrotor(double dt) {
    this->set_name("Quadrotor");

    auto tree = std::make_unique<RigidBodyTree<T>>();
    ModelInstanceIdTable model_id_table = AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow("drake/examples/quadrotor/quadrotor.urdf"),
        kRollPitchYaw, tree.get());
    const int quadrotor_id = model_id_table.at("quadrotor");
    AddModelInstancesFromSdfFile(
        FindResourceOrThrow("drake/examples/quadrotor/warehouse.sdf"),
        kFixed, nullptr /* weld to frame */, tree.get());
    drake::multibody::AddFlatTerrainToWorld(tree.get());

    systems::DiagramBuilder<T> builder;

    if (dt == 0.0) {
      plant_ = builder.template AddSystem<RigidBodyPlant<T>>(
          std::move(tree));
    } else {
      plant_ = builder.template AddSystem<TimeSteppingRigidBodyPlant<T>>(
          std::move(tree), dt);
    }

    plant_->set_name("plant");

    // Verifies that the quadrotor has no actuators.
    DRAKE_DEMAND(plant_->get_num_actuators() == 0);
    DRAKE_DEMAND(plant_->get_num_actuators(quadrotor_id) == 0);

    builder.BuildInto(this);
  }

 const systems::RigidBodyPlant<T>& get_plant() const { return *plant_; }

 private:
  systems::RigidBodyPlant<T>* plant_{};
};

// Verifies that the output of the time stepping rigid body plant and the
// continuous rigid body plant are equal with 
GTEST_TEST(QuadrotorTest, Equality) {
  // Set the step size.
  const double step_size = 2.5e-4;

  // Construct the two models.
  Quadrotor<double> continuous_model(0.0);
  Quadrotor<double> discrete_model(step_size);

  // Get the two RigidBodyPlant refs.
  const RigidBodyPlant<double>& continuous_plant = continuous_model.get_plant();
  const RigidBodyPlant<double>& discrete_plant = discrete_model.get_plant();

  // Construct two simulators for those models.
  systems::Simulator<double> continuous_sim(continuous_model);
  systems::Simulator<double> discrete_sim(discrete_model);

  // Make them use the same "integrator".
  continuous_sim.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      continuous_model, step_size, continuous_sim.get_mutable_context());

  // Initialize the simulators.
  continuous_sim.Initialize();
  discrete_sim.Initialize();

  // Set the initial conditions for the continuous plant. We modify the height
  // so that there are no contact forces.
  const int kQuadrotorStateDim = 12;
  VectorX<double> x0(kQuadrotorStateDim);
  for (int i = 0; i < kQuadrotorStateDim; ++i)
    x0[i] = i + 1; 
  auto& continuous_context = continuous_model.GetMutableSubsystemContext(
    continuous_plant, continuous_sim.get_mutable_context());
  continuous_context.get_mutable_continuous_state()->SetFromVector(x0);

  // Set the initial conditions for the discrete plant.
  Context<double>& discrete_context = discrete_model.GetMutableSubsystemContext(
      discrete_plant, discrete_sim.get_mutable_context());
  discrete_context.get_mutable_discrete_state()->get_mutable_vector()->
      SetFromVector(x0);

  // Step both forward by one step into the future.
  const double duration = step_size; 
  continuous_sim.StepTo(duration);
  discrete_sim.StepTo(duration);

  // Compare the states.
  auto& continuous_state = continuous_sim.get_context().get_continuous_state_vector();
  auto& discrete_state = *discrete_sim.get_context().get_discrete_state()->get_data().front();

  ASSERT_EQ(continuous_state.size(), discrete_state.size());

  // Compare solutions.
  for (int i = 0; i < discrete_state.size(); ++i) {
    EXPECT_NEAR(discrete_state[i], continuous_state[i],
      1e3 * std::numeric_limits<double>::epsilon());
  }
}

}  // namespace
}  // namespace multibody 
}  // namespace drake

