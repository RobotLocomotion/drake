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

    VectorX<T> hover_input(plant_->get_input_size());
    hover_input.setZero();

    systems::DrakeVisualizer* publisher =
        builder.template AddSystem<systems::DrakeVisualizer>(
            plant_->get_rigid_body_tree(), &lcm_);

    builder.Connect(plant_->get_output_port(0), publisher->get_input_port(0));

    builder.BuildInto(this);
  }

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    systems::Diagram<T>::SetDefaultState(context, state);
    systems::State<T>& plant_state =
        this->GetMutableSubsystemState(*plant_, state);
    VectorX<T> x0(plant_->get_num_states());
    x0.setZero();
    /* x0 is the initial state where
     * x0(0), x0(1), x0(2) are the quadrotor's x, y, z -states
     * x0(3), x0(4), x0(5) are the quedrotor's Euler angles phi, theta, psi
     */
    x0(2) = 0.2;  // Sets arbitrary z-position. This is the initial height of
                  // the quadrotor in the world frame.
    plant_->set_state_vector(&plant_state, x0);
  }

 const systems::RigidBodyPlant<T>& get_plant() const { return *plant_; }

 private:
  systems::RigidBodyPlant<T>* plant_{};
  lcm::DrakeLcm lcm_;
};

// Verifies that the output of the time stepping rigid body plant and the
// continuous rigid body plant are equal with 
GTEST_TEST(QuadrotorTest, Equality) {
  // Set the simulation duration.
  const double duration = 1.0;

  // Set the step size.
  const double step_size = 1e-3;

  // Construct the two models.
  Quadrotor<double> continuous_model(0.0);
  Quadrotor<double> discrete_model(step_size);

  // Construct two simulators for those models.
  systems::Simulator<double> continuous_sim(continuous_model);
  systems::Simulator<double> discrete_sim(discrete_model);

  // Make them use the same "integrator".
  continuous_sim.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
      continuous_model, step_size, continuous_sim.get_mutable_context());

  // Initialize the simulators.
  continuous_sim.Initialize();
  discrete_sim.Initialize();

  // Step both forward by ten seconds (i.e., a long time into the future).
  continuous_sim.StepTo(duration);
  discrete_sim.StepTo(duration);

  // Compare the states.
  auto& continuous_state = continuous_sim.get_context().get_continuous_state_vector();
  auto& discrete_state = discrete_sim.get_context().get_discrete_state_vector();
  ASSERT_EQ(continuous_state.size(), discrete_state.size());
  for (int i = 0; i < discrete_state.size(); ++i) {
    EXPECT_NEAR(discrete_state[i], continuous_state[i],
      10 * std::numeric_limits<double>::epsilon());
  }
}

}  // namespace
}  // namespace multibody 
}  // namespace drake

