#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/implicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::Box;
using geometry::SceneGraph;
using multibody::Parser;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::IntegratorBase;
using systems::Simulator;

namespace multibody {
namespace {

// This unit test loads a model for a box on a flat plane. The plane is the x-y
// plane with the z axis pointing up and gravity pointing down in the negative z
// direction.
// In this test we apply an external horizontal force for which the box is in
// stiction.
// The test then verifies the computed values of contact results.
class SlidingBoxTest : public ::testing::Test {
 public:
  // Creates a MultibodyPlant model with the given `discrete_period`
  // (discrete_period = 0 for a continuous model), runs a simulation to reach
  // steady state, and verifies contact results.
  void RunSimulationToSteadyStateAndVerifyContactResults(
      double discrete_period) {
    auto diagram = MakeBoxDiagram(discrete_period);
    const auto& plant = dynamic_cast<const MultibodyPlant<double>&>(
        diagram->GetSubsystemByName("plant"));

    // Sanity check for the model's size.
    EXPECT_EQ(plant.num_velocities(), 2);
    EXPECT_EQ(plant.num_positions(), 2);
    EXPECT_EQ(plant.num_actuators(), 1);
    EXPECT_EQ(plant.num_actuated_dofs(), 1);

    // Create a context for this system:
    std::unique_ptr<Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());
    plant.get_actuation_input_port().FixValue(&plant_context, applied_force_);

    Simulator<double> simulator(*diagram, std::move(diagram_context));
    simulator.set_publish_every_time_step(true);
    // Implicit integration does much better than the default RK3 for this
    // system with regularized friction. Otherwise RK3 requires a very small
    // accuracy setting and it is very costly.
    simulator.reset_integrator<systems::ImplicitEulerIntegrator<double>>();
    simulator.get_mutable_integrator().set_maximum_step_size(1e-3);
    simulator.Initialize();
    simulator.AdvanceTo(simulation_time_);

    auto VerifyContactResults = [&](const MultibodyPlant<double>& the_plant,
                                    const Context<double>& the_context) {
      // We evaluate the contact results output port in order to verify the
      // simulation results.
      const ContactResults<double>& contact_results =
          the_plant.get_contact_results_output_port()
              .Eval<ContactResults<double>>(the_context);

      // Only one contact pair.
      ASSERT_EQ(contact_results.num_point_pair_contacts(), 1);
      const PointPairContactInfo<double>& point_pair_contact_info =
          contact_results.point_pair_contact_info(0);

      // Verify the bodies referenced by the contact info.
      const Body<double>& ground = the_plant.GetBodyByName("ground");
      const Body<double>& box = the_plant.GetBodyByName("box");
      EXPECT_TRUE((point_pair_contact_info.bodyA_index() == box.index() &&
                   point_pair_contact_info.bodyB_index() == ground.index()) ||
                  (point_pair_contact_info.bodyB_index() == box.index() &&
                   point_pair_contact_info.bodyA_index() == ground.index()));

      // Whether the normal points up or down depends on the order in which
      // the geometry engine orders bodies in the contact pair. direction = +1
      // indicates the normal is the +z axis pointing into the box (i.e. the
      // outward normal to the ground plane). direction = -1 indicates the
      // outward normal to the box (i.e. pointing down in the -z direction).
      double direction =
          point_pair_contact_info.bodyB_index() == box.index() ? 1.0 : -1.0;

      // The expected value of the contact force applied on the box at the
      // contact point C.
      const Vector3<double> f_Bc_W(-applied_force_, 0.0, mass_ * g_);
      EXPECT_TRUE(CompareMatrices(point_pair_contact_info.contact_force(),
                                  f_Bc_W * direction, kTolerance,
                                  MatrixCompareType::relative));

      // Upper limit on the x displacement computed using the maximum possible
      // velocity under stiction, i.e. the stiction tolerance.
      const double x_upper_limit = stiction_tolerance_ * simulation_time_;
      EXPECT_LT(point_pair_contact_info.contact_point().x(), x_upper_limit);

      // The penetration allowance is just an estimate. Therefore we only
      // expect the penetration depth to be close enough to it.
      EXPECT_NEAR(point_pair_contact_info.point_pair().depth,
                  penetration_allowance_, 1.0e-3);

      // Whether the normal points up or down depends on the order in which
      // scene graph orders bodies in the contact pair.
      const Vector3<double> expected_normal =
          -Vector3<double>::UnitZ() * direction;
      EXPECT_TRUE(CompareMatrices(
          point_pair_contact_info.point_pair().nhat_BA_W, expected_normal,
          kTolerance, MatrixCompareType::relative));

      // If we are in stiction, the slip speed should be smaller than the
      // specified stiction tolerance.
      EXPECT_LT(point_pair_contact_info.slip_speed(), stiction_tolerance_);

      // There should not be motion in the normal direction.
      EXPECT_NEAR(point_pair_contact_info.separation_speed(), 0.0, kTolerance);
    };

    // Verify contact results at the end of the simulation.
    VerifyContactResults(plant, plant_context);

    // We want to verify that contact results can be evaluated simply from the
    // information stored in the context. To verify this we create a completely
    // new model of the same mechanical system and a new context for that
    // system. Therefore we don't run the risk of using previously computed
    // results in the simulation above. We only use the final state of that
    // simulation to set the new context. Thus contact results evaluation in the
    // following test is completely independent from the simulation above
    // (besides of course the initial condition).
    auto diagram2 = MakeBoxDiagram(discrete_period);
    const auto& plant2 = dynamic_cast<const MultibodyPlant<double>&>(
        diagram2->GetSubsystemByName("plant"));
    std::unique_ptr<Context<double>> diagram_context2 =
        diagram2->CreateDefaultContext();
    Context<double>& plant_context2 =
        diagram2->GetMutableSubsystemContext(plant2, diagram_context2.get());
    plant2.get_actuation_input_port().FixValue(&plant_context2, applied_force_);
    // Set the state from the computed solution.
    plant2.SetPositionsAndVelocities(
        &plant_context2, plant.GetPositionsAndVelocities(plant_context));
    VerifyContactResults(plant2, plant_context2);
  }

  // Creates a MultibodyPlant model with the given `discrete_period`
  // (discrete_period = 0 for a continuous model).
  std::unique_ptr<Diagram<double>> MakeBoxDiagram(double time_step) {
    DiagramBuilder<double> builder;
    const std::string full_name =
        FindResourceOrThrow("drake/multibody/plant/test/box.sdf");
    MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(
        &builder, std::make_unique<MultibodyPlant<double>>(time_step));
    plant.set_name("plant");
    Parser(&plant).AddModelFromFile(full_name);

    // Add gravity to the model.
    plant.mutable_gravity_field().set_gravity_vector(-g_ *
                                                     Vector3<double>::UnitZ());

    plant.Finalize();  // Done creating the model.

    // Set contact parameters.
    plant.set_penetration_allowance(penetration_allowance_);
    plant.set_stiction_tolerance(stiction_tolerance_);

    // And build the Diagram:
    return builder.Build();
  }

 protected:
  // Length of the simulation, in seconds.
  const double simulation_time_{0.5};

  // Plant's parameters.
  const double mass_{1.0};  // Box mass, [kg]
  // We use g = 10 m/s² so that numbers are simpler.
  const double g_{10.0};  // Acceleration of gravity, [m/s²]
  // Simulation parameters.
  const double time_step_{1.0e-3};              // in seconds.
  const double penetration_allowance_{1.0e-4};  // in meters.
  const double stiction_tolerance_{1.0e-4};     // in meters per second.
  const double applied_force_{5.0};             // Force in Newtons.

  // Tolerance used to verify the results.
  // Since the contact solver uses a continuous ODE to model Coulomb friction
  // (a modified Stribeck model), we simulate for a long enough time to reach
  // a "steady state". Therefore the precision of the results in these tests
  // is dominated for "how well we reached steady state".
  const double kTolerance{1.0e-12};
};

TEST_F(SlidingBoxTest, ContinuousModel) {
  RunSimulationToSteadyStateAndVerifyContactResults(1.0e-3);
}

TEST_F(SlidingBoxTest, DiscreteModel) {
  RunSimulationToSteadyStateAndVerifyContactResults(0.0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
