#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::SceneGraph;
using systems::BasicVector;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::IntegratorBase;
using systems::Simulator;

namespace multibody {
namespace {

// This parameterized fixture allows us to run inclined planes tests using
// either a continuous plant model or a discrete plant model.
class InclinedPlaneTest : public ::testing::TestWithParam<bool> {
 public:
  void SetUp() override {
    // If "true" the plant is modeled as a discrete system with periodic
    // updates.
    time_stepping_ = GetParam();

    // The period of the periodic updates for the discrete plant model or zero
    // when the plant is modeled as a continuous system.
    time_step_ = time_stepping_ ? 1.0e-3 : 0.0;

    // Contact parameters. Results converge to the analytical solution as the
    // penetration allowance and the stiction tolerance go to zero.
    // Since the discrete system uses an implicit scheme, we can use much
    // tighter contact parameters than those used with a continuous plant model.

    // The penetration allowance in meters.
    penetration_allowance_ = time_stepping_ ? 1.0e-6 : 1.0e-3;
    // The stiction tolerance in meters per second.
    stiction_tolerance_ = time_stepping_ ? 1.0e-5 : 1.0e-3;

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

// This test creates a simple multibody model of a sphere B rolling down an
// inclined-plane A. After simulating the model for a given length of time, this
// test verifies the numerical solution against analytical results obtained from
// an energy conservation analysis.
TEST_P(InclinedPlaneTest, RollingSphereTest) {
  DiagramBuilder<double> builder;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 1.0e-6;

  // Length of the simulation, in seconds.
  const double simulation_time = 1.0;

  // Plant's parameters.
  const double radius = 0.05;   // Rolling sphere radius, [m]
  const double mass = 0.1;      // Rolling sphere mass, [kg]
  const double gravity = 9.81;  // Acceleration of gravity, [m/s²]
  const double slope_radians = 15.0 / 180 * M_PI;  // Inclined-plane slope.
  const bool is_inclined_plane_half_space = true;
  const double muS_inclined_plane = 1.0;  // Coefficient static friction.
  const double muK_inclined_plane = 0.5;  // Coefficient kinetic friction.
  const double muS_sphere = 1.0;          // Coefficient static friction.
  const double muK_sphere = 0.5;          // Coefficient kinetic friction.
  const CoulombFriction<double> coefficient_friction_inclined_plane(
      muS_inclined_plane, muK_inclined_plane);
  const CoulombFriction<double> coefficient_friction_sphere(
      muS_sphere, muK_sphere);

  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(
      &builder, std::make_unique<MultibodyPlant<double>>(time_step_));
  benchmarks::inclined_plane::AddInclinedPlaneWithSpherePlant(
    gravity, slope_radians, is_inclined_plane_half_space, 0, 0, 0, radius, mass,
    coefficient_friction_inclined_plane, coefficient_friction_sphere, &plant);

  plant.Finalize();
  // Set how much penetration (in meters) we are willing to accept.
  plant.set_penetration_allowance(penetration_allowance_);
  plant.set_stiction_tolerance(stiction_tolerance_);
  const RigidBody<double>& ball = plant.GetRigidBodyByName("BodyB");

  // Sanity check for the model's size.
  DRAKE_DEMAND(plant.num_velocities() == 6);
  DRAKE_DEMAND(plant.num_positions() == 7);

  // And build the Diagram:
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // This will set a default initial condition with the sphere located at
  // p_WBcm = (0; 0; 0) and zero spatial velocity.
  plant.SetDefaultContext(&plant_context);

  // The default initial configuration and motion of body B in World W are:
  // R_WB (B's initial rotation matrix in W) is 3x3 identity matrix,
  // p_Wo_Bcm [position from Wo (World origin) to Bcm (B's center of mass)] and
  // p_Wo_Bo  [position from Wo to Bo (B's origin)] is zero vector [0; 0; 0],
  // B is stationary in world (B's velocity and angular velocity in W are zero).
  // Overwrite B's default initial position so it contacts the inclined-plane A.
  const Vector3<double> p_WoBo_A_initial(0, 0, radius);

  // Express p_WoBo_A in terms of Wx, Wy, Wz.  To that end, remember that
  // the inclined-plane A is oriented by initially setting Ax=Wx, Ay=Wy, Az=Wz,
  // and then subjecting A to a right-handed rotation in W about Ay=Wy, so that
  // Ax is directed downhill.  Wz points locally upward (opposite gravity).
  const math::RotationMatrix<double> R_WA =
    math::RotationMatrix<double>::MakeYRotation(slope_radians);
  const Vector3<double> p_WoBo_W_initial = R_WA * p_WoBo_A_initial;
  const math::RigidTransform<double> X_WB_initial(p_WoBo_W_initial);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, ball,
    X_WB_initial.GetAsIsometry3());

  Simulator<double> simulator(*diagram, std::move(diagram_context));
  IntegratorBase<double>* integrator = simulator.get_mutable_integrator();
  integrator->set_target_accuracy(target_accuracy);
  simulator.set_publish_every_time_step(true);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

  // Compute B's kinetic energy ke in W from V_WB (B's spatial velocity in W).
  const SpatialVelocity<double>& V_WB =
      plant.EvalBodySpatialVelocityInWorld(plant_context, ball);
  const SpatialInertia<double> M_BBo_B = ball.default_spatial_inertia();
  const math::RigidTransform<double> X_WB(
      plant.EvalBodyPoseInWorld(plant_context, ball));
  const math::RotationMatrix<double>& R_WB = X_WB.rotation();
  const SpatialInertia<double> M_BBo_W = M_BBo_B.ReExpress(R_WB);
  const double ke = 0.5 * V_WB.dot(M_BBo_W * V_WB);
  const double v = V_WB.translational().norm();
  const double wy = V_WB.rotational().y();

  // Determine the distance traveled downward by sphere's centroid.
  // This change in height gives a change in potential energy pe.
  const Vector3<double> p_Wo_Bo_W = X_WB.translation();
  const double h_initial = p_WoBo_W_initial.z();
  const double h_actual = p_Wo_Bo_W.z();
  const double h = h_initial - h_actual;                  // Positive number.
  const double pe_change = -mass * gravity * h;           // Negative number.

  // Sphere's moment of inertia about is centroid is 2/5 * mass * radius^2,
  // so the sphere's unit-mass inertia is 2/5 * radius^2
  const double G_Bcm = 0.4 * radius * radius;

  // For this problem, the sum of kinetic energy ke and potential energy pe is
  // constant (ke + pe = constant) and initially ke=0, hence ke = -pe_change.
  // ke = 1/2*mass*v^2 + 1/2 * mass * G_Bcm * wy^2  =  mass * gravity * h
  // Substituting rolling wy = v/r, multiplying by 2, and dividing by mass gives
  // v^2 + G_Bcm * v^2/radius^2 = 2 * gravity * h, which when solved for v gives
  const double v_expected =
      std::sqrt(2 * gravity * h / (1.0 + G_Bcm / (radius * radius)));
  const double wy_expected = v_expected / radius;

  // Verify the relative errors. For the continuous model of the plant errors
  // are dominated by the penetration allowance and the stiction tolerance since
  // the integrator's accuracy was set to a relatively tight value.
  // For the time stepping model of the plant, errors are dominated by the
  // finite time step, given that we are able to use very tight penetration
  // allowance and stiction tolerance.
  // Notice that given the kinematic relationship between linear and angular
  // velocity v_WBcm = radius * w_WB at rolling, relative errors
  // in v_WBcm and w_WB have the same order of magnitude. Moreover, since the
  // kinetic energy scales with the velocities (translational and angular)
  // squared, standard error propagation shows that the relative error in the
  // kinetic energy is expected to be twice that in the velocities. Thus the
  // factor of two used below for the relative error in kinetic energy.
  const double delta_energy = ke + pe_change;
  EXPECT_TRUE(std::abs(delta_energy) < ke * 2 * relative_tolerance_);
  EXPECT_TRUE(std::abs(v - v_expected) < v_expected * relative_tolerance_);
  EXPECT_TRUE(std::abs(wy - wy_expected) < wy_expected * relative_tolerance_);

  // Verify the value of the contact forces when using time stepping.
  if (time_stepping_) {
    const auto& contact_forces_port =
        plant.get_generalized_contact_forces_output_port(
            default_model_instance());

    // Evaluate the generalized contact forces using the system's API.
    const VectorX<double>& tau_contact =
        contact_forces_port.Eval(plant_context);
    EXPECT_EQ(tau_contact.size(), 6);

    // Unit inertia computed about the contact point using the
    // parallel axis theorem.
    const double G_Bc = G_Bcm + radius * radius;
    // Analytical value of the friction force at the contact point.
    const double ft_expected = mass * gravity * (1.0 - radius * radius / G_Bc) *
                               std::sin(slope_radians);
    // The expected value of the moment due to friction applied to the sphere.
    const double friction_moment_expected = ft_expected * radius;

    EXPECT_TRUE(
        std::abs(friction_moment_expected - tau_contact(1))
            / friction_moment_expected < 1.0e-9);
  } else {
    // This port is not available when the plant is modeled as a continuous
    // system.
    EXPECT_THROW(plant.get_generalized_contact_forces_output_port(
        default_model_instance()), std::exception);
  }
}

// Instantiate the tests.
INSTANTIATE_TEST_CASE_P(ContinuousAndTimeSteppingTest, InclinedPlaneTest,
                        ::testing::Bool());

}  // namespace
}  // namespace multibody
}  // namespace drake

