#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/geometry/scene_graph.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/analysis/radau_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::SceneGraph;
using systems::BasicVector;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::RadauIntegrator;
using systems::IntegratorBase;
using systems::Simulator;

namespace multibody {
namespace {

using math::RigidTransform;
using math::RotationMatrix;

// This parameterized fixture allows us to run inclined planes tests using
// either a continuous plant model or a discrete plant model.
class InclinedPlaneTest : public ::testing::TestWithParam<bool> {
 public:
  void SetUp() override {
    // If "true", the plant is a discrete system with a fixed-step periodic
    // discrete update, otherwise the plant is modeled as a continuous system.
    time_stepping_ = GetParam();

    // The period (in seconds) of the periodic updates for the discrete plant
    // model or zero when the plant is modeled as a continuous system.
    time_step_ = time_stepping_ ? 1.0e-3 : 0.0;
  }

 protected:
  bool time_stepping_;
  double time_step_{0};  // in seconds.

  // Contact parameters. Results converge to the analytical solution as the
  // penetration allowance and the stiction tolerance go to zero.
  double penetration_allowance_{1.0e-7};  // (meters)
  double stiction_tolerance_{1.0e-5};     // (m/s)

  // Relative tolerance (unitless) used to verify the numerically computed
  // results against the analytical solution. This is about the tightest
  // tolerance we can use such that all tests pass.
  double relative_tolerance_{5.5e-4};  // dimensionless.
};

// This test creates a multibody model of a uniform-density sphere B rolling
// down an inclined plane A.  After simulating for a given length of time, this
// test verifies the numerical solution against analytical results obtained from
// an energy conservation analysis.
TEST_P(InclinedPlaneTest, RollingSphereTest) {
  DiagramBuilder<double> builder;

  // The target accuracy determines the size of the actual time steps taken
  // whenever a variable time step integrator is used.
  const double target_accuracy = 1.0e-6;

  // Length of the simulation, in seconds.
  const double simulation_time = 1;

  // Plant's parameters.
  const double radius = 0.05;   // Rolling sphere radius, [m]
  const double mass = 0.1;      // Rolling sphere mass, [kg]
  const double gravity = 9.81;  // Acceleration of gravity, [m/s²]
  const double inclined_plane_angle = 15.0 / 180 * M_PI;
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
  benchmarks::inclined_plane::AddInclinedPlaneWithSphereToPlant(
      gravity, inclined_plane_angle, std::nullopt,
      coefficient_friction_inclined_plane, coefficient_friction_sphere,
      mass, radius, &plant);

  // We should be able to set the penetration allowance pre- and post-finalize.
  // For this test we decide to set it pre-finalize.
  plant.set_penetration_allowance(penetration_allowance_);  // (in meters)

  plant.Finalize();
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

  // In the plant's default context, we assume the state of body B in world W is
  // such that X_WB is an identity transform and B's spatial velocity is zero.
  plant.SetDefaultContext(&plant_context);

  // The next several lines of code change the plant's context to overwrite B's
  // default initial position so sphere B contacts the top surface of inclined
  // plane A.  Note: The direction of unit vectors Ax, Ay, Az relative to
  // Wx, Wy, Wz is described in AddInclinedPlaneAndGravityToPlant().
  const Vector3<double> p_WoBo_A_initial(0, 0, radius);
  const RotationMatrix<double> R_WA =
    RotationMatrix<double>::MakeYRotation(inclined_plane_angle);
  const Vector3<double> p_WoBo_W_initial = R_WA * p_WoBo_A_initial;
  const RigidTransform<double> X_WB_initial(p_WoBo_W_initial);
  plant.SetFreeBodyPoseInWorldFrame(&plant_context, ball, X_WB_initial);

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  // We want an implicit integrator for this test since the computational
  // stiffness is high. 3rd order Radau is about 4x as fast as implicit
  // Euler.
  simulator.reset_integrator<RadauIntegrator<double, 2>>();

  IntegratorBase<double>& integrator = simulator.get_mutable_integrator();
  integrator.set_maximum_step_size(1e-3);    // Reasonable for this problem.
  integrator.set_target_accuracy(target_accuracy);
  simulator.set_publish_every_time_step(true);
  simulator.Initialize();
  simulator.AdvanceTo(simulation_time);

  // Compute B's kinetic energy ke in W from V_WB (B's spatial velocity in W).
  const SpatialVelocity<double>& V_WB =
      plant.EvalBodySpatialVelocityInWorld(plant_context, ball);
  const SpatialInertia<double> M_BBo_B = ball.default_spatial_inertia();
  const RigidTransform<double>& X_WB(
      plant.EvalBodyPoseInWorld(plant_context, ball));
  const RotationMatrix<double>& R_WB = X_WB.rotation();
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

  // Sphere's moment of inertia about its centroid is 2/5 * mass * radius^2,
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
  EXPECT_NEAR(ke, -pe_change, ke * 2 * relative_tolerance_);
  EXPECT_NEAR(v, v_expected, v_expected * relative_tolerance_);
  EXPECT_NEAR(wy, wy_expected, wy_expected * relative_tolerance_);

  // Verify the value of the contact forces.
  const auto& contact_forces_port =
      plant.get_generalized_contact_forces_output_port(
          default_model_instance());
  // Evaluate the generalized contact forces using the system's API.
  const VectorX<double>& tau_contact = contact_forces_port.Eval(plant_context);
  EXPECT_EQ(tau_contact.size(), 6);
  // Unit inertia computed about the contact point using the
  // parallel axis theorem.
  const double G_Bc = G_Bcm + radius * radius;
  // Analytical value of the friction force at the contact point.
  const double ft_expected = mass * gravity * (1.0 - radius * radius / G_Bc) *
                             std::sin(inclined_plane_angle);
  // The expected value of the moment due to friction applied to the sphere.
  const double friction_moment_expected = ft_expected * radius;
  EXPECT_LT(std::abs(friction_moment_expected - tau_contact(1)) /
                friction_moment_expected,
            relative_tolerance_);
}

// Instantiate the tests.
INSTANTIATE_TEST_SUITE_P(ContinuousAndTimeSteppingTest, InclinedPlaneTest,
                        ::testing::Bool());

}  // namespace
}  // namespace multibody
}  // namespace drake

