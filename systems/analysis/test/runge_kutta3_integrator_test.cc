#include "drake/systems/analysis/runge_kutta3_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/joints/prismatic_joint.h"
#include "drake/multibody/joints/quaternion_floating_joint.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/test_utilities/explicit_error_controlled_integrator_test.h"
#include "drake/systems/analysis/test_utilities/my_spring_mass_system.h"

namespace drake {
namespace systems {
namespace analysis_test {

typedef ::testing::Types<RungeKutta3Integrator<double>> Types;
INSTANTIATE_TYPED_TEST_CASE_P(My, ExplicitErrorControlledIntegratorTest, Types);

// Tests accuracy when generalized velocity is not the time derivative of
// generalized configuration (using a rigid body).
GTEST_TEST(RK3RK2IntegratorTest, RigidBody) {
  // Instantiates a Multibody Dynamics (MBD) model of the world.
  auto tree = std::make_unique<RigidBodyTree<double>>();

  // Add a single free body with a quaternion base.
  RigidBody<double>* body;
  tree->add_rigid_body(
      std::unique_ptr<RigidBody<double>>(body = new RigidBody<double>()));
  body->set_name("free_body");

  // Sets body to have a non-zero spatial inertia. Otherwise the body gets
  // welded by a fixed joint to the world by RigidBodyTree::compile().
  body->set_mass(1.0);
  body->set_spatial_inertia(Matrix6<double>::Identity());
  body->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
      "base", Eigen::Isometry3d::Identity()));

  tree->compile();

  // Instantiates a RigidBodyPlant from the MBD model.
  RigidBodyPlant<double> plant(move(tree));
  auto context = plant.CreateDefaultContext();

  Eigen::Vector3d v0(1, 2, 3);    // Linear velocity in body's frame.
  Eigen::Vector3d w0(-4, 5, -6);  // Angular velocity in body's frame.
  BasicVector<double> generalized_velocities(plant.get_num_velocities());
  generalized_velocities.get_mutable_value() << w0, v0;

  // Set the linear and angular velocity.
  for (int i=0; i< plant.get_num_velocities(); ++i)
    plant.set_velocity(context.get(), i, generalized_velocities[i]);

  // Set a non-identity position and orientation.
  plant.set_position(context.get(), 0, 1.0);  // Set position to (1,2,3).
  plant.set_position(context.get(), 1, 2.0);
  plant.set_position(context.get(), 2, 3.0);
  plant.set_position(context.get(), 3, std::sqrt(2)/2);  // Set orientation to
  plant.set_position(context.get(), 4, 0.0);             // 90 degree rotation
  plant.set_position(context.get(), 5, std::sqrt(2)/2);  // about y-axis.
  plant.set_position(context.get(), 6, 0.0);

  // Integrate for ten thousand steps using a RK2 integrator with
  // small step size.
  const double dt = 5e-5;
  RungeKutta2Integrator<double> rk2(plant, dt, context.get());
  rk2.Initialize();
  const double t_final = 1.0;
  const int n_steps = t_final / dt;
  for (int i = 0; i< n_steps; ++i)
    rk2.IntegrateWithSingleFixedStep(dt);

  // Get the final state.
  VectorX<double> x_final_rk2 = context->get_continuous_state_vector().
      CopyToVector();

  // Re-integrate with RK3
  context->set_time(0.);
  plant.SetDefaultState(*context, &context->get_mutable_state());
  for (int i=0; i< plant.get_num_velocities(); ++i)
    plant.set_velocity(context.get(), i, generalized_velocities[i]);
  // Reset the non-identity position and orientation.
  plant.set_position(context.get(), 0, 1.0);  // Set position to (1,2,3).
  plant.set_position(context.get(), 1, 2.0);
  plant.set_position(context.get(), 2, 3.0);
  plant.set_position(context.get(), 3, std::sqrt(2)/2);  // Set orientation to
  plant.set_position(context.get(), 4, 0.0);             // 90 degree rotation
  plant.set_position(context.get(), 5, std::sqrt(2)/2);  // about y-axis.
  plant.set_position(context.get(), 6, 0.0);
  RungeKutta3Integrator<double> rk3(plant, context.get());
  rk3.set_maximum_step_size(0.1);
  rk3.set_target_accuracy(1e-6);
  rk3.Initialize();

  // Verify that IntegrateWithMultipleSteps works.
  const double tol = std::numeric_limits<double>::epsilon();
  rk3.IntegrateWithMultipleSteps(t_final - context->get_time());
  EXPECT_NEAR(context->get_time(), t_final, tol);

  // Verify that the final states are "close".
  VectorX<double> x_final_rk3 = context->get_continuous_state_vector().
      CopyToVector();
  const double close_tol = 2e-6;
  for (int i=0; i< x_final_rk2.size(); ++i)
    EXPECT_NEAR(x_final_rk2[i], x_final_rk3[i], close_tol);
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
