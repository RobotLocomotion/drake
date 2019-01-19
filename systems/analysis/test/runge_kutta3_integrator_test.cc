#include "drake/systems/analysis/runge_kutta3_integrator.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rotation_matrix.h"
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

// A testing fixture for RK3 integrators, providing a simple
// free body plant with closed form solutions to test against.
class RK3IntegratorTest : public ::testing::Test {
 protected:
  void SetUp() {
    // Instantiates a Multibody Dynamics (MBD) model of the world.
    auto tree = std::make_unique<RigidBodyTree<double>>();

    // Add a single free body with a quaternion base.
    RigidBody<double>* body = tree->add_rigid_body(
        std::make_unique<RigidBody<double>>());
    body->set_name("free_body");

    // Sets body to have a non-zero spatial inertia. Otherwise the body gets
    // welded by a fixed joint to the world by RigidBodyTree::compile().
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    body->add_joint(&tree->world(), std::make_unique<QuaternionFloatingJoint>(
        "base", Isometry3<double>::Identity()));

    tree->compile();

    // Instantiates a RigidBodyPlant from the MBD model.
    plant_ = std::make_unique<RigidBodyPlant<double>>(std::move(tree));
  }

  std::unique_ptr<Context<double>> MakePlantContext() const {
    std::unique_ptr<Context<double>> context =
        plant_->CreateDefaultContext();

    // Set body linear and angular velocity.
    Vector3<double> v0(1., 2., 3.);    // Linear velocity in body's frame.
    Vector3<double> w0(-4., 5., -6.);  // Angular velocity in body's frame.
    VectorX<double> generalized_velocities(6);
    generalized_velocities << v0, w0;
    for (int i = 0; i < plant_->get_num_velocities(); ++i)
      plant_->set_velocity(context.get(), i, generalized_velocities[i]);

    // Set body position and orientation.
    Vector3<double> p0(1., 2., 3.);  // Body's frame position in the world.
    // Set body's frame orientation to 90 degree rotation about y-axis.
    Vector4<double> q0(std::sqrt(2.)/2., 0., std::sqrt(2.)/2., 0.);
    VectorX<double> generalized_positions(7);
    generalized_positions << p0, q0;
    for (int i = 0; i < plant_->get_num_positions(); ++i)
      plant_->set_position(context.get(), i, generalized_positions[i]);

    return context;
  }

  std::unique_ptr<RigidBodyPlant<double>> plant_{};
};

// Tests accuracy when generalized velocity is not the time derivative of
// generalized configuration against an RK2 integrator.
TEST_F(RK3IntegratorTest, ComparisonWithRK2) {
  // Integrate for ten thousand steps using a RK2 integrator with
  // small step size.
  const double dt = 5e-5;
  std::unique_ptr<Context<double>> rk2_context = MakePlantContext();
  RungeKutta2Integrator<double> rk2(*plant_, dt, rk2_context.get());

  rk2.Initialize();
  const double t_final = 1.0;
  const int n_steps = t_final / dt;
  for (int i = 1; i <= n_steps; ++i)
    rk2.IntegrateWithSingleFixedStepToTime(i * dt);

  // Re-integrate with RK3.
  std::unique_ptr<Context<double>> rk3_context = MakePlantContext();
  RungeKutta3Integrator<double> rk3(*plant_, rk3_context.get());
  rk3.set_maximum_step_size(0.1);
  rk3.set_target_accuracy(1e-6);
  rk3.Initialize();

  // Verify that IntegrateWithMultipleStepsToTime works.
  const double tol = std::numeric_limits<double>::epsilon();
  rk3.IntegrateWithMultipleStepsToTime(t_final);
  EXPECT_NEAR(rk3_context->get_time(), t_final, tol);

  // Verify that the final states are "close".
  const VectorBase<double>& x_final_rk2 =
      rk2_context->get_continuous_state_vector();

  const VectorBase<double>& x_final_rk3 =
      rk3_context->get_continuous_state_vector();

  const double close_tol = 2e-6;
  for (int i = 0; i < x_final_rk2.size(); ++i)
    EXPECT_NEAR(x_final_rk2[i], x_final_rk3[i], close_tol);
}

// Tests accuracy of integrator's dense output.
TEST_F(RK3IntegratorTest, DenseOutputAccuracy) {
  std::unique_ptr<Context<double>> context = MakePlantContext();
  RungeKutta3Integrator<double> rk3(*plant_, context.get());
  rk3.set_maximum_step_size(0.1);
  rk3.set_target_accuracy(1e-6);
  rk3.Initialize();

  // Start a dense integration i.e. one that generates a dense
  // output for the state function.
  rk3.StartDenseIntegration();

  const double t_final = 1.0;
  // Arbitrary step, valid as long as it doesn't match the same
  // steps taken by the integrator. otherwise, dense output accuracy
  // would not be checked.
  const double t_step = t_final / 100.;
  for (double t = t_step; t < t_final ; t += t_step) {
    // Integrate the whole step.
    rk3.IntegrateWithMultipleStepsToTime(t);
    // Check solution.
    EXPECT_TRUE(CompareMatrices(
        rk3.get_dense_output()->Evaluate(t),
        plant_->GetStateVector(*context),
        rk3.get_accuracy_in_use(),
        MatrixCompareType::relative));
  }

  // Stop undergoing dense integration.
  std::unique_ptr<DenseOutput<double>> rk3_dense_output =
      rk3.StopDenseIntegration();
  EXPECT_FALSE(rk3.get_dense_output());

  // Integrate one more step.
  rk3.IntegrateWithMultipleStepsToTime(t_final);

  // Verify that the dense output was not updated.
  EXPECT_LT(rk3_dense_output->end_time(), context->get_time());
}

}  // namespace analysis_test
}  // namespace systems
}  // namespace drake
